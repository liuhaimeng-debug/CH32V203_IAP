#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CH32V203 IAP 上位机
====================
基于 CH32V203_BOOT 的 IAP 通讯协议 v2（见 Bsp/inc/bsp_iap.h / bsp_iap.c）。

协议要点
--------
请求帧:  [5A A5][LENH LENL][CMD][SEQ][PAYLOAD...][CRC_L CRC_H]
          LEN  = 2 (CMD+SEQ) + PAYLOAD
          CRC16 覆盖 [LENH..PAYLOAD 末尾]，即 LEN+2 字节
响应帧:  [5A A5][LENH LENL][CMD|0x80][SEQ][FLAG][STATUS][DATA...][CRC_L CRC_H]
          LEN  = 2 (FLAG+STATUS) + DATA
          真实帧长 = 8 + LEN = 2(sync)+2(LEN)+1(CMD)+1(SEQ)+LEN+2(CRC)
          FLAG = 0x79 ACK / 0x1F NAK；NAK 时 DATA 携带 1 字节错误码
CRC16:   CCITT-FALSE, poly 0x1021, init 0xFFFF

Flash 分区（64KB 芯片）
------------------------
BOOT:  0x08000000
APP :  0x08004000 ~ 0x08010000   (48KB 区，全为可写区，无保留页)
APP 可写区: 0x08004000 ~ 0x0800FEFF (47.5KB)

命令
----
GET_INFO  0x01      -> 回 0x81 (设备信息字符串)
GET_VER   0x02      -> 回 0x82 (协议版本 2 字节)
ERASE_APP 0x03      -> 回 0x83 (擦除 App 区)
START_DL  0x10      -> 回 0x83   载荷: [Addr:4B][Size:4B] 大端
WRITE_BLK 0x11      -> 回 0x91   载荷: [Addr:4B][Data:N], 单包 N<=120
CALC_CRC  0x12      -> 回 0x92   数据: [CRC_L CRC_H]
JUMP_APP  0x20      -> 回 0xA0   写 Flag 后跳转
ACK_PROGRESS 0xFE   -> 每 16 包上报一次进度 [written:3B] 大端
NAK_CMD    0xF0     -> 数据: [errcode]
GO_IAP     0x21     -> 回 0xA1（仅 App 支持）：写 BKP 标志后复位进 Boot IAP
App 角色   GET_INFO 回 "...|APP:vX|..."；擦写类命令回 NAK 0x0B（角色不符）
"""

import queue
import threading
import time

import serial
import serial.tools.list_ports
from serial.tools import list_ports


# ===================== 协议常量（与 bsp_iap.h 保持一致） =====================
SYNC_H, SYNC_L = 0x5A, 0xA5
FLAG_ACK, FLAG_NAK = 0x79, 0x1F

CMD_GET_INFO = 0x01
CMD_GET_VER  = 0x02
CMD_ERASE_APP = 0x03
CMD_START_DL = 0x10
CMD_WRITE_BLK = 0x11
CMD_CALC_CRC = 0x12
CMD_JUMP_APP = 0x20
CMD_GO_IAP = 0x21

ACK_INFO, ACK_VER, ACK_OK = 0x81, 0x82, 0x83
ACK_WRITE, ACK_CRC, ACK_JUMP = 0x91, 0x92, 0xA0
ACK_GO_IAP = 0xA1
ACK_PROGRESS = 0xFE
NAK_CMD = 0xF0

ERR_MAP = {
    0x01: "未知命令", 0x02: "CRC 校验失败", 0x03: "地址越界",
    0x04: "长度错误", 0x05: "擦除失败", 0x06: "写入失败",
    0x07: "序列号错误", 0x08: "缓冲区溢出", 0x09: "超时",
    0x0A: "App 未就绪", 0x0B: "命令不支持（当前角色）",
}

APP_START = 0x08004000
APP_END   = 0x08010000
APP_WRITE_END = APP_END                          # 48KB 全为 App 可写区（无保留页）
APP_EXEC_BASE = 0x00004000   # App 链接/执行窗口（0x08004000 的别名，BOOT 跳转执行用；
                             # CH32V203 代码只能在 0x00000000 窗口运行，官方 EVT 同款）
CHUNK = 256                                      # 单包数据 = 256B 整页（FLASH_ROM_WRITE 要求 256B 整数倍）

# RS-485 半双工换向间隙：Boot 的 Usart4_Send 在应答帧前后各 Delay_Ms(2) 切换方向，
# 且每 16 包在 ACK 后紧跟一帧 0xFE 进度上报（连发再占线 ~5ms），即 Boot 发出 ACK
# 最后一字节后总线最长仍被独占约 7ms。若上位机收到应答立刻回发，请求帧会撞进
# Boot 的发送窗口被总线竞争打烂（Boot 侧 CRC 不过静默丢帧 -> 上位机超时）。
# 发送前固定留 10ms 间隙即可覆盖正常应答与"ACK+进度帧"两种情形。
TX_GUARD_S = 0.010

# 各请求命令对应的期望响应码（注意 START_DL 回的是 ACK_OK=0x83 而非 0x90，见协议文档 §3）
EXPECT_RESP = {
    CMD_GET_INFO: ACK_INFO,
    CMD_GET_VER: ACK_VER,
    CMD_ERASE_APP: ACK_OK,
    CMD_START_DL: ACK_OK,
    CMD_WRITE_BLK: ACK_WRITE,
    CMD_CALC_CRC: ACK_CRC,
    CMD_JUMP_APP: ACK_JUMP,
    CMD_GO_IAP: ACK_GO_IAP,
}


# ===================== 工具函数 =====================
def crc16_ccitt(data: bytes) -> int:
    """CRC-16-CCITT-FALSE, poly 0x1021, init 0xFFFF。"""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def le32(data: bytes) -> int:
    """Boot 侧地址/长度字段为低字节在前（4 字节完整 32 位）。"""
    return int.from_bytes(data[:4], "little") if len(data) >= 4 else 0


def be32(v: int) -> bytes:
    """Boot 侧 START_DL 载荷为大端（4 字节完整 32 位）。"""
    return bytes([(v >> 24) & 0xFF, (v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF])


def load_hex_to_app_image(hex_path: str):
    """
    解析 Intel HEX，返回 (image_bytes, first_addr, last_addr, size)。
    支持扩展基址记录（type 2 段基址 / type 4 线性基址 / type 5 偏移基址）。
    WCH 工具非标准兼容：
      - type 4 cnt=2：2 字节为高 16 位 base，绝对地址 = (base<<16)|vma
      - type 4/5 cnt=4：标准 32 位 base，绝对地址 = base + vma
    image_bytes 为从 first_addr 起对齐的连续字节（空洞填 0）。
    """
    segments = []   # (base_addr, bytes)
    seg_base = 0   # type 2 段基址（32 位，高 16 位有效）
    off_base = 0   # type 4/5 偏移基址（32 位）
    wch_hi16 = 0   # WCH 非标准 type 4 cnt=2 的高 16 位 base

    with open(hex_path, "r") as f:
        for lineno, raw in enumerate(f, 1):
            raw = raw.strip()
            if not raw or raw[0] != ":":
                continue
            h = raw[1:]
            if len(h) < 10:
                continue
            full = bytes.fromhex(h)
            # 校验：全字节和 mod 256 应为 0（含校验和字节）
            if sum(full) % 256 != 0:
                raise ValueError(f"HEX 第 {lineno} 行校验失败: {raw}")
            rec = full[:-1]  # 去掉校验和字节
            cnt, addr, rtype = rec[0], (rec[1] << 8) | rec[2], rec[3]
            data = rec[4:4 + cnt]
            if rtype == 0:  # 数据记录
                if wch_hi16:
                    base = (wch_hi16 << 16) | addr
                else:
                    base = (seg_base | off_base) + addr
                segments.append((base, data))
            elif rtype == 2:  # 扩展段基址：addr 为高 16 位
                seg_base = addr << 16
                off_base = 0
                wch_hi16 = 0
            elif rtype in (4, 5):  # 扩展线性/偏移基址
                if cnt == 2:  # WCH 非标准：高 16 位 base
                    wch_hi16 = (data[0] << 8) | data[1]
                    off_base = 0
                elif cnt == 4:  # 标准 32 位
                    off_base = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]
                    wch_hi16 = 0
                else:
                    raise ValueError(f"HEX 第 {lineno} 行 type {rtype} 非标准长度 {cnt}")
                seg_base = 0
            # type 1/3/6 为结束/起始记录，忽略

    if not segments:
        raise ValueError("HEX 文件无有效数据记录")

    # 找出地址范围（hi 需加上每段数据长度）
    lo = min(a for a, _ in segments)
    hi = max(a + len(d) for a, d in segments)
    image = bytearray(hi - lo)
    for base, data in segments:
        image[base - lo: base - lo + len(data)] = data
    return bytes(image), lo, hi, len(image)


def normalize_app_image(image: bytes, lo: int) -> bytes:
    """把 load_hex_to_app_image() 的结果归一到编程窗口 APP_START（IAP 写 Flash 用）。

    仅接受两种已知链接基址：
      - 执行窗口 APP_EXEC_BASE (0x00004000)：新工程（官方 EVT 同款），与 0x08004000 同一物理 Flash；
      - 编程窗口 APP_START      (0x08004000)：旧工程。
    其它基址一律拒绝——否则 pick_hex 里的 max(0, base-lo) 会把映像前部静默截掉后
    写到 App 区，且主机/设备各自对"实际写入的字节"算 CRC，**CRC 仍会一致**，
    界面全绿但内容错位（最难排查的一类事故）。

    返回归一后的 App 映像（自 APP_START 起）。校验不过抛 ValueError。
    """
    if lo not in (APP_EXEC_BASE, APP_START):
        raise ValueError(
            f"HEX 起始地址 {lo:#010x} 既不是执行窗口 {APP_EXEC_BASE:#010x}，"
            f"也不是编程窗口 {APP_START:#010x}，拒绝载入（防止写错位置；"
            f"BOOT 映像请勿用本工具升级）")

    off = max(0, (APP_EXEC_BASE if lo < APP_START else APP_START) - lo)
    app_image = image[off:]

    if not app_image:
        raise ValueError("归一到 App 区后映像为空")
    if APP_START + len(app_image) > APP_WRITE_END:
        raise ValueError(
            f"映像 {len(app_image)} 字节超出 App 可写区 "
            f"({APP_START:#010x}~{APP_WRITE_END:#010x})")
    return app_image


# ===================== 协议帧构造/解析 =====================
class IAP:
    """同步的 IAP 收发器（在 worker 线程内使用）。"""

    def __init__(self, ser: serial.Serial):
        self.ser = ser
        self._seq = 0
        self._tx_seq = 0   # 最近一次请求实际发送的 SEQ（用于丢弃重传后迟到的旧响应）

    # ---- 底层 ----
    def _next_seq(self):
        self._seq = (self._seq + 1) & 0xFF
        return self._seq

    def _read_response(self, expect_cmd: int, timeout=2.0):
        """
        读取一个与 expect_cmd 匹配的响应帧并做基本校验。
        返回 (cmd, seq, flag, status, data)。
        使用累积缓冲，即使响应帧跨多次串口到达也能完整拼出。
        容错策略（防止响应流错位）：
        - 跳过 Boot 每 16 包主动上报的 0xFE 进度帧；
        - 跳过 SEQ 与本次请求不符的滞留旧帧（重传后迟到的旧响应）；
        - 跳过 CMD 不符的无关帧；NAK(0xF0) 不过滤，交上层按错误码处理；
        - CRC 损坏的帧（总线冲突等）丢弃后继续等，不立即抛错。
        """
        buf = bytearray()
        deadline = time.time() + timeout
        expect_resp = EXPECT_RESP.get(expect_cmd)
        while True:
            if time.time() > deadline:
                raise TimeoutError("等待响应超时")
            if self.ser.in_waiting > 0:
                buf += self.ser.read(self.ser.in_waiting)

            while True:  # 尽量消费缓冲里已有的完整帧
                idx = self._find_sync(buf)
                if idx is None:
                    # 没找到完整同步头（可能 5A 在末尾），只保留最后 1 字节
                    if len(buf) > 1:
                        del buf[:-1]
                    break
                if idx > 0:
                    del buf[:idx]   # 丢弃同步头之前的垃圾字节
                # 需先拿到 LENH LENL 才知整帧多长
                if len(buf) < 4:
                    break  # 等更多字节
                len_val = (buf[2] << 8) | buf[3]
                # 响应帧长度 = 2(sync) + 2(LENH LENL) + 1(CMD) + 1(SEQ) + len_val(FLAG+STATUS+DATA) + 2(CRC)
                frame_len = 8 + len_val
                if len(buf) < frame_len:
                    break  # 半帧未凑齐，等更多字节
                frame = bytes(buf[:frame_len])
                del buf[:frame_len]
                try:
                    cmd, seq, flag, status, data = self._parse_response(frame)
                except ValueError:
                    continue  # CRC 损坏帧，丢弃继续解析后续字节
                if cmd == ACK_PROGRESS:
                    continue  # Boot 每 16 包主动上报的进度帧，跳过
                if seq != self._tx_seq:
                    continue  # 重传后迟到的旧响应，丢弃
                if cmd != expect_resp and cmd != NAK_CMD:
                    continue  # 与本次请求无关的帧，丢弃
                return cmd, seq, flag, status, data

            time.sleep(0.005)

    @staticmethod
    def _find_sync(buf: bytearray):
        for i in range(len(buf) - 1):
            if buf[i] == SYNC_H and buf[i + 1] == SYNC_L:
                return i
        return None

    @staticmethod
    def _parse_response(frame: bytes):
        # frame: [5A A5][LENH LENL][CMD][SEQ][FLAG][STATUS][DATA...][CRC_L CRC_H]
        lenh, lenl = frame[2], frame[3]
        payload_len = (lenh << 8) | lenl   # = FLAG(1)+STATUS(1)+DATA
        cmd = frame[4]
        seq = frame[5]
        flag = frame[6]
        status = frame[7]
        plen = payload_len - 2            # DATA 长度（LEN - FLAG - STATUS）
        data = frame[8:8 + plen] if plen else b""
        # CRC 范围：LENH..DATA 末尾（与 Boot IAP_SendFrame 一致）
        crc_calc = crc16_ccitt(frame[2:8 + plen])
        crc_stored = frame[8 + plen] | (frame[8 + plen + 1] << 8)
        if crc_calc != crc_stored:
            raise ValueError(f"CRC 校验失败 cmd=0x{cmd:02X} calc={crc_calc:04X} recv={crc_stored:04X}")
        return cmd, seq, flag, status, data

    # ---- 各命令 ----
    def request(self, cmd: int, payload: bytes = b""):
        """通用请求：返回 (cmd, seq, flag, status, data)。"""
        seq = self._next_seq()
        body = bytes([cmd, seq]) + payload
        # 请求帧 LEN = CMD + SEQ + PAYLOAD = 2 + len(payload)
        length = 2 + len(payload)
        frame = bytearray()
        frame.append(SYNC_H)
        frame.append(SYNC_L)
        frame.append((length >> 8) & 0xFF)
        frame.append(length & 0xFF)
        frame += body
        # Boot 侧 USART4_DataPack_Process：CRC 覆盖 LENH LENL CMD SEQ PAYLOAD
        # = frame[2 : 4 + length]，共 4 + length 字节（length = CMD+SEQ+PAYLOAD）
        crc = crc16_ccitt(bytes(frame[2:4 + length]))
        frame.append(crc & 0xFF)
        frame.append((crc >> 8) & 0xFF)
        self._tx_seq = seq
        time.sleep(TX_GUARD_S)   # RS-485 换向间隙（见模块常量区 TX_GUARD_S 说明）
        self.ser.flush()
        self.ser.write(bytes(frame))

        return self._read_response(cmd)

    def check_ok(self, cmd: int, payload: bytes = b"", retries: int = 3):
        """同 request，但响应超时自动原地重发（这些命令均幂等），遇 NAK 直接抛错。"""
        for attempt in range(retries):
            try:
                cmd_r, seq_r, flag, status, data = self.request(cmd, payload)
            except TimeoutError:
                if attempt < retries - 1:
                    time.sleep(0.05)   # 等总线彻底空闲后原地重发
                    continue
                raise
            if flag == FLAG_NAK:
                err = data[0] if data else status
                # Boot 诊断版 NAK 可能带附加数据（如擦除失败: [0x05][STATR][失败页序号]）
                detail = f" data={[hex(b) for b in data]}" if len(data) > 1 else ""
                raise RuntimeError(f"NAK: {ERR_MAP.get(err, f'0x{err:02X}')}{detail}")
            return data

    # 便捷封装
    def get_info(self):
        return self.check_ok(CMD_GET_INFO)

    def get_ver(self):
        return self.check_ok(CMD_GET_VER)

    def get_mode(self, retries=3):
        """GET_INFO 判断当前运行角色：'BOOT' / 'APP' / 'UNKNOWN'，并返回信息串。

        Boot 应答 "...|BOOT:vX|..."，App 应答 "...|APP:vX|..."（App 也实现协议查询）。
        """
        info = self.check_ok(CMD_GET_INFO, retries=retries).decode("ascii", "replace")
        if "|BOOT:" in info:
            return "BOOT", info
        if "|APP:" in info:
            return "APP", info
        return "UNKNOWN", info

    def go_iap(self, attempts=4):
        """请求 App 复位进入 Boot IAP（GO_IAP 0x21）。

        流程：探测角色 → 在 App 时发 0x21（App 写 BKP 标志后复位）→ 等复位
        → 再次探测；返回 True = 已确认进入 Boot。
        ACK 可能因复位时序丢失，因此以“探测到 BOOT”为成功判据。
        """
        mode = None
        for _ in range(attempts):
            try:
                mode, _ = self.get_mode(retries=1)
            except TimeoutError:
                mode = None
            if mode == "BOOT":
                return True
            if mode == "APP":
                try:
                    self.check_ok(CMD_GO_IAP, retries=1)
                except TimeoutError:
                    pass  # ACK 丢失但设备可能已在复位
                except RuntimeError as ex:
                    raise RuntimeError(
                        f"GO_IAP 被设备拒绝（App 固件可能不支持 0x21 升级入口）: {ex}") from ex
            time.sleep(0.6)   # 等复位 + Boot 初始化（BKP 标志由 Boot 启动时消费）
        try:
            mode, _ = self.get_mode(retries=1)
        except TimeoutError:
            mode = None
        if mode == "BOOT":
            return True
        raise RuntimeError(f"GO_IAP 失败：设备未进入 Boot 模式（当前: {mode or '无响应'}）")

    def erase(self):
        self.check_ok(CMD_ERASE_APP)

    def start_dl(self, addr, size):
        data = self.check_ok(CMD_START_DL, be32(addr) + be32(size))
        # 新 Boot 会回显解析出的 addr/size（8 字节大端 = 4+4），打印供核对
        if len(data) == 8:
            echo_addr = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]
            echo_size = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7]
            print(f"[START_DL 回显] Boot 解析 addr={echo_addr:#010x} size={echo_size}"
                  f"  (期望 addr={addr:#010x} size={size})  -> "
                  f"{'一致' if (echo_addr, echo_size) == (addr, size) else '不一致!'}")
        else:
            print(f"[START_DL 回显] 旧 Boot（无回显，数据长度={len(data)}）— 固件可能未刷入")

    def write_blk(self, addr, data):
        _cmd, _seq, flag, status, d = self.request(CMD_WRITE_BLK, be32(addr) + data)
        return flag, status, d

    def calc_crc(self):
        return self.check_ok(CMD_CALC_CRC)

    def jump_app(self):
        """发送跳转命令。返回 True=已确认不在 Boot（新 App 应答查询或确认已跳走）；
        False=设备无响应（视为已跳入不支持查询的旧 App）。

        ACK(0xA0) 可能丢失而 Boot 实际已跳走，所以以“GET_INFO 角色探测”为准：
        - 探测到 BOOT → 跳转失败（真失败，抛错可重试）；
        - 探测到 APP（新版 App 应答查询）→ 跳转成功；
        - 无响应 → 旧版 App 不实现协议，视为已跳走。
        """
        try:
            self.check_ok(CMD_JUMP_APP, retries=1)
        except TimeoutError:
            pass          # 0xA0 丢失，Boot 可能仍执行了跳转，交给探测判定
        time.sleep(0.3)   # Boot 回 ACK 后 Delay 50ms 才跳，留足余量
        try:
            mode, _ = self.get_mode(retries=1)
        except TimeoutError:
            return False  # 无响应 = 旧版 App（不应答协议）或正处于复位窗口
        if mode == "BOOT":
            raise RuntimeError("跳转 App 失败：设备仍在 Boot 模式，请重试")
        return True


# ===================== 更新流程（供 worker 调用） =====================
def run_full_update(iap: IAP, image: bytes, addr0: int, progress_q, log_q):
    """执行 擦除 -> 逐包写入 -> CRC 校验 -> 跳转 的完整流程。"""
    size = len(image)
    if addr0 < APP_START or addr0 + size > APP_WRITE_END:
        log_q.put(f"[错误] 地址范围 {hex(addr0)}~{hex(addr0+size)} 超出可写区 {hex(APP_START)}~{hex(APP_WRITE_END)}")
        raise ValueError("地址范围超出 App 可写区")

    log_q.put(f"擦除 App 区 (可写 {APP_WRITE_END - APP_START} 字节)...")
    iap.erase()
    log_q.put("擦除完成，开始写入")
    # size 按 256B 向上取整申报给 Boot，使 g_iap_dl_size 为 256B 整数倍，
    # 最后一块补 0xFF 后 g_iap_dl_written 恰好等于取整值，边界校验不越界。
    aligned_size = -(-size // CHUNK) * CHUNK   # ceil 到 256 的整数倍
    log_q.put(f"发送 START_DL addr={addr0:#010x} size={size} (对齐 {aligned_size})...")
    iap.start_dl(addr0, aligned_size)
    log_q.put("START_DL 确认")
    # RS-485 半双工：给收发器/总线留出发送->接收切换时间，
    # 避免紧随其后的 WRITE_BLK 长帧撞在 Boot 发送尾窗口上丢帧头
    time.sleep(0.05)

    written = 0
    off = 0
    base_addr = addr0
    while written < aligned_size:
        n = min(CHUNK, aligned_size - written)
        chunk = image[off:off + n] if off < size else b""
        if len(chunk) < n:
            chunk = chunk + b"\xff" * (n - len(chunk))   # 末块 0xFF 补齐到整页
        addr = base_addr + off
        for attempt in range(3):
            try:
                flag, status, data = iap.write_blk(addr, chunk)
            except TimeoutError:
                # Boot 无应答（帧被总线毛刺/收发切换吃掉）。
                # 写指针未推进时重发同 addr 是幂等的
                if attempt < 2:
                    log_q.put(f"[重试 {attempt+1}] 响应超时，重发 addr={addr:#x}")
                    time.sleep(0.05)
                    continue
                raise RuntimeError(f"写入失败: addr={addr:#x} 多次响应超时")
            if flag == FLAG_ACK:
                # Boot 回显地址为 4 字节大端（与 bsp_iap.c wr_ack 一致）
                echo_addr = int.from_bytes(data[:4], "big") if len(data) >= 4 else None
                if echo_addr is not None and echo_addr != addr:
                    log_q.put(f"[警告] 写回地址 {hex(echo_addr)} 与期望 {hex(addr)} 不符，重连重来")
                    raise RuntimeError("地址错位，需重新执行")
                break
            elif flag == FLAG_NAK:
                err = data[0] if data else status
                log_q.put(f"[NAK 详情] addr={addr:#x} data_len={len(data)} data={[hex(b) for b in data[:4]]} err_code=0x{err:02x} ({ERR_MAP.get(err, '未知')})")
                if attempt < 2:
                    log_q.put(f"[重试 {attempt+1}] NAK {ERR_MAP.get(err, hex(err))}")
                    time.sleep(0.1)
                else:
                    raise RuntimeError(f"写入失败: {ERR_MAP.get(err, hex(err))}")
            else:
                raise RuntimeError(f"未知 FLAG 0x{flag:02X}")
        written += n
        off += n
        progress_q.put(min(written, size) / size)
        # Boot 每 16 包发 0xFE 进度，这里忽略；本地进度已足够

    log_q.put(f"写入完成 {written} 字节，开始 CRC 校验")
    # Boot 的 CALC_CRC 按 g_iap_dl_written（= aligned_size，含末页 0xFF 补齐）计算，
    # 主机侧须对"补齐后的完整字节"算 CRC，二者范围才一致。
    padded_image = image + b"\xff" * (aligned_size - size)
    crc_host = crc16_ccitt(padded_image)
    # Boot 侧 CALC_CRC 返回 [CRC低字节, CRC高字节]（小端，仅 2 字节）。
    # 注意不能用 le32()：它要求 len>=4，2 字节会返回 0 导致设备 CRC 恒为 0x0000。
    crc_data = iap.calc_crc()
    crc_fw = (crc_data[0] | (crc_data[1] << 8)) if len(crc_data) >= 2 else 0
    ok = "一致" if crc_host == crc_fw else "不一致"
    log_q.put(f"CRC 主机={crc_host:04X} 设备={crc_fw:04X} -> {ok}")
    if crc_host != crc_fw:
        raise RuntimeError("CRC 不一致，固件未正确写入")

    log_q.put("跳转 App")
    if iap.jump_app():
        log_q.put("[完成] 固件更新成功，已跳入 App")
    else:
        log_q.put("[完成] 跳转指令已发送，设备无响应（应已跳入 App；若 App 支持查询可点“查询状态”复核）")


# ===================== GUI =====================
# 非 GUI 环境（如协议自测）也允许 import 本模块：提供 tkinter 常量/类的占位。
# 真正运行时 _gui() 会用真实 tkinter 对象覆盖这些全局。
class _Dummy:
    def __init__(self, *a, **k):
        pass
    def __getattr__(self, name):
        return _Dummy()

# 默认值，_gui() 会用真实 tkinter 常量覆盖
DISABLED = "disabled"
NORMAL = "normal"
W = "w"
X = "x"
BOTH = "both"
END = "end"
Tk = Frame = Label = Entry = Button = Combobox = StringVar = Text = Scrollbar = _Dummy
ttk = _Dummy()
filedialog = _Dummy()


def _gui():
    global Tk, Frame, Label, Entry, Button, Combobox, StringVar, Text, END
    global X, Y, BOTH, RIGHT, DISABLED, NORMAL, W, Scrollbar, ttk, filedialog
    global L, R, NONE, tkinter
    import tkinter
    import tkinter.ttk as _ttk
    from tkinter import (
        Tk, Frame, Label, Entry, Button, StringVar,
        Text, END, X, Y, BOTH, RIGHT, DISABLED, NORMAL, W, Scrollbar,
        NONE,
    )
    from tkinter import filedialog
    Combobox = _ttk.Combobox
    Tk, Frame, Label, Entry, Button, StringVar, Text, END = \
        Tk, Frame, Label, Entry, Button, StringVar, Text, END
    X, Y, BOTH, RIGHT, DISABLED, NORMAL, W, Scrollbar, NONE = \
        X, Y, BOTH, RIGHT, DISABLED, NORMAL, W, Scrollbar, NONE
    ttk, filedialog = _ttk, filedialog
    L, R = tkinter.LEFT, tkinter.RIGHT  # 布局常量别名


class App:
    def __init__(self, root):
        self.root = root
        root.title("CH32V203 IAP 上位机")
        root.geometry("520x540")

        self.ser = None
        self.worker = None
        self.log_q = queue.Queue()
        self.prog_q = queue.Queue()
        self.hex_path = ""
        self._busy = False

        # ---- 串口连接区 ----
        f1 = ttk.LabelFrame(root, text="串口连接")
        f1.pack(fill=X, padx=8, pady=4)
        self.port_var = StringVar()
        self.baud_var = StringVar()
        self.port_cb = Combobox(f1, textvariable=self.port_var, state="readonly", width=8)
        self.baud_cb = Combobox(f1, textvariable=self.baud_var, state="readonly", width=8,
                                 values=[9600, 115200, 230400, 460800, 921600])
        btn_refresh = Button(f1, text="刷新", width=6, command=self.refresh_ports)
        self.btn_conn = Button(f1, text="连接", width=6, command=self.toggle_conn)

        # 用 grid 精确对齐：
        #   第0行：[端口:] (col 0-1)  [波特率:] (col 2-3)
        #   第1行：[COM…] (col 0) [刷新] (col 1) [115200] (col 2) [连接] (col 3)
        # col 1 与 col 2 之间用固定 16px 间距分隔两组，避免"波特率:"与"刷新"重合
        f1.columnconfigure(0, weight=1, uniform="port")
        f1.columnconfigure(1, minsize=16)
        f1.columnconfigure(2, weight=1, uniform="baud")

        ttk.Label(f1, text="端口:").grid(row=0, column=0, sticky="w")
        ttk.Label(f1, text="波特率:").grid(row=0, column=2, sticky="w")
        self.port_cb.grid(row=1, column=0, sticky="we", padx=(0, 4))
        btn_refresh.grid(row=1, column=1, sticky="w", padx=(0, 4))
        self.baud_cb.grid(row=1, column=2, sticky="we", padx=(4, 4))
        self.btn_conn.grid(row=1, column=3, sticky="w")

        self.port_cb.set("COM1")
        self.baud_cb.set("115200")
        self.refresh_ports()

        # ---- 固件文件区 ----
        f2 = ttk.LabelFrame(root, text="固件文件")
        f2.pack(fill=X, padx=8, pady=4)
        self.hex_var = StringVar()
        # 总页数在 pick_hex 后计算
        self._prog_total = 0
        self._last_image = None
        self._last_lo = 0
        e = Entry(f2, textvariable=self.hex_var, width=40)
        e.pack(side=L, fill=X, expand=True)
        Button(f2, text="浏览...", width=8, command=self.pick_hex).pack(side=L, padx=4)

        # ---- 功能按钮区 ----
        # 日常使用只需两个操作：
        #   查询状态 = 运行角色 + 设备信息 + 协议版本（GET_INFO + GET_VER）
        #   一键升级 = 自动判断角色（App 则先 GO_IAP 进 Boot）-> 擦除 -> 写入
        #              -> CRC 校验 -> 跳转 App，覆盖原 ①~④ 单步按钮的全部功能。
        # 单步命令（erase/start_dl/write_blk/calc_crc/jump_app/go_iap）仍保留在
        # IAP 类中供 run_full_update 与脚本调用，调试救砖时可用命令行操作。
        f3 = ttk.LabelFrame(root, text="操作")
        f3.pack(fill=X, padx=8, pady=4)
        self.btn_query = Button(f3, text="查询状态", width=12, command=lambda: self.action("query"))
        self.btn_upg   = Button(f3, text="一键升级", width=12, command=lambda: self.action("upgrade"))
        self.btn_query.pack(side=L, padx=8, pady=4, expand=True)
        self.btn_upg.pack(side=L, padx=8, pady=4, expand=True)

        # ---- 进度条 ----
        self.progress = ttk.Progressbar(root, mode="determinate", maximum=100)
        self.progress.pack(fill=X, padx=8, pady=2)
        self.prog_lbl = Label(root, text="0 / 0 页", anchor=W)
        self.prog_lbl.pack(fill=X, padx=8)

        # ---- 日志区 ----
        f4 = ttk.LabelFrame(root, text="日志")
        f4.pack(fill=BOTH, expand=True, padx=8, pady=4)
        # 清空日志按钮（独立一行，置于日志框上方，避免被右侧滚动条挡住）
        self.btn_clear_log = Button(f4, text="清空日志", width=12, command=self.clear_log)
        self.btn_clear_log.pack(side=R, padx=4, pady=2)
        self.log = Text(f4, height=12, wrap=NONE, state=DISABLED)
        sb = Scrollbar(f4, command=self.log.yview)
        self.log.configure(yscrollcommand=sb.set)
        self.log.pack(side=L, fill=BOTH, expand=True, padx=(4, 0), pady=2)
        sb.pack(side=R, fill=Y, pady=2)

        # 轮询刷新
        self._poll()

    # ---------- 串口 ----------
    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_cb["values"] = ports if ports else ["COM1"]
        if self.port_var.get() not in (ports or [self.port_var.get()]):
            self.port_cb.set(ports[0] if ports else "COM1")

    def toggle_conn(self):
        if self.ser:
            try:
                self.ser.close()
            finally:
                self.ser = None
            self.btn_conn.config(text="连接")
            self.log_line("[系统] 串口已断开")
            return
        try:
            self.ser = serial.Serial(self.port_var.get(), int(self.baud_var.get()),
                                     timeout=0.1)
            self.btn_conn.config(text="断开")
            self.log_line(f"[系统] 已连接 {self.port_var.get()} @ {self.baud_var.get()}")
        except Exception as ex:
            self.log_line(f"[错误] 连接失败: {ex}")

    # ---------- 文件 ----------
    def pick_hex(self):
        path = filedialog.askopenfilename(
            title="选择固件 (.hex)",
            filetypes=[("Intel HEX", "*.hex"), ("所有文件", "*.*")])
        if path:
            self.hex_var.set(path)
            try:
                image, lo, hi, size = load_hex_to_app_image(path)
                # 兼容两种链接基址（执行窗口 0x00004000 / 编程窗口 0x08004000），
                # 校验并归一到编程窗口 APP_START，走同一套 START_DL/WRITE_BLK 流程。
                # 非法基址直接拒绝（见 normalize_app_image 说明）。
                app_image = normalize_app_image(image, lo)
                self._last_image, self._last_lo = app_image, APP_START
                app_size = len(app_image)
                self._prog_total = -(-app_size // CHUNK)  # 向上取整页数
                self.log_line(f"[系统] 载入 {path}: 原始 {size} 字节, 地址 {hex(lo)}~{hex(hi)}; App 区 {app_size} 字节 / {self._prog_total} 页")
            except Exception as ex:
                self.log_line(f"[错误] 解析 HEX 失败: {ex}")
                self._last_image = None

    # ---------- 操作 ----------
    def action(self, what):
        if self._busy:
            self.log_line("[系统] 已有操作进行中，请稍候")
            return
        if not self.ser:
            self.log_line("[错误] 请先连接串口")
            return
        self._busy = True
        self.set_buttons(state=DISABLED)
        self.worker = threading.Thread(target=self._do_action, args=(what,), daemon=True)
        self.worker.start()

    def _do_action(self, what):
        iap = IAP(self.ser)
        try:
            if what == "query":
                mode, info = iap.get_mode()
                self.log_q.put(f"运行角色: {mode}")
                self.log_q.put(f"设备信息: {info}")
                v = iap.get_ver()
                self.log_q.put(f"协议版本: {v[0]}.{v[1]}")
            elif what == "upgrade":
                if not getattr(self, "_last_image", None):
                    self.log_q.put("[错误] 未载入固件文件，请先浏览 .hex")
                    raise RuntimeError("未载入固件")
                mode, _ = iap.get_mode()
                if mode == "APP":
                    self.log_q.put("App 运行中，先 GO_IAP 进入 Boot...")
                    iap.go_iap()
                    self.log_q.put("已进入 Boot")
                run_full_update(iap, self._last_image, APP_START, self.prog_q, self.log_q)
            self.log_q.put("[系统] 操作结束")
        except TimeoutError as ex:
            # 设备无响应常见原因：接线/波特率/485 方向，或正处于 GO_IAP/跳转复位窗口。
            # 新版 App 应答查询（点"查询状态"看角色）；旧版 App 不应答则需 WCH-Link/ISP
            # 擦除 App 区使入口失效，复位后停在 Boot。
            self.log_q.put(f"[错误] {ex}（设备无响应：检查接线/波特率；设备可能正在复位，稍候重试）")
        except Exception as ex:
            self.log_q.put(f"[错误] {ex}")
        finally:
            self.log_q.put(None)  # 结束哨兵

    def set_buttons(self, state=NORMAL):
        for b in (self.btn_query, self.btn_upg):
            b.config(state=state)

    # ---------- 轮询/日志 ----------
    def _poll(self):
        # 处理日志
        try:
            while True:
                msg = self.log_q.get_nowait()
                if msg is None:
                    self._busy = False
                    self.set_buttons(state=NORMAL)
                    break
                self.log_line(msg)
        except queue.Empty:
            pass
        # 处理进度
        try:
            while True:
                p = self.prog_q.get_nowait()
                self.progress["value"] = int(p * 100)
                total = getattr(self, "_prog_total", 0)
                pages = int(p * total) if total else 0
                self.prog_lbl.config(text=f"{pages} / {total} 页")
        except queue.Empty:
            pass
        self.root.after(50, self._poll)

    def log_line(self, msg):
        self.log.config(state=NORMAL)
        self.log.insert(END, msg + "\n")
        self.log.see(END)
        self.log.config(state=DISABLED)

    def clear_log(self):
        self.log.config(state=NORMAL)
        self.log.delete(1.0, END)
        self.log.config(state=DISABLED)


if __name__ == "__main__":
    _gui()
    root = Tk()
    App(root)
    root.mainloop()
