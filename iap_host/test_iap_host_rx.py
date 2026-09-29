# -*- coding: utf-8 -*-
"""IAP 上位机容错逻辑的离线自测（FakeSerial 模拟 Boot 应答，无需真实串口）。

覆盖：
1. 0xFE 进度帧被跳过，不影响紧随其后 ACK 的匹配；
2. 重传后迟到的旧响应（SEQ 不符）被丢弃；
3. CRC 损坏帧被丢弃后继续等待，不立即抛错；
4. NAK 帧正常透传给 check_ok 抛错；
5. jump_app 收到 0xA0 且角色探测为 App（新 App 应答查询）返回 True；
6. jump_app 无响应且探测也无响应（旧 App 不应答）返回 False；
7. jump_app 探测到 BOOT（仍在 Boot）抛 RuntimeError；
8. check_ok 首次超时后自动重发并成功（幂等命令重试）；
9. get_mode 从 GET_INFO 串识别 APP/BOOT 角色；
10. go_iap：收到 0xA1 后复位进 Boot -> True；
11. go_iap：旧 App NAK 0x21 -> 明确报错；
12. go_iap：回 ACK 但仍在 App（BKP 标志未生效）-> 报错。
"""
import time

import iap_host
from iap_host import (
    IAP, crc16_ccitt, be32,
    CMD_WRITE_BLK, CMD_GET_VER, CMD_GET_INFO, CMD_JUMP_APP, CMD_GO_IAP,
    ACK_WRITE, ACK_JUMP, ACK_VER, ACK_PROGRESS, ACK_INFO, ACK_GO_IAP, NAK_CMD,
    FLAG_ACK, FLAG_NAK,
)


def make_resp(cmd, seq, flag, status, data=b""):
    len_val = 2 + len(data)
    frame = bytearray([iap_host.SYNC_H, iap_host.SYNC_L,
                       (len_val >> 8) & 0xFF, len_val & 0xFF,
                       cmd, seq, flag, status]) + data
    crc = crc16_ccitt(bytes(frame[2:]))
    frame += bytes([crc & 0xFF, (crc >> 8) & 0xFF])
    return bytes(frame)


class FakeSerial:
    """responder(req_frame: bytes) -> bytes：收到主机请求时即刻把应答放入 RX 缓冲。"""

    def __init__(self, responder):
        self.responder = responder
        self.rx = bytearray()
        self.written = []

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, n):
        out = bytes(self.rx[:n])
        del self.rx[:n]
        return out

    def write(self, data):
        self.written.append(bytes(data))
        self.rx += self.responder(bytes(data))

    def flush(self):
        pass


def req_seq(frame: bytes) -> int:
    assert frame[0] == iap_host.SYNC_H and frame[1] == iap_host.SYNC_L
    return frame[5]


def test_progress_frame_skipped():
    addr = 0x08004000

    def responder(req):
        seq = req_seq(req)
        prog = make_resp(ACK_PROGRESS, seq, FLAG_ACK, 0, b"\x00\x10\x00")
        ack = make_resp(ACK_WRITE, seq, FLAG_ACK, 0, be32(addr))
        return prog + ack  # 进度帧在前，ACK 在后

    iap = IAP(FakeSerial(responder))
    flag, status, data = iap.write_blk(addr, b"\xAA" * 256)
    assert flag == FLAG_ACK, hex(flag)
    assert int.from_bytes(data[:4], "big") == addr
    print("PASS 1: 0xFE 进度帧被跳过，ACK 正常匹配")


def test_stale_seq_dropped():
    addr = 0x08004100

    def responder(req):
        seq = req_seq(req)
        stale = make_resp(ACK_WRITE, (seq - 1) & 0xFF, FLAG_ACK, 0, be32(addr - 0x100))
        good = make_resp(ACK_WRITE, seq, FLAG_ACK, 0, be32(addr))
        return stale + good  # 迟到的旧响应在前

    iap = IAP(FakeSerial(responder))
    flag, status, data = iap.write_blk(addr, b"\xBB" * 256)
    assert flag == FLAG_ACK
    assert int.from_bytes(data[:4], "big") == addr, "匹配到的是旧 SEQ 的过期帧!"
    print("PASS 2: 旧 SEQ 滞留帧被丢弃")


def test_crc_bad_frame_dropped():
    addr = 0x08004200

    def responder(req):
        seq = req_seq(req)
        bad = bytearray(make_resp(ACK_WRITE, seq, FLAG_ACK, 0, be32(addr)))
        bad[8] ^= 0xFF  # 破坏数据字节使 CRC 不过
        good = make_resp(ACK_WRITE, seq, FLAG_ACK, 0, be32(addr))
        return bytes(bad) + good

    iap = IAP(FakeSerial(responder))
    flag, status, data = iap.write_blk(addr, b"\xCC" * 256)
    assert flag == FLAG_ACK
    assert int.from_bytes(data[:4], "big") == addr
    print("PASS 3: CRC 损坏帧被丢弃，后续好帧正常匹配")


def test_nak_passthrough():
    def responder(req):
        return make_resp(NAK_CMD, req_seq(req), FLAG_NAK, 0, b"\x01")  # ERR_UNKNOWN_CMD

    iap = IAP(FakeSerial(responder))
    try:
        iap.check_ok(CMD_GET_VER)
        raise AssertionError("NAK 未被抛出")
    except RuntimeError as e:
        assert "未知命令" in str(e), str(e)
    print("PASS 4: NAK 正常透传抛错:", "NAK: 未知命令")


def test_jump_ack_ok():
    def responder(req):
        cmd = req[4]
        if cmd == CMD_JUMP_APP:
            return make_resp(ACK_JUMP, req_seq(req), FLAG_ACK, 0)
        if cmd == CMD_GET_INFO:
            # 新版 App 跳转成功后应答查询，角色为 APP
            return make_resp(ACK_INFO, req_seq(req), FLAG_ACK, 0,
                             b"CH32V203C8T6|APP:v1.0.0|Flash:64KB")
        return b""

    iap = IAP(FakeSerial(responder))
    assert iap.jump_app() is True
    print("PASS 5: 收到 0xA0 + 角色探测为 App -> jump_app() == True")


def test_jump_no_response_app_running():
    iap = IAP(FakeSerial(lambda req: b""))  # 旧版 App 不应答任何 IAP 帧
    t0 = time.time()
    assert iap.jump_app() is False
    print(f"PASS 6: 无确认且探测无响应 -> jump_app() == False ({time.time()-t0:.1f}s)")


def test_jump_still_in_boot():
    def responder(req):
        cmd = req[4]
        if cmd == CMD_GET_INFO:
            return make_resp(ACK_INFO, req_seq(req), FLAG_ACK, 0,
                             b"CH32V203C8T6|BOOT:v1.0.3|Flash:64KB")
        if cmd == CMD_GET_VER:
            return make_resp(ACK_VER, req_seq(req), FLAG_ACK, 0, b"\x01\x00")
        return b""  # JUMP_APP 帧"丢失"

    iap = IAP(FakeSerial(responder))
    try:
        iap.jump_app()
        raise AssertionError("仍在 Boot 却未报错")
    except RuntimeError as e:
        assert "仍在 Boot" in str(e), str(e)
    print("PASS 7: 探测到仍在 Boot -> 抛 RuntimeError")


def test_check_ok_retry_after_timeout():
    calls = []

    def responder(req):
        calls.append(req)
        if len(calls) == 1:
            return b""  # 第一次无响应，触发超时重发
        return make_resp(ACK_VER, req_seq(req), FLAG_ACK, 0, b"\x01\x00")

    iap = IAP(FakeSerial(responder))
    data = iap.check_ok(CMD_GET_VER, retries=3)
    assert data == b"\x01\x00"
    assert len(calls) == 2, f"应重发 1 次，实际发送 {len(calls)} 次"
    print("PASS 8: 超时后自动重发并成功")


def test_get_mode_app():
    def responder(req):
        if req[4] == CMD_GET_INFO:
            return make_resp(ACK_INFO, req_seq(req), FLAG_ACK, 0,
                             b"CH32V203C8T6|APP:v1.0.0|Flash:64KB")
        return b""

    iap = IAP(FakeSerial(responder))
    mode, info = iap.get_mode()
    assert mode == "APP" and "APP:v1.0.0" in info, (mode, info)
    print("PASS 9: get_mode 从 GET_INFO 串识别 App 角色")


def test_go_iap_success():
    state = {"boot": False}

    def responder(req):
        cmd = req[4]
        if cmd == CMD_GET_INFO:
            info = (b"CH32V203C8T6|BOOT:v1.0.3|Flash:64KB" if state["boot"]
                    else b"CH32V203C8T6|APP:v1.0.0|Flash:64KB")
            return make_resp(ACK_INFO, req_seq(req), FLAG_ACK, 0, info)
        if cmd == CMD_GO_IAP:
            state["boot"] = True   # 模拟：写 BKP 标志 + ACK + 复位进 Boot
            return make_resp(ACK_GO_IAP, req_seq(req), FLAG_ACK, 0)
        return b""

    iap = IAP(FakeSerial(responder))
    assert iap.go_iap() is True
    mode, _ = iap.get_mode()
    assert mode == "BOOT"
    print("PASS 10: GO_IAP ACK 后复位进 Boot，go_iap() == True")


def test_go_iap_rejected_by_old_app():
    def responder(req):
        cmd = req[4]
        if cmd == CMD_GET_INFO:
            return make_resp(ACK_INFO, req_seq(req), FLAG_ACK, 0,
                             b"CH32V203C8T6|APP:v0.9|Flash:64KB")
        if cmd == CMD_GO_IAP:
            return make_resp(NAK_CMD, req_seq(req), FLAG_NAK, 0, b"\x01")  # 未知命令
        return b""

    iap = IAP(FakeSerial(responder))
    try:
        iap.go_iap(attempts=1)
        raise AssertionError("旧 App NAK 未被抛出")
    except RuntimeError as e:
        assert "拒绝" in str(e), str(e)
    print("PASS 11: 旧 App NAK GO_IAP -> 明确报错")


def test_go_iap_flag_not_working():
    # App 回 ACK 但复位后仍应答 APP（模拟 BKP 标志未生效）
    def responder(req):
        cmd = req[4]
        if cmd == CMD_GET_INFO:
            return make_resp(ACK_INFO, req_seq(req), FLAG_ACK, 0,
                             b"CH32V203C8T6|APP:v1.0.0|Flash:64KB")
        if cmd == CMD_GO_IAP:
            return make_resp(ACK_GO_IAP, req_seq(req), FLAG_ACK, 0)
        return b""

    iap = IAP(FakeSerial(responder))
    try:
        iap.go_iap(attempts=2)
        raise AssertionError("始终在 App 却未报错")
    except RuntimeError as e:
        assert "未进入 Boot" in str(e), str(e)
    print("PASS 12: GO_IAP 后仍在 App -> 报错（BKP 标志未生效时可发现）")


if __name__ == "__main__":
    test_progress_frame_skipped()
    test_stale_seq_dropped()
    test_crc_bad_frame_dropped()
    test_nak_passthrough()
    test_jump_ack_ok()
    test_jump_no_response_app_running()
    test_jump_still_in_boot()
    test_check_ok_retry_after_timeout()
    test_get_mode_app()
    test_go_iap_success()
    test_go_iap_rejected_by_old_app()
    test_go_iap_flag_not_working()
    print("ALL PASS")
