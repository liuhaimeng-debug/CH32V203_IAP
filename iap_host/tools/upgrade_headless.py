#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
无头恢复 + 端到端验证（CH32V203_IAP）
======================================
背景：openocd 的 wch 驱动 auto-erase 为整片擦除——两次烧录后 APP 区现为
ECC 擦除态（0xE339E339），设备停在 BOOT IAP 模式。本脚本走设计内的
IAP 升级通道把 RTOS_APP 写回 APP 区（只写 0x08004000~0x0800FEFF，
物理上不碰 BOOT），CRC 校验后 JUMP_APP，最后就地用原始帧验证
修复了 vTaskDelay(2) 的 RTOS_APP 对 GET_INFO / GET_VER 的应答。
"""
import sys
import time

sys.path.insert(0, r'd:\LHM\AI\CH32V203_IAP\iap_host')

import serial
from iap_host import (IAP, load_hex_to_app_image, normalize_app_image,
                      crc16_ccitt, CHUNK,
                      APP_START, APP_WRITE_END, FLAG_ACK)

HEX = r'd:\LHM\AI\CH32V203_IAP\CH32V203_RTOS_APP\obj\CH32V203_RTOS_APP.hex'
PORT = 'COM7'


def log(msg):
    print(msg, flush=True)


def raw_query(ser, cmd, seq, wait=0.6):
    """发一帧原始请求并按 [5A A5][LEN][CMD|0x80][SEQ][FLAG][STATUS][DATA][CRC]
    解析应答，返回 (分类标签, 应答字节)。"""
    ln = 2
    fr = bytes([0x5A, 0xA5, (ln >> 8) & 0xFF, ln & 0xFF, cmd, seq])
    # CRC 范围 = LENH..DATA（跳过 5A A5 同步头），与设备侧收帧校验一致；
    # 曾误把同步头算进去导致设备 100% 丢帧、验证段全 NONE。
    c = crc16_ccitt(fr[2:])
    fr += bytes([c & 0xFF, (c >> 8) & 0xFF])
    time.sleep(0.010)
    ser.reset_input_buffer()
    ser.write(fr)
    end = time.time() + wait
    got = b''
    while time.time() < end:
        b = ser.read(64)
        if b:
            got += b
        if len(got) >= 4:
            lnv = (got[2] << 8) | got[3]
            tot = 8 + lnv
            if lnv < 2 or lnv > 300:
                break  # 显然不是有效应答
            if len(got) >= tot:
                break
    if len(got) < 4:
        return f'NONE(rx={len(got)}B)', got
    lnv = (got[2] << 8) | got[3]
    if lnv < 2 or lnv > 300:
        return 'NO_SYNC', got
    tot = 8 + lnv
    if len(got) < tot:
        return f'CLIPPED({len(got)}/{tot})', got
    fr2 = got[:tot]
    if fr2[0] != 0x5A or fr2[1] != 0xA5:
        return 'NO_SYNC', got
    ch = crc16_ccitt(fr2[2:tot - 2])
    cs = fr2[tot - 2] | (fr2[tot - 1] << 8)
    if ch != cs:
        return f'CRC_BAD(calc={ch:04X} recv={cs:04X})', got
    pl = fr2[8:tot - 2]
    return f'OK cmd={fr2[4]:02X} seq={fr2[5]:02X} data={pl[:44]!r}', got


def main():
    image, first, last, size = load_hex_to_app_image(HEX)
    log(f'[HEX] first={first:#010x} last={last:#010x} size={size}')
    # 与 GUI pick_hex() 使用同一套校验/归一逻辑：只接受执行窗口 0x00004000
    # 或编程窗口 0x08004000，非法基址直接拒绝（防止写错位置而 CRC 仍"一致"）。
    image = normalize_app_image(image, first)
    size = len(image)
    addr0 = APP_START
    log(f'[MAP] {first:#010x} -> {addr0:#010x}; App 映像 {size} 字节')
    if addr0 + size > APP_WRITE_END:
        raise SystemExit('image exceeds APP writable area')

    ser = serial.Serial(PORT, 115200, timeout=0.1)
    iap = IAP(ser)

    mode, info = iap.get_mode(retries=3)
    log(f'[ROLE] {mode} | {info}')
    if mode == 'APP':
        log('[GO_IAP] App 运行中，先请求进入 Boot（与 GUI“一键升级”相同流程）...')
        iap.go_iap()
        mode, info = iap.get_mode(retries=3)
        log(f'[ROLE] {mode} | {info}')
    if mode != 'BOOT':
        raise SystemExit('device not in BOOT/IAP — abort (manual check needed)')

    log('[1/5] ERASE_APP ...')
    iap.erase()
    log('      erase OK')

    aligned = -(-size // CHUNK) * CHUNK
    iap.start_dl(APP_START, aligned)
    log(f'[2/5] START_DL addr={APP_START:#010x} size={size} aligned={aligned} OK')
    time.sleep(0.05)

    written = 0
    last_print = 0
    while written < aligned:
        n = min(CHUNK, aligned - written)
        chunk = image[written:written + n]
        if len(chunk) < n:
            chunk = chunk + b'\xff' * (n - len(chunk))
        addr = APP_START + written
        for attempt in range(3):
            try:
                flag, status, data = iap.write_blk(addr, chunk)
            except TimeoutError:
                if attempt < 2:
                    time.sleep(0.05)
                    continue
                raise SystemExit(f'write timeout at addr={addr:#x}')
            if flag == FLAG_ACK:
                break
            if attempt == 2:
                raise SystemExit(f'NAK at addr={addr:#x} data={data}')
        written += n
        if written - last_print >= CHUNK * 16:
            last_print = written
            log(f'[3/5] {written}/{aligned} bytes')
    log(f'[3/5] written {written} bytes OK')

    padded = image + b'\xff' * (aligned - size)
    crc_host = crc16_ccitt(padded)
    d = iap.calc_crc()
    crc_dev = (d[0] | (d[1] << 8)) if len(d) >= 2 else -1
    log(f'[4/5] CRC host={crc_host:04X} dev={crc_dev:04X} -> '
        f'{"MATCH" if crc_host == crc_dev else "MISMATCH!"}')
    if crc_host != crc_dev:
        raise SystemExit('CRC mismatch — firmware not written correctly')

    jumped = iap.jump_app()
    log(f'[5/5] JUMP_APP sent (confirmed={jumped}) — waiting for APP boot ...')
    time.sleep(1.5)

    log('')
    log('=== 验证修复后 RTOS_APP 的应答（vTaskDelay(2) 实测） ===')
    # 根因说明：raw_query 曾因 CRC 覆盖同步头而 100% 发出坏帧（12×NONE）。
    # 重开串口 + 预热探测用于在正式验证前确认链路已就绪，仍保留。
    ser.close()
    time.sleep(2.0)
    ser = serial.Serial(PORT, 115200, timeout=0.1)

    warm = None
    for attempt in range(5):
        try:
            warm = IAP(ser).get_mode(retries=1)
            break
        except TimeoutError:
            time.sleep(0.5)
    log(f'[预热] 角色探测: {warm}')

    stats = {1: 0, 2: 0}
    for t in range(12):
        cmd = 1 if t % 2 == 0 else 2
        label, _ = raw_query(ser, cmd, (t + 1) & 0xFF)
        stats[cmd] += label.startswith('OK')
        name = 'GET_INFO' if cmd == 1 else 'GET_VER '
        log(f'  t{t:02d} {name} {label}')
    log(f'')
    log(f'=== stats: GET_INFO OK x{stats[1]}/6, GET_VER OK x{stats[2]}/6 ===')
    ser.close()
    if stats[1] == 6 and stats[2] == 6:
        log('RESULT: ALL PASS — APP 恢复且两查询全部成功')
    else:
        raise SystemExit('RESULT: PARTIAL FAIL — see above')


if __name__ == '__main__':
    main()
