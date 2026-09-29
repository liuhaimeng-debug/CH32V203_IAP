#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GO_IAP(0x21) 端到端验证
=======================
流程：APP 在跑 → GO_IAP（APP 写 BKP 标志 → 回 0xA1 → vTaskDelay(50) → NVIC_SystemReset）
      → BOOT 启动读到标志 → 停在 IAP → 探测应识别为 BOOT
      → 再发 JUMP_APP 回到 APP，确认回程正常。
修复点：RTOS_APP 的 IAP_HandleGoIap 原本调用 Delay_Ms(50)（未 Delay_Init，
        且改写 SysTick->CMP/关计数 → 内核节拍死亡、任务永久阻塞），
        改为 vTaskDelay(50)。
"""
import sys
import time

sys.path.insert(0, r'd:\LHM\AI\CH32V203_IAP\iap_host')

import serial
from iap_host import IAP

PORT = 'COM7'


def main():
    ser = serial.Serial(PORT, 115200, timeout=0.1)
    time.sleep(0.3)
    iap = IAP(ser)

    mode, info = iap.get_mode(retries=3)
    print(f'[1] 当前角色: {mode} | {info}')
    if mode != 'APP':
        raise SystemExit('FAIL: 测试需从 APP 模式开始（当前非 APP）')

    print('[2] 发送 GO_IAP(0x21) ...')
    ok = iap.go_iap()
    print(f'[2] go_iap() -> {ok}   (True = 已确认进入 BOOT)')
    if not ok:
        raise SystemExit('FAIL: GO_IAP 未进入 BOOT')

    mode2, info2 = iap.get_mode(retries=3)
    print(f'[3] 复位后角色: {mode2} | {info2}')
    if mode2 != 'BOOT':
        raise SystemExit('FAIL: 复位后未处于 BOOT')

    print('[4] 发送 JUMP_APP 回 APP ...')
    res = iap.jump_app()
    print(f'[4] jump_app() -> {res}')
    time.sleep(0.5)

    try:
        mode3, info3 = iap.get_mode(retries=3)
    except Exception as ex:
        mode3, info3 = 'NONE', str(ex)
    print(f'[5] 最终角色: {mode3} | {info3}')
    ser.close()

    if mode3 == 'APP':
        print('RESULT: PASS —— GO_IAP 进 BOOT、JUMP_APP 回 APP 全部正常')
    else:
        raise SystemExit('RESULT: 回程异常（见 [5]）')


if __name__ == '__main__':
    main()
