# -*- coding: utf-8 -*-
"""防掉电变砖验证：模拟升级中途断电（ERASE + 部分写入后中断），
硬复位后设备必须停留在 BOOT（不能跳进残缺 App）。"""
import sys, time, serial
sys.path.insert(0, r'd:\LHM\AI\CH32V203_IAP\iap_host')
from iap_host import (IAP, load_hex_to_app_image, normalize_app_image,
                      CHUNK, APP_START, FLAG_ACK)

PORT = sys.argv[1] if len(sys.argv) > 1 else 'COM7'
HEX = r'd:\LHM\AI\CH32V203_IAP\CH32V203_RTOS_APP\obj\CH32V203_RTOS_APP.hex'

image, first, last, size = load_hex_to_app_image(HEX)
image = normalize_app_image(image, first)
print(f'[1] image {len(image)}B first={first:#010x}')

ser = serial.Serial(PORT, 115200, timeout=0.1)
iap = IAP(ser)
mode, info = iap.get_mode(retries=3)
print(f'[2] role: {mode} | {info}')
if mode == 'APP':
    iap.go_iap()
    mode, info = iap.get_mode(retries=3)
    print(f'[2] after GO_IAP: {mode}')

print('[3] ERASE_APP (写入 DIRTY 标志)...')
iap.erase()
print('    erase OK')

print('[4] START_DL + 只写前 1024B（4 x 256B 页，向量已有效，随后“断电”）...')
iap.start_dl(APP_START, 11520)
time.sleep(0.05)
for i in range(4):  # 只写 4 x 256B 整页
    addr = APP_START + i * 256
    chunk = image[i*256:(i+1)*256]
    if len(chunk) < 256:
        chunk = chunk + b'\xff' * (256 - len(chunk))
    flag, status, data = iap.write_blk(addr, chunk)
    if flag != 0x79 or status != 0:
        print(f'    write_blk failed flag={flag:#x} status={status:#x}')
        sys.exit(1)
    print(f'    wrote {addr:#x} (+256B)')
print('[5] 模拟断电：直接断开连接，不发 JUMP_APP（设备此时带有效向量 + DIRTY 标志）')

ser.close()
print('DONE')
