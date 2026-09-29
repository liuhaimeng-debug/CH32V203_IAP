# IAP bus probe: raw-capture every response byte, non-integer-tick retry spacing
import serial, time, sys

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else ((crc << 1) & 0xFFFF)
    return crc

def make_req(cmd, seq):
    frame = bytearray([0x5A, 0xA5, 0x00, 0x02, cmd, seq])
    crc = crc16_ccitt(bytes(frame[2:6]))
    frame += bytes([crc & 0xFF, (crc >> 8) & 0xFF])
    return bytes(frame)

def classify(cmd, buf):
    if not buf:
        return "NONE"
    # find sync
    i = -1
    for k in range(len(buf) - 1):
        if buf[k] == 0x5A and buf[k+1] == 0xA5:
            i = k; break
    if i < 0:
        return f"NO_SYNC({len(buf)}B)"
    f = buf[i:]
    if len(f) < 4:
        return f"SHORT_HEAD({len(f)}B)"
    flen = 8 + ((f[2] << 8) | f[3])
    if len(f) < flen:
        return f"CLIPPED({len(f)}/{flen}B)"
    crc_calc = crc16_ccitt(bytes(f[2:flen-2]))
    crc_recv = f[flen-2] | (f[flen-1] << 8)
    if crc_calc != crc_recv:
        return f"CRC_BAD(calc={crc_calc:04X},recv={crc_recv:04X})"
    return f"OK(cmd={f[4]:02X},seq={f[5]:02X})"

port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
trials = int(sys.argv[2]) if len(sys.argv) > 2 else 30
ser = serial.Serial(port, 115200, timeout=0.05)
seq = 0
stats = {}
for t in range(trials):
    seq = (seq + 1) & 0xFF
    cmd = 0x02 if (t % 2) else 0x01   # alternate GET_INFO / GET_VER
    ser.reset_input_buffer()
    req = make_req(cmd, seq)
    time.sleep(0.010)
    t0 = time.time()
    ser.write(req); ser.flush()
    buf = bytearray()
    first_rx = None
    while time.time() - t0 < 0.4:
        n = ser.in_waiting
        if n:
            if first_rx is None:
                first_rx = (time.time() - t0) * 1000
            buf += ser.read(n)
        else:
            time.sleep(0.004)
    verdict = classify(cmd, buf)
    stats[(cmd, verdict.split('(')[0])] = stats.get((cmd, verdict.split('(')[0]), 0) + 1
    lat = f"{first_rx:6.1f}ms" if first_rx is not None else "   --- "
    print(f"t{t:02d} cmd={cmd:02X} {lat} rx={len(buf):3d}B {verdict} raw={buf.hex()}")
    time.sleep(0.13)   # non-integer-tick spacing to break phase lock
print("\n=== stats ===")
for k in sorted(stats):
    print(f"cmd={k[0]:02X} {k[1]:12s} x{stats[k]}")
ser.close()
