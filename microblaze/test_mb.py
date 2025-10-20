from pynq.overlays.base import BaseOverlay
from pynq.lib import MicroblazeLibrary, PynqMicroblaze
import time

CMD_START_PID = 0x1
CMD_STOP_PID = 0x2
MAILBOX_CMD_ADDR = 0xFFFC
MAILBOX_OFFSET = 0xF000
Q16_ONE = 1 << 16
INT32_MAX = 2**31
UINT32_MAX = 2**32

def sign_convert_32bit(unsigned_val):
    """Converts a 32-bit unsigned integer to a 32-bit signed integer."""
    if unsigned_val >= INT32_MAX: # Check if the most significant bit is set (negative)
        return unsigned_val - UINT32_MAX
    else:
        return unsigned_val

def q16_to_float(q):
    return q / Q16_ONE

base = BaseOverlay("base.bit")
mb_info = base.iop_pmodb.mb_info
mb = PynqMicroblaze(mb_info, "/home/xilinx/pynq/lib/pmod/pid_mb.bin")


# Start firmware loop
period_ms = 10
mb.write(MAILBOX_OFFSET + 0, period_ms)
mb.write(MAILBOX_OFFSET + 9*4, int(0.2 * Q16_ONE))
mb.write(MAILBOX_CMD_ADDR, CMD_START_PID)

print("Reading gyro values from MicroBlaze... Press Ctrl+C to stop.")
try:
    while True:
        raw_gx_u = mb.read(MAILBOX_OFFSET + 10*4)
        raw_gy_u = mb.read(MAILBOX_OFFSET + 11*4)
        raw_gz_u = mb.read(MAILBOX_OFFSET + 12*4)
        print(f"raw_gx_u={raw_gx_u:.2f}, raw_gy_u={raw_gy_u:.2f}, raw_gz_u={raw_gz_u:.2f}")

        raw_gx = sign_convert_32bit(raw_gx_u)
        raw_gy = sign_convert_32bit(raw_gy_u)
        raw_gz = sign_convert_32bit(raw_gz_u)
        print(f"raw_gx={raw_gx:.2f}, raw_gy={raw_gy:.2f}, raw_gz={raw_gz:.2f}")
        
        gx = q16_to_float(raw_gx)
        gy = q16_to_float(raw_gy)
        gz = q16_to_float(raw_gz)
        print(f"gx={gx:.2f}, gy={gy:.2f}, gz={gz:.2f}")
        time.sleep(0.1)
except KeyboardInterrupt:
    mb.write(MAILBOX_CMD_ADDR, CMD_STOP_PID)
    print("Stopped.")

