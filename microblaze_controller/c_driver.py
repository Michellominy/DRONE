from pynq.overlays.base import BaseOverlay
from pynq.lib import MicroblazeLibrary, PynqMicroblaze
import time

CMD_NOP = 0x0
CMD_INIT = 0x1
CMD_SET_GAINS = 0x2
CMD_START_PID = 0x3
CMD_STOP_PID = 0x4
MAILBOX_CMD_ADDR = 0xFFFC
MAILBOX_OFFSET = 0xF000
Q16_ONE = 1 << 16
INT32_MAX = 2**31
UINT32_MAX = 2**32

init_gains = [
    int(1.0 * Q16_ONE),  # Roll Kp
    int(0.0 * Q16_ONE),  # Roll Ki
    int(0.0 * Q16_ONE),  # Roll Kd
    int(1.0 * Q16_ONE),  # Pitch Kp
    int(0.0 * Q16_ONE),  # Pitch Ki
    int(0.0 * Q16_ONE),  # Pitch Kd
    int(1.0 * Q16_ONE),  # Yaw Kp
    int(0.0 * Q16_ONE),  # Yaw Ki
    int(0.0 * Q16_ONE),  # Yaw Kd
]

base = BaseOverlay("base.bit")
mb_info = base.iop_pmodb.mb_info
mb = PynqMicroblaze(mb_info, "/home/xilinx/pynq/lib/pmod/pid_mb.bin")

def q16_to_float(q):
    return q / Q16_ONE

def mailbox_read_signed(index):
    val = mb.read(MAILBOX_OFFSET + index*4)
    # val is in 32-bit unsigned integer format ==> Need to convert to signed
    if val > INT32_MAX:
        val -= UINT32_MAX
    return q16_to_float(val)


def mailbox_write_data(index, data):
    mb.write(MAILBOX_OFFSET + index*4, data)

def mailbox_write_cmd(cmd):
    mb.write(MAILBOX_CMD_ADDR, cmd)

def wait_for_cmd(cmd_val, timeout=2.0):
    """Wait until MAILBOX_CMD_ADDR is not equal to cmd_val or timeout."""
    start = time.time()
    while mb.read(MAILBOX_CMD_ADDR) == cmd_val:
        # if time.time() - start > timeout:
        #     break
        time.sleep(0.01)

def set_gains():
    for i, gain in enumerate(init_gains):
        mailbox_write_data(i, gain)
    mailbox_write_cmd(CMD_SET_GAINS)
    wait_for_cmd(CMD_SET_GAINS)

def init_firmware():
    mailbox_write_cmd(CMD_INIT)
    wait_for_cmd(CMD_INIT)

def start_pid_loop(set_roll=0, set_pitch=0, set_yaw=0, throttle=0.1):
    mailbox_write_data(0, int(set_roll * Q16_ONE))
    mailbox_write_data(1, int(set_pitch * Q16_ONE))
    mailbox_write_data(2, int(set_yaw * Q16_ONE))
    mailbox_write_data(3, int(throttle * Q16_ONE))
    mailbox_write_cmd(CMD_START_PID)

def stop_pid_loop():
    mailbox_write_cmd(CMD_STOP_PID)
    wait_for_cmd(CMD_STOP_PID)

if __name__ == "__main__":
    print("Setting initial PID gains...")
    set_gains()
    print("Gains set. Initializing firmware...")
    init_firmware()
    print("Firmware initialized. Starting PID loop...")
    input("Press key to start ...")

    try:
        start_pid_loop()
        print("PID loop running. Press Ctrl+C to stop.")
        while True:
            a=mb.read(MAILBOX_OFFSET + 11*4)
            b=mb.read(MAILBOX_OFFSET + 12*4)
            c=mb.read(MAILBOX_OFFSET + 13*4)
            d=mb.read(MAILBOX_OFFSET + 14*4)
            e=mb.read(MAILBOX_OFFSET + 15*4)
            f=mb.read(MAILBOX_OFFSET + 16*4)
            g=mb.read(MAILBOX_OFFSET + 17*4)
            print("Debug values: a(11)={}, b(12)={}, c(13)={}, d(14)={}, e(15)={}, f(16)={}, g(17)={}".format(a,b,c,d,e,f,g))
            
            gx = mailbox_read_signed(8)
            gy = mailbox_read_signed(9)
            gz = mailbox_read_signed(10)
            print("Gyro rates: gx={:.3f}, gy={:.3f}, gz={:.3f}".format(gx, gy, gz))
            
            t1 = mailbox_read_signed(4)
            t2 = mailbox_read_signed(5)
            t3 = mailbox_read_signed(6)
            t4 = mailbox_read_signed(7)
            print("Motor throttles: {:.3f} {:.3f} {:.3f} {:.3f}".format(t1, t2, t3, t4))
            
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping PID loop...")
        stop_pid_loop()
        print("Stopped.")
