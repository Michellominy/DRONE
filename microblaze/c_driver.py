from pynq.overlays.base import BaseOverlay
from pynq.lib import MicroblazeLibrary
import time

base = BaseOverlay("base.bit")
lib = MicroblazeLibrary(base.PMODB, ['i2c'])

# Constants (must match C firmware)
CMD_NOP           = 0x0
CMD_START_PID     = 0x1
CMD_STOP_PID      = 0x2
CMD_SET_GAINS     = 0x3
CMD_SET_SETPOINTS = 0x4
CMD_HEARTBEAT     = 0x5
CMD_GET_LOG       = 0x6

MAILBOX_CMD_ADDR = 0xFFFC
MAILBOX_BASE     = 0xF000

Q16_ONE = 1 << 16

def float_to_q16(f: float) -> int:
    return int(f * Q16_ONE)

def q16_to_float(q: int) -> float:
    return q / Q16_ONE

class DroneController:
    def __init__(self, bitfile="base.bit", mb_program="drone_pid.elf"):
        # Load FPGA overlay
        self.base = BaseOverlay(bitfile)
        # Attach MicroBlaze
        self.mb = PynqMicroblaze(self.base.PMODB, mb_program)

    def _write_cmd(self, cmd: int, args=None):
        """Low-level: write data to mailbox then trigger command."""
        if args:
            for i, val in enumerate(args):
                self.mb.write_mailbox(i * 4, int(val))
        self.mb.write_cmd(cmd)

    def set_gains(self, roll=(1.0,0.0,0.0), pitch=(1.0,0.0,0.0), yaw=(1.0,0.0,0.0)):
        """Set PID gains (floats)."""
        args = [float_to_q16(k) for k in (*roll, *pitch, *yaw)]
        self._write_cmd(CMD_SET_GAINS, args)

    def set_setpoints(self, roll=0.0, pitch=0.0, yaw=0.0):
        """Set roll/pitch/yaw setpoints."""
        args = [float_to_q16(roll), float_to_q16(pitch), float_to_q16(yaw)]
        self._write_cmd(CMD_SET_SETPOINTS, args)

    def start_pid(self, period_ms=10, throttle=0.2):
        """Start PID loop with update period and base throttle."""
        args = [period_ms] + [0]*8 + [float_to_q16(throttle)]  # throttle in slot 9
        self._write_cmd(CMD_START_PID, args)

    def stop_pid(self):
        """Stop PID loop."""
        self._write_cmd(CMD_STOP_PID)

    def heartbeat(self):
        """Send a heartbeat to keep system alive."""
        self._write_cmd(CMD_HEARTBEAT)

    def get_log(self):
        """Fetch number of logged samples (not the data itself)."""
        self._write_cmd(CMD_GET_LOG)
        count = self.mb.read_mailbox(0)
        return count

    def set_throttle(self, throttle):
        """Adjust throttle live while PID loop is running."""
        self.mb.write_mailbox(9*4, float_to_q16(throttle))


# Example usage:
if __name__ == "__main__":
    drone = DroneController()
    drone.set_gains((1.0,0.1,0.01), (1.0,0.1,0.01), (1.0,0.1,0.01))
    drone.set_setpoints(0.0, 0.0, 0.0)
    drone.start_pid(period_ms=10, throttle=0.3)

    try:
        print("PID running. Press Ctrl-C to stop.")
    finally:
        drone.stop_pid()
