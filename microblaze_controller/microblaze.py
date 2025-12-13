from pynq.overlays.base import BaseOverlay
from pynq.lib import PynqMicroblaze


from microblaze_controller.constant import Q16_ONE, MAILBOX_OFFSET, MAILBOX_CMD_ADDR, INT32_MAX, UINT32_MAX
from microblaze_controller.utils import q16_to_float
import time


class microblaze_driver:
    def __init__(self):
        base = BaseOverlay("base.bit")
        mb_info = base.iop_pmodb.mb_info
        self.mb = PynqMicroblaze(mb_info, "/home/xilinx/pynq/lib/pmod/pid_mb.bin")

    def mailbox_read_signed_q16(self, index):
        val = self.mb.read(MAILBOX_OFFSET + index*4)
        # val is in 32-bit unsigned integer format ==> Need to convert to signed
        if val > INT32_MAX:
            val -= UINT32_MAX
        return q16_to_float(val)

    def mailbox_read_unsigned(self, index):
        val = self.mb.read(MAILBOX_OFFSET + index*4)
        return val

    def mailbox_write_data(self, index, data):
        self.mb.write(MAILBOX_OFFSET + index*4, data)

    def mailbox_write_cmd(self, cmd):
        self.mb.write(MAILBOX_CMD_ADDR, cmd)

    def wait_for_cmd(self, cmd_val, timeout=2.0):
        """Wait until MAILBOX_CMD_ADDR is not equal to cmd_val or timeout."""
        start = time.time()
        while self.mb.read(MAILBOX_CMD_ADDR) == cmd_val:
            # if time.time() - start > timeout:
            #     break
            time.sleep(0.01)

mb_instance = microblaze_driver()
