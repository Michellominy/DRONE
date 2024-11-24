# Interface for the TCA9548A
from controller.pynq_i2c import pynq_i2c_instance
from controller.constant import TCA_DEVICE_ADDRESS

def channel_select(channel):
    """Select an individual channel."""
    if channel > 7:
        return
    pynq_i2c_instance.iic_writeByte(TCA_DEVICE_ADDRESS, 0x00, 1 << channel)
