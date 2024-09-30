from pynq_i2c import pynq_i2c_instance
from constant import *


def TCA_channel_select(channel):
    """Select an individual channel."""
    if channel > 7:
        return
    pynq_i2c_instance.iic_writeByte(TCA_DEVICE_ADDRESS, 0x00, 1 << channel)


def normalize(value:float, original_min:float, original_max:float, new_min:float, new_max:float) -> float:
    """Normalizes (scales) a value to within a specific range."""
    return new_min + ((new_max - new_min) * ((value - original_min) / (original_max - original_min)))
