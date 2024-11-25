from constant import *


def normalize(value:float, original_min:float, original_max:float, new_min:float, new_max:float) -> float:
    """Normalizes (scales) a value to within a specific range."""
    return new_min + ((new_max - new_min) * ((value - original_min) / (original_max - original_min)))

# Convert microseconds to PWM range used by PCA9685
# Assume a 20ms period
def us_to_pwm(value_us: int) -> int:
    return int(value_us * 4096 / 20000)
