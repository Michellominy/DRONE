from controller.PCA import PCA_motor_to_channel, set_pwm, PCA_Init
from controller.constant import MAX_MOTOR_PWM
from utils import TCA_channel_select
from time import sleep


def MOTOR_select():
    TCA_channel_select(0)



def MOTOR_arm_all():
    PCA_Init()

    channels = [3, 4, 7, 8]
    
     # Convert microseconds to PWM range used by PCA9685
    def us_to_pwm(value_us):
        return int(value_us * 4096 / 20000)  # 20ms period
    
    min_pwm = us_to_pwm(1000)
    max_pwm = us_to_pwm(2000)
    
    for channel in channels:
        sleep(0.5)
        set_pwm(channel, 0, min_pwm)
        sleep(1)
        set_pwm(channel, 0, max_pwm)
        sleep(1)
        set_pwm(channel, 0, min_pwm)
        sleep(0.5)
        
    print("ARMING SEQUENCE DONE")

def start_motor(motor, speed_us=1000):
    channel = PCA_motor_to_channel(motor)
    
    # Convert microseconds to PWM range used by PCA9685
    def us_to_pwm(value_us):
        return int(value_us * 4096 / 20000)  # 20ms period
    
    speed_pwm = us_to_pwm(speed_us)
    
    if speed_pwm > MAX_MOTOR_PWM: speed_pwm = MAX_MOTOR_PWM

    set_pwm(channel, 0, speed_pwm)
    
