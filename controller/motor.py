from PCA import PCA_motor_to_channel, set_pwm, PCA_Init
from constant import MAX_MOTOR_DUTY_CYCLE
from utils import TCA_channel_select
from time import sleep


def MOTOR_select():
#    TCA_channel_select(0)
    print("selected MOTOR")


def MOTOR_arm_all():
#    PCA_Init()
    print("ARMING MOTOR")
    sleep(0.5)

    channels = [3, 4, 7, 8]
    
     # Convert microseconds to PWM range used by PCA9685
    def us_to_pwm(value_us):
        return int(value_us * 4096 / 20000)  # 20ms period
    
    min_pwm = us_to_pwm(1000)
    max_pwm = us_to_pwm(2000)
    
    for channel in channels:
        print(f"arming channel: {channel}")
#        sleep(0.5)
#        set_pwm(channel, 0, min_pwm)
#        sleep(1)
#        set_pwm(channel, 0, max_pwm)
#        sleep(1)
#        set_pwm(channel, 0, min_pwm)
#        sleep(0.5)
        
    print("ARMING SEQUENCE DONE")

def start_motor(motor, speed_us=1000):
#    channel = PCA_motor_to_channel(motor)

    corrected_speed_us = speed_us
    if corrected_speed_us > MAX_MOTOR_DUTY_CYCLE: corrected_speed_us = MAX_MOTOR_DUTY_CYCLE
    
    print(f"start motor: {motor} at duty cycle: {corrected_speed_us}")
    
    # Convert microseconds to PWM range used by PCA9685
    def us_to_pwm(value_us):
        return int(value_us * 4096 / 20000)  # 20ms period
    
    speed_pwm = us_to_pwm(corrected_speed_us)
#    set_pwm(channel, 0, speed_pwm)
    
