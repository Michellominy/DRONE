from MPU import *
from motor import *
from time import sleep, time
from utils import normalize
# https://timhanewich.medium.com/how-i-developed-the-scout-flight-controller-part-7-full-flight-controller-code-4269c83b3b48

# Max attitude rate of change rates (degrees per second)
max_rate_roll:float = 30.0
max_rate_pitch:float = 30.0 
max_rate_yaw:float = 50.0 

target_cycle_hz:float = 250.0

# PID Controller values
pid_roll_kp:float = 0.00043714285
pid_roll_ki:float = 0.00255
pid_roll_kd:float = 0.00002571429
pid_pitch_kp:float = pid_roll_kp
pid_pitch_ki:float = pid_roll_ki
pid_pitch_kd:float = pid_roll_kd
pid_yaw_kp:float = 0.001714287
pid_yaw_ki:float = 0.003428571
pid_yaw_kd:float = 0.0


cycle_time_seconds:float = 1.0 / target_cycle_hz
cycle_time_us:int = int(round(cycle_time_seconds * 1000000, 0)) 
i_limit:float = 150.0

roll_last_integral:float = 0.0
roll_last_error:float = 0.0
pitch_last_integral:float = 0.0
pitch_last_error:float = 0.0
yaw_last_integral:float = 0.0
yaw_last_error:float = 0.0

throttle = 40 # in %

print("initializing MPU")
MPU_select()
MPU_Init()
print("DONE init MPU")


MOTOR_select()
MOTOR_arm_all()

try:
    while True:
        loop_begin_us: int = int(time() * 1000000)
        
        
        MPU_select()
        gyro_x, gyro_y, gyro_z = MPU_read_gyro()
        error_rate_roll:float = max_rate_roll - gyro_x
        error_rate_pitch:float = max_rate_pitch - gyro_y
        error_rate_yaw:float = max_rate_yaw - gyro_z
        print(f"gyrox: {gyro_x}, gyroy: {gyro_y}, gyroz: {gyro_z}")
        print(f"error roll: {error_rate_roll}, error pitch: {error_rate_pitch}, error yaw: {error_rate_yaw}")
        sleep(1)
        
        # roll PID calc
        roll_p:float = error_rate_roll * pid_roll_kp
        roll_i:float = roll_last_integral + (error_rate_roll * pid_roll_ki * cycle_time_seconds)
        roll_i = max(min(roll_i, i_limit), -i_limit) 
        roll_d:float = pid_roll_kd * (error_rate_roll - roll_last_error) / cycle_time_seconds
        pid_roll:float = roll_p + roll_i + roll_d
        print(f"roll PID: {pid_roll} = {roll_p} + {roll_i} + {roll_d}")
        sleep(1)
        
        # pitch PID calc
        pitch_p:float = error_rate_pitch * pid_pitch_kp
        pitch_i:float = pitch_last_integral + (error_rate_pitch * pid_pitch_ki * cycle_time_seconds)
        pitch_i = max(min(pitch_i, i_limit), -i_limit) 
        pitch_d:float = pid_pitch_kd * (error_rate_pitch - pitch_last_error) / cycle_time_seconds
        pid_pitch = pitch_p + pitch_i + pitch_d
        print(f"pitch PID: {pid_pitch} = {pitch_p} + {pitch_i} + {pitch_d}")
        sleep(1)
        
        # yaw PID calc
        yaw_p:float = error_rate_yaw * pid_yaw_kp
        yaw_i:float = yaw_last_integral + (error_rate_yaw * pid_yaw_ki * cycle_time_seconds)
        yaw_i = max(min(yaw_i, i_limit), -i_limit)
        yaw_d:float = pid_yaw_kd * (error_rate_yaw - yaw_last_error) / cycle_time_seconds
        pid_yaw = yaw_p + yaw_i + yaw_d
        print(f"yaw PID: {pid_yaw} = {yaw_p} + {yaw_i} + {yaw_d}")
        sleep(1)
        
        # calculate throttle values
        t1:float = throttle + pid_pitch + pid_roll - pid_yaw
        t2:float = throttle + pid_pitch - pid_roll + pid_yaw
        t3:float = throttle - pid_pitch + pid_roll + pid_yaw
        t4:float = throttle - pid_pitch - pid_roll - pid_yaw
        print(f"t1: {t1}, t2: {t2}, t3: {t3}, t4: {t4}")
        sleep(1)
        
        # Adjust throttle according to input
        MOTOR_select()
        start_motor(1, normalize(t1, 0, 100, 1000, 2000))
        start_motor(2, normalize(t2, 0, 100, 1000, 2000))
        start_motor(3, normalize(t3, 0, 100, 1000, 2000))
        start_motor(4, normalize(t4, 0, 100, 1000, 2000))

        # Save state values for next loop
        roll_last_error = error_rate_roll
        pitch_last_error = error_rate_pitch
        yaw_last_error = error_rate_yaw
        roll_last_integral = roll_i
        pitch_last_integral = pitch_i
        yaw_last_integral = yaw_i

        
        elapsed_us: int = int(time() * 1000000) - loop_begin_us
        if elapsed_us < cycle_time_us:
            sleep((cycle_time_us - elapsed_us) / 1000000)

except Exception as e:
    print(f"ERROR: {e}")
    MOTOR_select()
    start_motor(1, 0)
    start_motor(2, 0)
    start_motor(3, 0)
    start_motor(4, 0)
    
