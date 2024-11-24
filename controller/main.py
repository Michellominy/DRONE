from pynq.overlays.base import BaseOverlay
from pynq import Overlay
from time import *
import controller.drone as drone

# https://timhanewich.medium.com/how-i-developed-the-scout-flight-controller-part-7-full-flight-controller-code-4269c83b3b48

# Max attitude rate of change rates (degrees per second)
max_rate_roll:float = 30.0
max_rate_pitch:float = 30.0 
max_rate_yaw:float = 50.0 


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

target_cycle_hz:float = 250.0
cycle_time_seconds:float = 1.0 / target_cycle_hz
cycle_time_us:int = int(round(cycle_time_seconds * 1000000, 0)) 
i_limit:float = 150.0

roll_last_integral:float = 0.0
roll_last_error:float = 0.0
pitch_last_integral:float = 0.0
pitch_last_error:float = 0.0
yaw_last_integral:float = 0.0
yaw_last_error:float = 0.0

throttle_idle:float = 0.14 # the minumum throttle needed to apply to the four motors for them all to spin up, but not provide lift (idling on the ground). the only way to find this value is through testing (with props off).
throttle_max:float = 0.22
throttle_range:float = throttle_max - throttle_idle

drone.init()

try:
    while True:
        input_throttle:float = 0.0  # between 0.0 and 1.0
        input_pitch:float = 0.0 # between -1.0 and 1.0
        input_roll:float = 0.0  # between -1.0 and 1.0
        input_yaw:float = 0.0   # between -1.0 and 1.0

        loop_begin_us:int = time.ticks_us()

        adj_throttle:float = throttle_idle + (throttle_range * input_throttle)

        gyro_x, gyro_y, gyro_z = drone.read_gyro()
        # calculate errors - diff between the actual rate of change in that axis (gyro_*) and the desired rate of change in that axis (input_* * max_rate_*)
        error_rate_roll:float = (input_roll * max_rate_roll) - gyro_x
        error_rate_pitch:float = (input_pitch * max_rate_pitch) - gyro_y
        error_rate_yaw:float = (input_yaw * max_rate_yaw) - gyro_z

        # roll PID calc
        roll_p:float = error_rate_roll * pid_roll_kp
        roll_i:float = roll_last_integral + (error_rate_roll * pid_roll_ki * cycle_time_seconds)
        roll_i = max(min(roll_i, i_limit), -i_limit) 
        roll_d:float = pid_roll_kd * (error_rate_roll - roll_last_error) / cycle_time_seconds
        pid_roll:float = roll_p + roll_i + roll_d

        # pitch PID calc
        pitch_p:float = error_rate_pitch * pid_pitch_kp
        pitch_i:float = pitch_last_integral + (error_rate_pitch * pid_pitch_ki * cycle_time_seconds)
        pitch_i = max(min(pitch_i, i_limit), -i_limit) 
        pitch_d:float = pid_pitch_kd * (error_rate_pitch - pitch_last_error) / cycle_time_seconds
        pid_pitch = pitch_p + pitch_i + pitch_d

        # yaw PID calc
        yaw_p:float = error_rate_yaw * pid_yaw_kp
        yaw_i:float = yaw_last_integral + (error_rate_yaw * pid_yaw_ki * cycle_time_seconds)
        yaw_i = max(min(yaw_i, i_limit), -i_limit)
        yaw_d:float = pid_yaw_kd * (error_rate_yaw - yaw_last_error) / cycle_time_seconds
        pid_yaw = yaw_p + yaw_i + yaw_d

        # calculate throttle values
        t1:float = adj_throttle + pid_pitch + pid_roll - pid_yaw
        t2:float = adj_throttle + pid_pitch - pid_roll + pid_yaw
        t3:float = adj_throttle - pid_pitch + pid_roll + pid_yaw
        t4:float = adj_throttle - pid_pitch - pid_roll - pid_yaw

        # Adjust throttle according to input
        motor_to_speed_dict = {
            1: t1,
            2: t2,
            3: t3,
            4: t4
        }
        drone.start_motors(motor_to_speed_dict)

        # Save state values for next loop
        roll_last_error = error_rate_roll
        pitch_last_error = error_rate_pitch
        yaw_last_error = error_rate_yaw
        roll_last_integral = roll_i
        pitch_last_integral = pitch_i
        yaw_last_integral = yaw_i

        
        elapsed_us:int = time.ticks_us() - loop_begin_us
        if elapsed_us < cycle_time_us:
            time.sleep_us(cycle_time_us - elapsed_us)

except Exception as e:
    motor_to_speed_dict = {
            1: 0,
            2: 0,
            3: 0,
            4: 0
        }
    drone.start_motors(motor_to_speed_dict)
    