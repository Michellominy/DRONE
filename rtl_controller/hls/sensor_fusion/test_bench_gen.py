import csv
import math

# Constants mapping to sensor_fusion.cpp
GYRO_SCALE = 1.0 / 131.0
ACCEL_SCALE = 1.0 / 16384.0
MAG_SCALE = 0.15
DT = 0.01
ALPHA = 0.98
ONE_MINUS_ALPHA = 1.0 - ALPHA
RAD_TO_DEG = 57.2957795
DEG_TO_RAD = 1.0 / 57.2957795
PI_F = 3.14159265

def generate_test_data(num_samples=1000):
    rows = []
    
    # State variables for the complementary filter
    roll_est_rad = 0.0
    pitch_est_rad = 0.0
    yaw_est_rad = 0.0
    
    rows.append([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])

    for i in range(1, num_samples):
        t = i * DT

        # Simulate dynamic movement
        ax_raw = int(16384 * math.sin(t * 0.5) * 0.2)
        ay_raw = int(16384 * math.sin(t * 0.3) * 0.2)
        # Calculate az to keep the total acceleration vector near 1G (16384)
        az_raw = int(math.sqrt(max(0, 16384**2 - ax_raw**2 - ay_raw**2)))

        gx_raw = int(math.cos(t * 0.3) * 0.2 * 131.0) 
        gy_raw = int(math.cos(t * 0.5) * 0.2 * 131.0)
        gz_raw = int(math.sin(t * 0.1) * 30.0 * 131.0)

        # Simulate rotating magnetic field
        mx_raw = int(300 * math.cos(t * 0.1))
        my_raw = int(300 * math.sin(t * 0.1))
        mz_raw = 150

        temp_raw = 2500
        mag_z_offset_raw = 0
        mag_y_offset_raw = 0
        mag_x_offset_raw = 0
        gyro_z_offset_raw = 0
        gyro_y_offset_raw = 0
        gyro_x_offset_raw = 0

        # --- Golden Reference Math Execution ---
        ax = ax_raw * ACCEL_SCALE
        ay = ay_raw * ACCEL_SCALE
        az = az_raw * ACCEL_SCALE

        gx = (gx_raw - gyro_x_offset_raw) * GYRO_SCALE * DEG_TO_RAD
        gy = (gy_raw - gyro_y_offset_raw) * GYRO_SCALE * DEG_TO_RAD
        gz = (gz_raw - gyro_z_offset_raw) * GYRO_SCALE * DEG_TO_RAD

        mx = (mx_raw - mag_x_offset_raw) * MAG_SCALE
        my = (my_raw - mag_y_offset_raw) * MAG_SCALE
        mz = (mz_raw - mag_z_offset_raw) * MAG_SCALE

        norm_sq = ax*ax + ay*ay + az*az
        
        # NaN protection matching hardware implementation
        if norm_sq < 0.0001:
            ax, ay, az = 0.0, 0.0, 1.0
        else:
            inv_norm = 1.0 / math.sqrt(norm_sq)
            ax *= inv_norm
            ay *= inv_norm
            az *= inv_norm

        if ay > 1.0: ay = 1.0
        if ay < -1.0: ay = -1.0

        accel_pitch = math.asin(ay)
        accel_roll = math.atan2(-ax, az)

        Bx, By, Bz = mx, -my, -mz

        M_flat_x = Bx * az - Bz * ax
        M_flat_y = -Bx * ax * ay + By * (1.0 - ay * ay) - Bz * ay * az
        
        mag_yaw = 0.0
        if abs(M_flat_x) > 0.0001 or abs(M_flat_y) > 0.0001:
            mag_yaw = math.atan2(M_flat_x, M_flat_y)

        sin_roll = math.sin(roll_est_rad)
        cos_roll = math.cos(roll_est_rad)
        cos_pitch = math.cos(pitch_est_rad)
        tan_pitch = math.tan(pitch_est_rad)

        pitch_rate = gx + sin_roll * tan_pitch * gy - cos_roll * tan_pitch * gz
        roll_rate = cos_roll * gy + sin_roll * gz
        
        yaw_rate = 0.0
        if abs(cos_pitch) > 0.01:
            yaw_rate = (-sin_roll * gy + cos_roll * gz) / cos_pitch

        roll_est_rad = ALPHA * (roll_est_rad + roll_rate * DT) + ONE_MINUS_ALPHA * accel_roll
        pitch_est_rad = ALPHA * (pitch_est_rad + pitch_rate * DT) + ONE_MINUS_ALPHA * accel_pitch

        yaw_gyro = yaw_est_rad + yaw_rate * DT
        yaw_err = mag_yaw - yaw_gyro
        
        if yaw_err > PI_F: yaw_err -= 2.0 * PI_F
        elif yaw_err < -PI_F: yaw_err += 2.0 * PI_F

        yaw_est_rad = yaw_gyro + ONE_MINUS_ALPHA * yaw_err

        if yaw_est_rad > PI_F: yaw_est_rad -= 2.0 * PI_F
        elif yaw_est_rad < -PI_F: yaw_est_rad += 2.0 * PI_F

        expected_roll_deg = roll_est_rad * RAD_TO_DEG
        expected_pitch_deg = pitch_est_rad * RAD_TO_DEG
        expected_yaw_deg = yaw_est_rad * RAD_TO_DEG

        rows.append([
            i, ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, 
            mx_raw, my_raw, mz_raw, temp_raw, 
            mag_z_offset_raw, mag_y_offset_raw, mag_x_offset_raw, 
            gyro_z_offset_raw, gyro_y_offset_raw, gyro_x_offset_raw, 
            expected_roll_deg, expected_pitch_deg, expected_yaw_deg
        ])

    with open('test_data.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            "sample","ax_raw","ay_raw","az_raw","gx_raw","gy_raw","gz_raw",
            "mx_raw","my_raw","mz_raw","temp_raw",
            "mag_z_offset_raw","mag_y_offset_raw","mag_x_offset_raw",
            "gyro_z_offset_raw","gyro_y_offset_raw","gyro_x_offset_raw",
            "expected_roll_deg","expected_pitch_deg","expected_yaw_deg"
        ])
        writer.writerows(rows)
    print(f"Generated {num_samples} samples in test_data.csv")

if __name__ == "__main__":
    generate_test_data()