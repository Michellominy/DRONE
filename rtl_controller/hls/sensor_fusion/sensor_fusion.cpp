#include "sensor_fusion.h"

const float GYRO_SCALE = 1.0f / 131.0f;
const float ACCEL_SCALE = 1.0f / 16384.0f;
const float MAG_SCALE = 0.15f;
const float DT = 0.01f;  // 100 Hz
const float ALPHA = 0.98f; 
const float ONE_MINUS_ALPHA = 1.0f - ALPHA;
const float ALPHA_YAW = 0.995f; 
const float ONE_MINUS_ALPHA_YAW = 1.0f - ALPHA_YAW;
const float RAD_TO_DEG = 57.2957795f;
const float DEG_TO_RAD = 1.0f / 57.2957795f;
const float PI_F = 3.14159265f;
const float MAG_OFFSET_X = -63.00;
const float MAG_OFFSET_Y = -78.82;
const float MAG_OFFSET_Z = -29.47;

void sensor_fusion(
    hls::stream<axis_256_t>& imu_in,
    hls::stream<axis_128_t>& fusion_out,
    q16_t *roll,
    q16_t *pitch,
    q16_t *yaw
) {
    #pragma HLS TOP
    #pragma HLS INTERFACE axis port=imu_in
    #pragma HLS INTERFACE axis port=fusion_out
    #pragma HLS INTERFACE ap_none port=roll
    #pragma HLS INTERFACE ap_none port=pitch
    #pragma HLS INTERFACE ap_none port=yaw
    
    static float roll_est_rad = 0.0f;
    static float pitch_est_rad = 0.0f;
    static float yaw_est_rad = 0.0f;
    static bool first_time = true; // Boolean to estimate euler angles purely from Accel/Mag readings for init

    axis_256_t packet_in;
    imu_in.read(packet_in);
    
    ap_int<16> ax_raw = packet_in.data.range(15, 0);
    ap_int<16> ay_raw = packet_in.data.range(31, 16);
    ap_int<16> az_raw = packet_in.data.range(47, 32);

    ap_int<16> gx_raw = packet_in.data.range(63, 48);
    ap_int<16> gy_raw = packet_in.data.range(79, 64);
    ap_int<16> gz_raw = packet_in.data.range(95, 80);

    ap_int<16> mx_raw = packet_in.data.range(111, 96);
    ap_int<16> my_raw = packet_in.data.range(127, 112);
    ap_int<16> mz_raw = packet_in.data.range(143, 128);

    ap_int<16> temp_raw = packet_in.data.range(159, 144); 

    ap_int<16> gyro_z_offset_raw = packet_in.data.range(175, 160);
    ap_int<16> gyro_y_offset_raw = packet_in.data.range(191, 176);
    ap_int<16> gyro_x_offset_raw = packet_in.data.range(207, 192);

    float ax = (float)ax_raw * ACCEL_SCALE;
    float ay = (float)ay_raw * ACCEL_SCALE;
    float az = (float)az_raw * ACCEL_SCALE;

    float gx = (float)(gx_raw - gyro_x_offset_raw) * GYRO_SCALE * DEG_TO_RAD;
    float gy = (float)(gy_raw - gyro_y_offset_raw) * GYRO_SCALE * DEG_TO_RAD;
    float gz = (float)(gz_raw - gyro_z_offset_raw) * GYRO_SCALE * DEG_TO_RAD;

    float mx = ((float)(mx_raw) * MAG_SCALE) - MAG_OFFSET_X;
    float my = ((float)(my_raw) * MAG_SCALE) - MAG_OFFSET_Y;
    float mz = ((float)(mz_raw) * MAG_SCALE) - MAG_OFFSET_Z;

    float norm_sq = ax*ax + ay*ay + az*az;
    float inv_norm = (norm_sq > 0.0001f) ? hls::rsqrt(norm_sq) : 1.0f;

    ax *= inv_norm;
    ay *= inv_norm;
    az *= inv_norm;

    if (ay > 1.0f) ay = 1.0f;
    if (ay < -1.0f) ay = -1.0f;
    
    float accel_pitch = hls::asin(ay);
    float accel_roll = hls::atan2(-ax, az);
    
    float Bx = mx;
    float By = -my;
    float Bz = -mz;

    float M_flat_x = Bx * az - Bz * ax;
    float M_flat_y = -Bx * ax * ay + By * (1.0f - ay * ay) - Bz * ay * az;
    float mag_yaw = hls::atan2(M_flat_x, M_flat_y);

    float pitch_rate, roll_rate, yaw_rate;

    if (first_time) {
        roll_est_rad = accel_roll;
        pitch_est_rad = accel_pitch;
        yaw_est_rad = mag_yaw;
        
        pitch_rate = 0.0f;
        roll_rate = 0.0f;
        yaw_rate = 0.0f;
        
        first_time = false;
    } else {
        float sin_roll = hls::sin(roll_est_rad);
        float cos_roll = hls::cos(roll_est_rad);
        float cos_pitch = hls::cos(pitch_est_rad);
        float tan_pitch = hls::tan(pitch_est_rad);
        float factor = gz * cos_roll - gx * sin_roll;

        pitch_rate = gx * cos_roll + gz * sin_roll;
        roll_rate = gy + tan_pitch * (factor);
        
        yaw_rate = 0.0f;
        if (hls::abs(cos_pitch) > 0.01f) {
            yaw_rate = (factor) / cos_pitch;
        }

        roll_est_rad  = ALPHA * (roll_est_rad + roll_rate * DT) + ONE_MINUS_ALPHA * accel_roll;
        pitch_est_rad = ALPHA * (pitch_est_rad + pitch_rate * DT) + ONE_MINUS_ALPHA * accel_pitch;

        float yaw_gyro = yaw_est_rad + yaw_rate * DT;
        float yaw_err = mag_yaw - yaw_gyro;
        
        if (yaw_err > PI_F) yaw_err -= 2.0f * PI_F;
        else if (yaw_err < -PI_F) yaw_err += 2.0f * PI_F;

        yaw_est_rad = yaw_gyro + ONE_MINUS_ALPHA_YAW * yaw_err;

        if (yaw_est_rad > PI_F) yaw_est_rad -= 2.0f * PI_F;
        else if (yaw_est_rad < -PI_F) yaw_est_rad += 2.0f * PI_F;
    }

    *roll = roll_est_rad * RAD_TO_DEG;
    *pitch = pitch_est_rad * RAD_TO_DEG;
    *yaw = yaw_est_rad * RAD_TO_DEG;

    axis_128_t packet_out;

    packet_out.data.range(31, 0)   = (ap_uint<32>)roll->range(31, 0);
    packet_out.data.range(63, 32)  = (ap_uint<32>)pitch->range(31, 0);
    packet_out.data.range(95, 64)  = (ap_uint<32>)yaw->range(31, 0);
    packet_out.data.range(127, 96) = 0;
    packet_out.keep = 0xFFFF;
    packet_out.strb = 0xFFFF;
    packet_out.last = packet_in.last;

    fusion_out.write(packet_out);
}