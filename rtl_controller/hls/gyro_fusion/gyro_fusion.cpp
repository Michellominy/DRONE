#include <ap_int.h>
#include <ap_fixed.h>
#include <hls_stream.h>
#include <hls_math.h>

typedef ap_fixed<32, 16> q16_t;

struct axis_128_t {
    ap_uint<128> data;
    ap_uint<16> keep;
    ap_uint<1> last;
};

struct axis_96_t {
    ap_uint<96> data;
    ap_uint<12> keep;
    ap_uint<1> last;
};

const float GYRO_SCALE = 1.0f / 131.0f;
const float ACCEL_SCALE = 1.0f / 16384.0f;
const float DT = 0.008f;       // 125 Hz 
const float ALPHA = 0.98f;     // Complementary filter tuning constant
const float RAD_TO_DEG = 57.2957795f;

void sensor_fusion(
    hls::stream<axis_128_t>& mpu_in,
    hls::stream<axis_96_t>& fusion_out,
    float gyro_x_offset,  
    float gyro_y_offset,
    float gyro_z_offset
) {
    #pragma HLS INTERFACE axis port=mpu_in
    #pragma HLS INTERFACE axis port=fusion_out
    #pragma HLS INTERFACE s_axilite port=gyro_x_offset bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=gyro_y_offset bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=gyro_z_offset bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=return bundle=CTRL

    static float roll_est = 0.0f;
    static float pitch_est = 0.0f;

    axis_128_t packet_in;
    
    mpu_in.read(packet_in);
    
    ap_int<16> ax_raw = packet_in.data.range(15, 0);
    ap_int<16> ay_raw = packet_in.data.range(31, 16);
    ap_int<16> az_raw = packet_in.data.range(47, 32);
    
    // temperature is at range(63, 48) - ignored for flight dynamics
    
    ap_int<16> gx_raw = packet_in.data.range(79, 64);
    ap_int<16> gy_raw = packet_in.data.range(95, 80);
    ap_int<16> gz_raw = packet_in.data.range(111, 96);

    float ax = (float)ax_raw * ACCEL_SCALE;
    float ay = (float)ay_raw * ACCEL_SCALE;
    float az = (float)az_raw * ACCEL_SCALE;

    float gx = ((float)gx_raw * GYRO_SCALE) - gyro_x_offset;
    float gy = ((float)gy_raw * GYRO_SCALE) - gyro_y_offset;
    float gz = ((float)gz_raw * GYRO_SCALE) - gyro_z_offset;

    // 3. Calculate absolute angles from Accelerometer using hardware CORDIC trig
    float accel_pitch = hls::atan2(ay, az) * RAD_TO_DEG;
    float accel_roll = hls::atan2(-ax, hls::sqrt(ay*ay + az*az)) * RAD_TO_DEG;

    // 4. Complementary Filter
    roll_est  = ALPHA * (roll_est + gx * DT) + (1.0f - ALPHA) * accel_roll;
    pitch_est = ALPHA * (pitch_est + gy * DT) + (1.0f - ALPHA) * accel_pitch;
    
    // We don't filter Yaw because the accelerometer can't measure rotation around gravity.
    // We just pass the Yaw rate directly to the PID.
    float yaw_rate = gz;

    // 5. Convert to Q16 Fixed-Point for the downstream PID module
    q16_t roll_out = roll_est;
    q16_t pitch_out = pitch_est;
    q16_t yaw_rate_out = yaw_rate;

    // 6. Pack and transmit the 96-bit vector
    axis_96_t packet_out;
    packet_out.data.range(31, 0)  = roll_out.range(31, 0);
    packet_out.data.range(63, 32) = pitch_out.range(31, 0);
    packet_out.data.range(95, 64) = yaw_rate_out.range(31, 0);
    packet_out.keep = 0xFFF; // 12 bytes valid
    packet_out.last = packet_in.last; // Pass the tlast signal forward

    fusion_out.write(packet_out);
}