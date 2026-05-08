#include <stdint.h>

typedef int32_t q16;
#define Q16_ONE (1 << 16)

static q16 q16_mul_hls(q16 a, q16 b) {
    return (q16)(((int64_t)a * (int64_t)b) >> 16);
}

struct pid_state {
    q16 integ;
    q16 last_err;
};

q16 run_pid(q16 setpoint, q16 measure, q16 kp, q16 ki, q16 kd, q16 dt, pid_state &state) {
    q16 err = setpoint - measure;
    q16 P = q16_mul_hls(kp, err);
    
    q16 delta_i = q16_mul_hls(q16_mul_hls(ki, err), dt);
    state.integ += delta_i;
    
    q16 diff = err - state.last_err;
    q16 D = (dt != 0) ? (q16)(((int64_t)q16_mul_hls(kd, diff) << 16) / dt) : 0;
    
    state.last_err = err;
    q16 out = P + state.integ + D;

    if (out > Q16_ONE) out = Q16_ONE;
    if (out < -Q16_ONE) out = -Q16_ONE;
    
    return out;
}

void hardware_pid_engine(
    // Inputs from MicroBlaze
    q16 set_roll, q16 set_pitch, q16 set_yaw,
    q16 gyro_x, q16 gyro_y, q16 gyro_z,
    q16 kp, q16 ki, q16 kd, q16 dt,
    q16 throttle,
    // Outputs to MicroBlaze
    q16 *out_roll, q16 *out_pitch, q16 *out_yaw
) {
    #pragma HLS INTERFACE s_axilite port=return bundle=CTRL_BUS
    #pragma HLS INTERFACE s_axilite port=set_roll bundle=CTRL_BUS
    #pragma HLS INTERFACE s_axilite port=set_pitch bundle=CTRL_BUS
    #pragma HLS INTERFACE s_axilite port=set_yaw bundle=CTRL_BUS
    #pragma HLS INTERFACE s_axilite port=gyro_x bundle=CTRL_BUS
    #pragma HLS INTERFACE s_axilite port=gyro_y bundle=CTRL_BUS
    #pragma HLS INTERFACE s_axilite port=gyro_z bundle=CTRL_BUS
    #pragma HLS INTERFACE s_axilite port=kp bundle=CTRL_BUS
    #pragma HLS INTERFACE s_axilite port=ki bundle=CTRL_BUS
    #pragma HLS INTERFACE s_axilite port=kd bundle=CTRL_BUS
    #pragma HLS INTERFACE s_axilite port=dt bundle=CTRL_BUS
    #pragma HLS INTERFACE s_axilite port=throttle bundle=CTRL_BUS
    #pragma HLS INTERFACE s_axilite port=out_roll bundle=CTRL_BUS
    #pragma HLS INTERFACE s_axilite port=out_pitch bundle=CTRL_BUS
    #pragma HLS INTERFACE s_axilite port=out_yaw bundle=CTRL_BUS

    static pid_state roll_st = {0, 0};
    static pid_state pitch_st = {0, 0};
    static pid_state yaw_st = {0, 0};

    *out_roll  = run_pid(set_roll,  gyro_y, kp, ki, kd, dt, roll_st);
    *out_pitch = run_pid(set_pitch, gyro_x, kp, ki, kd, dt, pitch_st);
    *out_yaw   = run_pid(set_yaw,   gyro_z, kp, ki, kd, dt, yaw_st);
}