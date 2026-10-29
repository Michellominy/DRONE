#include <ap_int.h>
#include <ap_fixed.h>
#include <hls_stream.h>
#include <stdint.h>
#include <ap_axi_sdata.h>

typedef ap_fixed<32, 16> q16_t;
typedef ap_axiu<128,0,0,0> axis_128_t;

struct PID_State {
    q16_t integral;
    q16_t prev_error;
};

inline q16_t wrap_180(q16_t angle) {
    #pragma HLS INLINE
    if (angle > 180) {
        return angle - 360;
    } else if (angle < -180) {
        return angle + 360;
    }
    return angle;
}

inline q16_t compute_pid(q16_t current, q16_t setpoint, 
                         q16_t kp, q16_t ki, q16_t kd, 
                         PID_State &state, q16_t i_min, q16_t i_max,
                         q16_t out_min, q16_t out_max, 
                         bool is_yaw = false) {
    #pragma HLS INLINE

    q16_t error = setpoint - current;
    
    if (is_yaw) {
        error = wrap_180(error);
    }

    // Fixed dt of 0.01 seconds (100 Hz)
    q16_t dt = 0.01;
    
    // Calculate raw tentative integral
    q16_t tentative_integral = state.integral + (error * dt);
    
    // Anti-Windup Strategy 1: Integrator Clamping
    // Prevent integral state from blowing up past safe bounds
    if (tentative_integral > i_max) {
        state.integral = i_max;
    } else if (tentative_integral < i_min) {
        state.integral = i_min;
    } else {
        state.integral = tentative_integral;
    }
    
    q16_t dt_inv = 100.0;
    q16_t derivative = (error - state.prev_error) * dt_inv;
    
    state.prev_error = error;
    
    // Compute PID terms
    q16_t p_term = kp * error;
    q16_t i_term = ki * state.integral;
    q16_t d_term = kd * derivative;
    
    q16_t raw_output = p_term + i_term + d_term;
    
    // Anti-Windup Strategy 2: Output Clamping (Saturation Limits)
    q16_t final_output = raw_output;
    if (final_output > out_max) {
        final_output = out_max;
    } else if (final_output < out_min) {
        final_output = out_min;
    }
    
    return final_output;
}

void pid_controller(
    hls::stream<axis_128_t> &sensor_in,
    hls::stream<axis_128_t> &correction_out,
    
    q16_t setpoint_roll,
    q16_t setpoint_pitch,
    q16_t setpoint_yaw, // delta degrees per cycle
    
    q16_t kp_roll, q16_t ki_roll, q16_t kd_roll,
    q16_t kp_pitch, q16_t ki_pitch, q16_t kd_pitch,
    q16_t kp_yaw, q16_t ki_yaw, q16_t kd_yaw,
    
    bool reset_states
) {
    #pragma HLS TOP

    #pragma HLS INTERFACE axis port=sensor_in
    #pragma HLS INTERFACE axis port=correction_out
    
    #pragma HLS INTERFACE ap_none port=setpoint_roll
    #pragma HLS INTERFACE ap_none port=setpoint_pitch
    #pragma HLS INTERFACE ap_none port=setpoint_yaw
    
    #pragma HLS INTERFACE ap_none port=kp_roll
    #pragma HLS INTERFACE ap_none port=ki_roll
    #pragma HLS INTERFACE ap_none port=kd_roll

    #pragma HLS INTERFACE ap_none port=kp_pitch
    #pragma HLS INTERFACE ap_none port=ki_pitch
    #pragma HLS INTERFACE ap_none port=kd_pitch

    #pragma HLS INTERFACE ap_none port=kp_yaw
    #pragma HLS INTERFACE ap_none port=ki_yaw
    #pragma HLS INTERFACE ap_none port=kd_yaw

    #pragma HLS INTERFACE ap_none port=reset_states
        
    static PID_State state_roll  = {0, 0};
    static PID_State state_pitch = {0, 0};
    static PID_State state_yaw   = {0, 0};
    
    static q16_t internal_yaw_target = 0;
    static bool yaw_initialized = false;

    if (reset_states) {
        state_roll  = {0, 0};
        state_pitch = {0, 0};
        state_yaw   = {0, 0};
        yaw_initialized = false;
    }

    if (!sensor_in.empty()) {
        axis_128_t packet_in = sensor_in.read();
        
        q16_t current_roll, current_pitch, current_yaw;
        current_roll.range(31, 0)  = packet_in.data.range(31, 0);
        current_pitch.range(31, 0) = packet_in.data.range(63, 32);
        current_yaw.range(31, 0)   = packet_in.data.range(95, 64);
        
        if (!yaw_initialized) {
            internal_yaw_target = current_yaw;
            yaw_initialized = true;
        }

        internal_yaw_target += setpoint_yaw;
        internal_yaw_target = wrap_180(internal_yaw_target);
        
        q16_t correction_roll = compute_pid(
            current_roll, setpoint_roll, 
            kp_roll, ki_roll, kd_roll, 
            state_roll, -10.0, 10.0, -0.3, 0.3, false
        );
        
        q16_t correction_pitch = compute_pid(
            current_pitch, setpoint_pitch, 
            kp_pitch, ki_pitch, kd_pitch, 
            state_pitch,  -10.0, 10.0, -0.3, 0.3, false
        );
        
        q16_t correction_yaw = compute_pid(
            current_yaw, internal_yaw_target, 
            kp_yaw, ki_yaw, kd_yaw, 
            state_yaw, -5.0, 5.0, -0.2, 0.2, true
        );
        
        axis_128_t packet_out;
        packet_out.data.range(31, 0)  = correction_roll.range(31, 0);
        packet_out.data.range(63, 32) = correction_pitch.range(31, 0);
        packet_out.data.range(95, 64) = correction_yaw.range(31, 0);
        packet_out.data.range(127, 96) = 0;
        packet_out.keep = 0x0FFF;            
        packet_out.last = packet_in.last;   
        
        correction_out.write(packet_out);
    }
}