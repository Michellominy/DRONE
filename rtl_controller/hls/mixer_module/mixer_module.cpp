#include <ap_int.h>
#include <ap_fixed.h>
#include <hls_stream.h>
#include <ap_axi_sdata.h>

typedef ap_fixed<32, 16> q16_t;
typedef ap_axiu<128,0,0,0>  axis_128_t;


void mixer_module(
    hls::stream<axis_128_t>& correction_in,
    hls::stream<axis_128_t>& motor_out,
    q16_t adj_throttle_m1,
    q16_t adj_throttle_m2,
    q16_t adj_throttle_m3,
    q16_t adj_throttle_m4,
    q16_t *motor_1,
    q16_t *motor_2,
    q16_t *motor_3,
    q16_t *motor_4
) {
    #pragma HLS TOP
    #pragma HLS INTERFACE axis port=correction_in
    #pragma HLS INTERFACE axis port=motor_out
    #pragma HLS INTERFACE ap_none port=adj_throttle_m1
    #pragma HLS INTERFACE ap_none port=adj_throttle_m2
    #pragma HLS INTERFACE ap_none port=adj_throttle_m3
    #pragma HLS INTERFACE ap_none port=adj_throttle_m4
    #pragma HLS INTERFACE ap_none port=motor_1
    #pragma HLS INTERFACE ap_none port=motor_2
    #pragma HLS INTERFACE ap_none port=motor_3
    #pragma HLS INTERFACE ap_none port=motor_4
    
    axis_128_t packet_in;
    correction_in.read(packet_in);

    q16_t out_roll, out_pitch, out_yaw;
    out_roll.range(31, 0)  = packet_in.data.range(31, 0);
    out_pitch.range(31, 0) = packet_in.data.range(63, 32);
    out_yaw.range(31, 0)   = packet_in.data.range(95, 64);

    *motor_1 = adj_throttle_m1 + out_pitch - out_roll - out_yaw;
    *motor_2 = adj_throttle_m2 - out_pitch - out_roll + out_yaw;
    *motor_3 = adj_throttle_m3 + out_pitch + out_roll + out_yaw;
    *motor_4 = adj_throttle_m4 - out_pitch + out_roll - out_yaw;

    axis_128_t packet_out;
    packet_out.data.range(31, 0)   = motor_1->range(31, 0);
    packet_out.data.range(63, 32)  = motor_2->range(31, 0);
    packet_out.data.range(95, 64)  = motor_3->range(31, 0);
    packet_out.data.range(127, 96) = motor_4->range(31, 0);
    
    packet_out.keep = 0xFFFF; 
    packet_out.last = packet_in.last;

    motor_out.write(packet_out);
}