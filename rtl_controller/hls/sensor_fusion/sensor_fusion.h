#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <ap_int.h>
#include <ap_fixed.h>
#include <hls_stream.h>
#include <hls_math.h>
#include <ap_axi_sdata.h>

typedef ap_fixed<32, 16> q16_t;

typedef ap_axiu<256, 0, 0, 0> axis_256_t;
typedef ap_axiu<128, 0, 0, 0> axis_128_t;

void sensor_fusion(
    hls::stream<axis_256_t>& imu_in,
    hls::stream<axis_128_t>& fusion_out
);

#endif
