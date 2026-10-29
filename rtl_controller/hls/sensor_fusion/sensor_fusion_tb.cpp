#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <iomanip>

#include "sensor_fusion.h"

const float TOLERANCE_DEG = 0.1f;

struct GoldenRow {
    int sample;
    int ax_raw;
    int ay_raw;
    int az_raw;
    int gx_raw;
    int gy_raw;
    int gz_raw;
    int mx_raw;
    int my_raw;
    int mz_raw;
    int temp_raw;
    int mag_z_offset_raw;
    int mag_y_offset_raw;
    int mag_x_offset_raw;
    int gyro_z_offset_raw;
    int gyro_y_offset_raw;
    int gyro_x_offset_raw;
    float expected_roll_deg;
    float expected_pitch_deg;
    float expected_yaw_deg;
};

bool parse_csv_line(const std::string& line, GoldenRow& row) {
    std::stringstream ss(line);
    std::string field;

    try {
        std::getline(ss, field, ','); row.sample = std::stoi(field);
        std::getline(ss, field, ','); row.ax_raw = std::stoi(field);
        std::getline(ss, field, ','); row.ay_raw = std::stoi(field);
        std::getline(ss, field, ','); row.az_raw = std::stoi(field);
        std::getline(ss, field, ','); row.gx_raw = std::stoi(field);
        std::getline(ss, field, ','); row.gy_raw = std::stoi(field);
        std::getline(ss, field, ','); row.gz_raw = std::stoi(field);
        std::getline(ss, field, ','); row.mx_raw = std::stoi(field);
        std::getline(ss, field, ','); row.my_raw = std::stoi(field);
        std::getline(ss, field, ','); row.mz_raw = std::stoi(field);
        std::getline(ss, field, ','); row.temp_raw = std::stoi(field);
        std::getline(ss, field, ','); row.mag_z_offset_raw = std::stoi(field);
        std::getline(ss, field, ','); row.mag_y_offset_raw = std::stoi(field);
        std::getline(ss, field, ','); row.mag_x_offset_raw = std::stoi(field);
        std::getline(ss, field, ','); row.gyro_z_offset_raw = std::stoi(field);
        std::getline(ss, field, ','); row.gyro_y_offset_raw = std::stoi(field);
        std::getline(ss, field, ','); row.gyro_x_offset_raw = std::stoi(field);
        std::getline(ss, field, ','); row.expected_roll_deg = std::stof(field);
        std::getline(ss, field, ','); row.expected_pitch_deg = std::stof(field);
        std::getline(ss, field, ','); row.expected_yaw_deg = std::stof(field);
    } catch (...) {
        return false;
    }
    return true;
}

axis_256_t make_packet(const GoldenRow& row) {
    axis_256_t packet;
    packet.data = 0;

    packet.data.range(15, 0)   = (ap_int<16>)row.ax_raw;
    packet.data.range(31, 16)  = (ap_int<16>)row.ay_raw;
    packet.data.range(47, 32)  = (ap_int<16>)row.az_raw;
    packet.data.range(63, 48)  = (ap_int<16>)row.gx_raw;
    packet.data.range(79, 64)  = (ap_int<16>)row.gy_raw;
    packet.data.range(95, 80)  = (ap_int<16>)row.gz_raw;
    packet.data.range(111, 96) = (ap_int<16>)row.mx_raw;
    packet.data.range(127, 112)= (ap_int<16>)row.my_raw;
    packet.data.range(143, 128)= (ap_int<16>)row.mz_raw;
    packet.data.range(159, 144)= (ap_int<16>)row.temp_raw;
    packet.data.range(175, 160)= (ap_int<16>)row.mag_z_offset_raw;
    packet.data.range(191, 176)= (ap_int<16>)row.mag_y_offset_raw;
    packet.data.range(207, 192)= (ap_int<16>)row.mag_x_offset_raw;
    packet.data.range(223, 208)= (ap_int<16>)row.gyro_z_offset_raw;
    packet.data.range(239, 224)= (ap_int<16>)row.gyro_y_offset_raw;
    packet.data.range(255, 240)= (ap_int<16>)row.gyro_x_offset_raw;

    packet.keep = -1;
    packet.strb = -1;
    packet.last = 1;

    return packet;
}

void decode_output(const axis_128_t& packet, float& roll_deg, float& pitch_deg, float& yaw_deg) {
    q16_t roll;
    q16_t pitch;
    q16_t yaw;

    roll.range(31, 0)  = packet.data.range(31, 0);
    pitch.range(31, 0) = packet.data.range(63, 32);
    yaw.range(31, 0)   = packet.data.range(95, 64);

    roll_deg  = roll.to_float();
    pitch_deg = pitch.to_float();
    yaw_deg   = yaw.to_float();
}

int main() {
    hls::stream<axis_256_t> imu_in;
    hls::stream<axis_128_t> fusion_out;

    std::ifstream file("test_data.csv");

    if (!file.is_open()) {
        std::cerr << "ERROR: Could not open test_data.csv" << std::endl;
        return 1;
    }

    std::cout << "Opened test_data.csv" << std::endl;

    std::string line;
    
    // Skip the CSV header row
    if (!std::getline(file, line)) {
        std::cerr << "ERROR: CSV is empty" << std::endl;
        return 1;
    }

    int total_samples = 0;
    int failed_samples = 0;
    float max_roll_error = 0.0f;
    float max_pitch_error = 0.0f;
    float max_yaw_error = 0.0f;

    while (std::getline(file, line)) {
        if (line.empty()) continue;

        GoldenRow row;
        if (!parse_csv_line(line, row)) {
            std::cerr << "ERROR: Could not parse CSV line:\n" << line << std::endl;
            return 1;
        }

        axis_256_t packet = make_packet(row);
        imu_in.write(packet);

        sensor_fusion(imu_in, fusion_out);

        if (fusion_out.empty()) {
            std::cerr << "ERROR: No output for sample " << row.sample << std::endl;
            return 1;
        }

        axis_128_t output = fusion_out.read();

        float actual_roll, actual_pitch, actual_yaw;
        decode_output(output, actual_roll, actual_pitch, actual_yaw);

        float roll_error  = std::fabs(actual_roll - row.expected_roll_deg);
        float pitch_error = std::fabs(actual_pitch - row.expected_pitch_deg);
        float yaw_error   = std::fabs(actual_yaw - row.expected_yaw_deg);

        if (roll_error > max_roll_error) max_roll_error = roll_error;
        if (pitch_error > max_pitch_error) max_pitch_error = pitch_error;
        if (yaw_error > max_yaw_error) max_yaw_error = yaw_error;

        bool pass = (roll_error <= TOLERANCE_DEG && 
                     pitch_error <= TOLERANCE_DEG && 
                     yaw_error <= TOLERANCE_DEG);

        total_samples++;

        if (!pass) {
            failed_samples++;
            if (failed_samples <= 20) {
                std::cout << std::fixed << std::setprecision(6);
                std::cout << "\nFAIL sample " << row.sample << std::endl;
                std::cout << "  Roll: expected=" << row.expected_roll_deg 
                          << " actual=" << actual_roll << " error=" << roll_error << std::endl;
                std::cout << "  Pitch: expected=" << row.expected_pitch_deg 
                          << " actual=" << actual_pitch << " error=" << pitch_error << std::endl;
                std::cout << "  Yaw: expected=" << row.expected_yaw_deg 
                          << " actual=" << actual_yaw << " error=" << yaw_error << std::endl;
            }
        }

        if ((total_samples % 100) == 0) {
            std::cout << "Processed " << total_samples << " samples" << std::endl;
        }
    }

    std::cout << "\n========================================" << std::endl;
    std::cout << "Golden reference test" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Total samples : " << total_samples << std::endl;
    std::cout << "Failed samples: " << failed_samples << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Max roll error : " << max_roll_error << " deg" << std::endl;
    std::cout << "Max pitch error: " << max_pitch_error << " deg" << std::endl;
    std::cout << "Max yaw error  : " << max_yaw_error << " deg" << std::endl;
    std::cout << "Tolerance      : " << TOLERANCE_DEG << " deg" << std::endl;

    if (failed_samples == 0) {
        std::cout << "\n*** TEST PASSED ***" << std::endl;
        return 0;
    } else {
        std::cout << "\n*** TEST FAILED ***" << std::endl;
        return 1;
    }
}