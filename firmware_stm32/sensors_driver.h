#ifndef SENSORS_DRIVER_H
#define SENSORS_DRIVER_H

#include <Wire.h>
#include <VL53L0X.h>                 // Thư viện Pololu VL53L0X
#include <SparkFun_VL53L5CX_Library.h> // Thư viện SparkFun VL53L5CX
#include "robot_config.h"

// Khai báo 3 Bus I2C phần cứng của Arduino UNO Q (STM32U585)
// Wire  = I2C2 (D20 SDA, D21 SCL) -> 2x VL53L5CX
// Wire1 = I2C3 (A4 SDA, A5 SCL)   -> VL53L0X_A
// Wire2 = I2C4 (Cổng QWIIC)       -> VL53L0X_B
extern TwoWire Wire1;
extern TwoWire Wire2;

// Biến toàn cục phục vụ Ngắt Ngoài (ISR) đo siêu âm không chặn (Non-blocking)
volatile uint32_t us_a_start_us = 0;
volatile uint32_t us_a_pulse_us = 0;
volatile uint32_t us_b_start_us = 0;
volatile uint32_t us_b_pulse_us = 0;

void isrEchoA() {
    if (digitalRead(PIN_US_A_ECHO) == HIGH) {
        us_a_start_us = micros();
    } else {
        us_a_pulse_us = micros() - us_a_start_us;
    }
}

void isrEchoB() {
    if (digitalRead(PIN_US_B_ECHO) == HIGH) {
        us_b_start_us = micros();
    } else {
        us_b_pulse_us = micros() - us_b_start_us;
    }
}

class SensorsDriver {
private:
    VL53L0X lox_a;
    VL53L0X lox_b;
    SparkFun_VL53L5CX tof_l5_a;
    SparkFun_VL53L5CX tof_l5_b;
    VL53L5CX_ResultsData l5_data;

    // Mảng trọng số 8 mắt IR (Từ trái sang phải theo hướng nhìn tới của đầu xe)
    const float ir_weights[8] = {-1.0f, -0.71f, -0.43f, -0.14f, 0.14f, 0.43f, 0.71f, 1.0f};
    uint16_t ir_threshold = 500; // Ngưỡng phân biệt vạch line đen/trắng (0..1023) - Hiệu chỉnh thực tế

    uint32_t last_us_trigger_ms = 0;
    bool trigger_turn_a = true;

public:
    bool initAll() {
        // 1. Cấu hình chân MUX 16 kênh cho Line IR
        pinMode(PIN_IR_MUX_S0, OUTPUT);
        pinMode(PIN_IR_MUX_S1, OUTPUT);
        pinMode(PIN_IR_MUX_S2, OUTPUT);
        pinMode(PIN_IR_MUX_S3, OUTPUT);
        pinMode(PIN_IR_MUX_SIG, INPUT);

        // 2. Cấu hình Siêu âm 2 chiều & Ngắt ngoài
        pinMode(PIN_US_A_TRIG, OUTPUT);
        pinMode(PIN_US_A_ECHO, INPUT);
        pinMode(PIN_US_B_TRIG, OUTPUT);
        pinMode(PIN_US_B_ECHO, INPUT);
        attachInterrupt(digitalPinToInterrupt(PIN_US_A_ECHO), isrEchoA, CHANGE);
        attachInterrupt(digitalPinToInterrupt(PIN_US_B_ECHO), isrEchoB, CHANGE);

        // 3. Khởi tạo 3 Bus I2C tốc độ cao (400kHz)
        Wire.begin();
        Wire.setClock(400000);
        Wire1.begin();
        Wire1.setClock(400000);
        Wire2.begin();
        Wire2.setClock(400000);

        // 4. Khởi tạo 2x ToF đơn điểm VL53L0X trên 2 bus độc lập
        lox_a.setBus(&Wire1);
        lox_a.setTimeout(50);
        if (!lox_a.init()) return false;
        lox_a.startContinuous(20); // Đo liên tục mỗi 20ms

        lox_b.setBus(&Wire2);
        lox_b.setTimeout(50);
        if (!lox_b.init()) return false;
        lox_b.startContinuous(20);

        // 5. Khởi tạo 2x ToF đa điểm VL53L5CX trên bus Wire (D20/D21) dùng chân LPn đổi địa chỉ
        pinMode(PIN_TOF_L5_A_LPN, OUTPUT);
        pinMode(PIN_TOF_L5_B_LPN, OUTPUT);
        
        // Tắt cả 2 cảm biến trước
        digitalWrite(PIN_TOF_L5_A_LPN, LOW);
        digitalWrite(PIN_TOF_L5_B_LPN, LOW);
        delay(20);

        // Bật mắt A và đổi địa chỉ sang 0x30
        digitalWrite(PIN_TOF_L5_A_LPN, HIGH);
        delay(20);
        if (!tof_l5_a.begin(0x29, Wire)) return false;
        tof_l5_a.setAddress(TOF_L5_A_I2C_ADDR);
        tof_l5_a.setResolution(8 * 8); // Ma trận 64 vùng
        tof_l5_a.setRangingFrequency(15); // 15Hz (~66ms/frame)
        tof_l5_a.startRanging();

        // Bật mắt B và giữ/đổi địa chỉ sang 0x32
        digitalWrite(PIN_TOF_L5_B_LPN, HIGH);
        delay(20);
        if (!tof_l5_b.begin(0x29, Wire)) return false;
        tof_l5_b.setAddress(TOF_L5_B_I2C_ADDR);
        tof_l5_b.setResolution(8 * 8);
        tof_l5_b.setRangingFrequency(15);
        tof_l5_b.startRanging();

        return true;
    }

    // Đọc 1 kênh từ MUX CD74HC4067 (channel 0..15)
    uint16_t readMuxChannel(uint8_t channel) {
        digitalWrite(PIN_IR_MUX_S0, (channel & 0x01));
        digitalWrite(PIN_IR_MUX_S1, (channel >> 1) & 0x01);
        digitalWrite(PIN_IR_MUX_S2, (channel >> 2) & 0x01);
        digitalWrite(PIN_IR_MUX_S3, (channel >> 3) & 0x01);
        delayMicroseconds(2); // Chờ ổn định tín hiệu chuyển mạch MUX
        return analogRead(PIN_IR_MUX_SIG);
    }

    // Tính độ lệch tâm line (-1.0 đến +1.0) theo hướng chạy hiện tại (+1: Đầu A, -1: Đầu B)
    void updateLineSensor(RobotState &state) {
        uint8_t base_channel = (state.direction == 1) ? 0 : 8; // Kênh 0..7 cho Đầu A, 8..15 cho Đầu B
        float weighted_sum = 0.0f;
        float total_active = 0.0f;
        uint8_t active_count = 0;

        for (uint8_t i = 0; i < 8; i++) {
            uint16_t val = readMuxChannel(base_channel + i);
            // Giả sử vạch line tối màu cho giá trị analog lớn hơn nền sân (đảo dấu '<' nếu line trắng nền đen)
            if (val > ir_threshold) {
                float intensity = (float)(val - ir_threshold);
                weighted_sum += ir_weights[i] * intensity;
                total_active += intensity;
                active_count++;
            }
        }

        if (active_count == 0) {
            state.line_lost = true;
            // Giữ nguyên line_error gần nhất để xe tự đánh lái tìm lại vạch
        } else {
            state.line_lost = false;
            state.line_error = weighted_sum / total_active;
        }
    }

    // Kích xung siêu âm xen kẽ mỗi 25ms để tránh nhiễu sóng âm giữa 2 đầu
    void updateUltrasonic(RobotState &state) {
        uint32_t now = millis();
        if (now - last_us_trigger_ms >= 25) {
            last_us_trigger_ms = now;
            if (trigger_turn_a) {
                digitalWrite(PIN_US_A_TRIG, HIGH);
                delayMicroseconds(10);
                digitalWrite(PIN_US_A_TRIG, LOW);
            } else {
                digitalWrite(PIN_US_B_TRIG, HIGH);
                delayMicroseconds(10);
                digitalWrite(PIN_US_B_TRIG, LOW);
            }
            trigger_turn_a = !trigger_turn_a;
        }

        // Lấy kết quả từ biến ngắt theo chiều đang chạy
        uint32_t pulse = (state.direction == 1) ? us_a_pulse_us : us_b_pulse_us;
        if (pulse > 0 && pulse < 25000) { // Giới hạn < 4m
            state.front_obstacle_m = (pulse * 0.000343f) / 2.0f;
        } else {
            state.front_obstacle_m = 4.0f; // Không có vật cản
        }
    }

    // Đọc 2 cảm biến VL53L0X bắt mép trụ Load Port (Non-blocking)
    void updateSideToF(RobotState &state) {
        if (lox_a.timeoutOccurred() == false) {
            // Đọc giá trị mới nhất từ thanh ghi (không chờ)
            state.lox_a_dist_mm = lox_a.readReg16Bit(lox_a.RESULT_RANGE_STATUS + 10);
        }
        if (lox_b.timeoutOccurred() == false) {
            state.lox_b_dist_mm = lox_b.readReg16Bit(lox_b.RESULT_RANGE_STATUS + 10);
        }
    }

    // Đọc cảm biến ma trận 8x8 VL53L5CX với Dynamic ROI để phát hiện ô Die trống
    void updateDieScanner(RobotState &state) {
        SparkFun_VL53L5CX* active_tof = (state.direction == 1) ? &tof_l5_a : &tof_l5_b;
        
        if (active_tof->isDataReady()) {
            if (active_tof->getRangingData(&l5_data)) {
                uint8_t col_start = STATION_MAP[state.current_station_id].tof_roi_col_start;
                uint8_t col_end   = STATION_MAP[state.current_station_id].tof_roi_col_end;
                
                uint8_t empty_pixel_hits = 0;
                // Quét vùng trung tâm theo chiều dọc (hàng 2..5) và cột Dynamic ROI
                for (uint8_t row = 2; row <= 5; row++) {
                    for (uint8_t col = col_start; col <= col_end; col++) {
                        uint8_t idx = row * 8 + col;
                        uint16_t dist = l5_data.distance_mm[idx];
                        if (dist >= DIE_EMPTY_DIST_MIN && dist <= DIE_EMPTY_DIST_MAX) {
                            empty_pixel_hits++;
                        }
                    }
                }
                // Nếu có ít nhất 4 điểm ảnh trong vùng ROI nằm đúng khoảng cách mặt Wafer trống
                state.die_empty_detected = (empty_pixel_hits >= 4);
            }
        }
    }
};

#endif // SENSORS_DRIVER_H