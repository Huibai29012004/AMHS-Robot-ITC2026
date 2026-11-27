#ifndef KINEMATICS_PHYSICS_H
#define KINEMATICS_PHYSICS_H

#include "robot_config.h"

class KinematicsPhysics {
public:
    // 1. Tính vận tốc dài tức thời tại tâm xe (m/s) từ mức PWM, điện áp pin và ma sát
    static float estimateLinearVelocity(int pwm_avg, float v_bat) {
        if (pwm_avg <= PWM_DEADZONE) {
            return 0.0f;
        }
        float effective_duty = (float)(pwm_avg - PWM_DEADZONE) / (float)(PWM_MAX_LIMIT - PWM_DEADZONE);
        float voltage_factor = v_bat / MOTOR_RATED_VOLTAGE;
        float v_center = effective_duty * V_MAX_THEORETICAL * voltage_factor;
        return constrain(v_center, 0.0f, 1.50f);
    }

    // 2. Tính vận tốc góc quanh tâm sân (rad/s)
    static float estimateAngularVelocity(float v_center) {
        return v_center / TRACK_RADIUS_R;
    }

    // 3. Tính vận tốc dài độc lập của bánh ngoài và bánh trong (m/s)
    static void getWheelVelocities(float v_center, float &v_outer, float &v_inner) {
        v_outer = v_center * RATIO_SPEED_OUTER;
        v_inner = v_center * RATIO_SPEED_INNER;
    }

    // 4. Tính PWM cơ sở (Feedforward) cho bánh ngoài và bánh trong để ôm cua tròn chuẩn
    static void getCurvedBasePWM(int base_pwm, int &pwm_outer, int &pwm_inner) {
        if (base_pwm <= PWM_DEADZONE) {
            pwm_outer = 0;
            pwm_inner = 0;
            return;
        }
        pwm_outer = constrain((int)(base_pwm * RATIO_SPEED_OUTER), 0, PWM_MAX_LIMIT);
        pwm_inner = constrain((int)(base_pwm * RATIO_SPEED_INNER), 0, PWM_MAX_LIMIT);
    }

    // 5. Cập nhật tích phân quãng đường cung tròn mỗi chu kỳ dt (2ms)
    static void updateOdometry(RobotState &state, float dt_sec) {
        state.current_vel_mps = estimateLinearVelocity(state.current_pwm_avg, state.battery_voltage);
        state.current_omega   = estimateAngularVelocity(state.current_vel_mps);

        float delta_s = state.current_vel_mps * dt_sec * state.direction;
        state.arc_traveled_m += delta_s;

        while (state.arc_traveled_m >= TRACK_CIRCUMFERENCE) {
            state.arc_traveled_m -= TRACK_CIRCUMFERENCE;
        }
        while (state.arc_traveled_m < 0.0f) {
            state.arc_traveled_m += TRACK_CIRCUMFERENCE;
        }

        state.dist_since_last_st += abs(delta_s);
    }

    // 6. Tính quãng đường phanh thẳng tới tâm trụ: D_phanh = k * v^2 (Đã bỏ D_creep)
    static float computeBrakingDistance(float v_current, float k_brake) {
        return k_brake * v_current * v_current;
    }

    // 7. Tính mức PWM giảm dần trong giai đoạn phanh (STATE_BRAKING) để dừng đúng tại tâm trụ
    static int computeBrakingPWM(float dist_remaining, float total_brake_dist) {
        if (dist_remaining <= 0.005f || total_brake_dist <= 0.01f) {
            return 0; // Đã tới tâm trụ -> Cắt xung kích hoạt phanh ghì BTS7960
        }
        float ratio = constrain(dist_remaining / total_brake_dist, 0.0f, 1.0f);
        int target_pwm = PWM_DEADZONE + (int)(ratio * (PWM_SCAN_ZONE - PWM_DEADZONE));
        return constrain(target_pwm, PWM_DEADZONE, PWM_SCAN_ZONE);
    }

    // 8. Khớp tọa độ & Sửa sai vị trí (Map Matching) khi đi qua 1 trụ
    static void syncStationOnPass(RobotState &state, bool is_first_station_ever) {
        if (is_first_station_ever) {
            state.current_station_id = 0;
            state.arc_traveled_m     = STATION_MAP.arc_position_m;
            state.dist_since_last_st = 0.0f;
            return;
        }

        int next_id = (state.current_station_id + state.direction + 10) % 10;

        if (state.dist_since_last_st > 1.00f) {
            if (state.direction == 1) {
                next_id = (state.current_station_id <= 4) ? 5 : 0;
            } else {
                next_id = (state.current_station_id >= 5) ? 4 : 9;
            }
        }

        state.current_station_id = next_id;
        state.arc_traveled_m     = STATION_MAP[next_id].arc_position_m;
        state.dist_since_last_st = 0.0f;
    }

    // 9. Tự hiệu chỉnh hệ số phanh k (Self-Calibration) từ sai số nhích sửa sai
    // align_error_m > 0: Xe trôi lố qua tâm trụ | align_error_m < 0: Xe dừng non trước tâm trụ
    static void calibrateBrakeCoeff(RobotState &state) {
        if (state.vel_at_brake_start < 0.25f) return; 
        if (abs(state.align_error_m) < 0.005f) return; // Sai số < 5mm coi như chuẩn, không cần sửa

        float delta_k = BRAKE_LEARNING_RATE * (state.align_error_m / (state.vel_at_brake_start * state.vel_at_brake_start));
        state.dynamic_brake_k = constrain(state.dynamic_brake_k + delta_k, 0.05f, 0.45f);
    }
};

#endif // KINEMATICS_PHYSICS_H