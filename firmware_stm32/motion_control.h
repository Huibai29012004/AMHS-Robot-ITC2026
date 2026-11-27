#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "robot_config.h"
#include "kinematics_physics.h"

class MotionControl {
private:
    // Thông số PID bám line (Cần tinh chỉnh thực tế trên đường tròn Ø4.0m)
    float Kp = 85.0f;
    float Ki = 0.05f;
    float Kd = 18.0f;

    float prev_error = 0.0f;
    float integral_error = 0.0f;
    int   current_base_pwm = 0; // Giá trị PWM hiện tại sau khi qua bộ lọc dốc (Velocity Ramp)

public:
    void init() {
        pinMode(PIN_MOTOR_L_RPWM, OUTPUT);
        pinMode(PIN_MOTOR_L_LPWM, OUTPUT);
        pinMode(PIN_MOTOR_R_RPWM, OUTPUT);
        pinMode(PIN_MOTOR_R_LPWM, OUTPUT);
        
        // Cài đặt tần số PWM cao (20kHz) trên STM32U585 để động cơ chạy êm
        analogWriteFrequency(20000);
        hardBrake();
    }

    // Phanh động lực học BTS7960: Xuất PWM = 0 trong khi R_EN & L_EN luôn ở mức 3.3V
    // Tuyệt đối không đảo chiều động cơ khi phanh để tránh trượt bánh mất tọa độ
    void hardBrake() {
        current_base_pwm = 0;
        integral_error = 0.0f;
        analogWrite(PIN_MOTOR_L_RPWM, 0);
        analogWrite(PIN_MOTOR_L_LPWM, 0);
        analogWrite(PIN_MOTOR_R_RPWM, 0);
        analogWrite(PIN_MOTOR_R_LPWM, 0);
    }

    // Bộ tạo dốc gia tốc (Velocity Ramp) chống trượt bánh và sụt áp nguồn
    int applyVelocityRamp(int target_pwm) {
        if (target_pwm == 0) {
            current_base_pwm = 0;
            return 0;
        }
        if (current_base_pwm < target_pwm) {
            current_base_pwm = min(current_base_pwm + PWM_RAMP_STEP, target_pwm);
        } else if (current_base_pwm > target_pwm) {
            current_base_pwm = max(current_base_pwm - (PWM_RAMP_STEP * 2), target_pwm); // Giảm tốc nhanh gấp đôi tăng tốc
        }
        return current_base_pwm;
    }

    // Vòng lặp điều khiển bám line 2 chiều (Chạy cố định mỗi 2ms)
    void updateLineFollowing(RobotState &state, int target_speed_pwm) {
        if (target_speed_pwm <= 0) {
            hardBrake();
            state.current_pwm_avg = 0;
            return;
        }

        // 1. Làm mượt tốc độ mục tiêu qua Velocity Ramp
        int smooth_pwm = applyVelocityRamp(target_speed_pwm);

        // 2. Tính PID từ độ lệch tâm line (-1.0 đến +1.0)
        float error = state.line_error;
        integral_error = constrain(integral_error + error * DT_SEC, -0.5f, 0.5f);
        float derivative = (error - prev_error) / DT_SEC;
        prev_error = error;

        float pid_correction = (Kp * error) + (Ki * integral_error) + (Kd * derivative);

        // 3. Tính tốc độ cơ sở vòng cung (Feedforward) cho bánh ngoài và bánh trong
        // Lưu ý: Trên sân tròn chạy ngược chiều kim đồng hồ (CCW) từ góc 7h -> 1h:
        // Bên Phải (R) là bánh vòng ngoài, bên Trái (L) là bánh vòng trong ôm sát Wafer.
        int pwm_outer, pwm_inner;
        KinematicsPhysics::getCurvedBasePWM(smooth_pwm, pwm_outer, pwm_inner);

        int pwm_left, pwm_right;
        if (state.direction == 1) {
            // Chạy tiến (Đầu A): Phải là ngoài, Trái là trong
            pwm_right = pwm_outer - (int)pid_correction;
            pwm_left  = pwm_inner + (int)pid_correction;
        } else {
            // Chạy lùi (Đầu B): Khi lùi thì bánh Trái (nhìn từ đầu B) trở thành bánh ngoài
            pwm_left  = pwm_outer - (int)pid_correction;
            pwm_right = pwm_inner + (int)pid_correction;
        }

        pwm_left  = constrain(pwm_left,  0, PWM_MAX_LIMIT);
        pwm_right = constrain(pwm_right, 0, PWM_MAX_LIMIT);

        // 4. Xuất xung PWM ra 2 mạch cầu H BTS7960 theo chiều di chuyển
        if (state.direction == 1) {
            analogWrite(PIN_MOTOR_L_RPWM, pwm_left);
            analogWrite(PIN_MOTOR_L_LPWM, 0);
            analogWrite(PIN_MOTOR_R_RPWM, pwm_right);
            analogWrite(PIN_MOTOR_R_LPWM, 0);
        } else {
            analogWrite(PIN_MOTOR_L_RPWM, 0);
            analogWrite(PIN_MOTOR_L_LPWM, pwm_left);
            analogWrite(PIN_MOTOR_R_RPWM, 0);
            analogWrite(PIN_MOTOR_R_LPWM, pwm_right);
        }

        // Lưu lại mức PWM trung bình thực tế để tính Odometry
        state.current_pwm_avg = (pwm_left + pwm_right) / 2;
    }

    // Điều khiển nhích vi chỉnh tiến/lùi cực chậm trong trạng thái ALIGN_CENTER
    void creepMove(int8_t move_dir) {
        if (move_dir == 0) {
            hardBrake();
            return;
        }
        if (move_dir > 0) {
            analogWrite(PIN_MOTOR_L_RPWM, PWM_CREEP_SPEED);
            analogWrite(PIN_MOTOR_L_LPWM, 0);
            analogWrite(PIN_MOTOR_R_RPWM, PWM_CREEP_SPEED);
            analogWrite(PIN_MOTOR_R_LPWM, 0);
        } else {
            analogWrite(PIN_MOTOR_L_RPWM, 0);
            analogWrite(PIN_MOTOR_L_LPWM, PWM_CREEP_SPEED);
            analogWrite(PIN_MOTOR_R_RPWM, 0);
            analogWrite(PIN_MOTOR_R_LPWM, PWM_CREEP_SPEED);
        }
    }
};

#endif // MOTION_CONTROL_H