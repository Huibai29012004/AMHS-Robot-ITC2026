#include <Arduino_RouterBridge.h> // Thư viện giao tiếp RPC với nhân Linux QRB2210
#include "robot_config.h"
#include "kinematics_physics.h"
#include "sensors_driver.h"
#include "motion_control.h"

SensorsDriver sensors;
MotionControl motion;
RobotState    robot;

uint32_t last_loop_ms = 0;
bool first_station_synced = false;
bool prev_pillar_detected = false;
int8_t align_move_dir = 0; // Hướng nhích sửa sai trong STATE_ALIGN_CENTER (+1 tiến, -1 lùi)

// ============================================================================
// CÁC HÀM NHẬN LỆNH TỪ NHÂN LINUX (ĐIỆN THOẠI & CAMERA AI) QUA BRIDGE RPC
// ============================================================================
void onRemoteReleaseStation() {
    robot.flag_release_btn = true;
}

void onRemoteEmergencyStop(bool stop_state) {
    robot.flag_emergency_stop = stop_state;
}

void onAITargetStation(int station_id) {
    robot.ai_target_station = (int8_t)station_id;
}

// ============================================================================
// KHỞI TẠO HỆ THỐNG (SETUP)
// ============================================================================
void setup() {
    Serial.begin(115200);

    // 1. Khởi tạo trạng thái ban đầu của Hợp đồng dữ liệu
    robot.current_state       = STATE_SCAN_ZONE;
    robot.direction           = 1; // Mặc định xuất phát chạy tiến theo Đầu A
    robot.battery_voltage     = BATTERY_NOMINAL_VOLT;
    robot.dynamic_brake_k     = INITIAL_BRAKE_K;
    robot.ai_target_station   = -1;
    robot.flag_release_btn    = false;
    robot.flag_emergency_stop = false;

    // 2. Khởi tạo Động cơ & Cảm biến
    motion.init();
    if (!sensors.initAll()) {
        Serial.println("LOI KHOI TAO CAM BIEN I2C/TOF!");
        while (1) { delay(100); } // Khóa an toàn nếu thiếu cảm biến
    }

    // 3. Khởi tạo cầu nối Bridge với nhân Linux QRB2210
    Bridge.begin();
    Bridge.provide("release_station", onRemoteReleaseStation);
    Bridge.provide("emergency_stop",  onRemoteEmergencyStop);
    Bridge.provide("set_ai_target",   onAITargetStation);

    last_loop_ms = millis();
}

// ============================================================================
// VÒNG LẶP THỜI GIAN THỰC CỐ ĐỊNH 2MS (LOOP)
// ============================================================================
void loop() {
    uint32_t now = millis();
    if (now - last_loop_ms < LOOP_PERIOD_MS) {
        return; // Đảm bảo chu kỳ cứng đúng 2ms (500Hz)
    }
    last_loop_ms = now;

    // ------------------------------------------------------------------------
    // BƯỚC 1: CẬP NHẬT TOÀN BỘ CẢM BIẾN & ĐỘNG HỌC
    // ------------------------------------------------------------------------
    sensors.updateLineSensor(robot);
    sensors.updateUltrasonic(robot);
    sensors.updateSideToF(robot);
    sensors.updateDieScanner(robot);
    KinematicsPhysics::updateOdometry(robot, DT_SEC);

    // Kiểm tra đi ngang qua trụ Load Port để đồng bộ tọa độ trạm (Map Matching)
    bool lox_a_hit = (robot.lox_a_dist_mm > 10 && robot.lox_a_dist_mm < TOF_L0_DISTANCE_MAX);
    bool lox_b_hit = (robot.lox_b_dist_mm > 10 && robot.lox_b_dist_mm < TOF_L0_DISTANCE_MAX);
    bool any_pillar_hit = (lox_a_hit || lox_b_hit);

    if (any_pillar_hit && !prev_pillar_detected) {
        KinematicsPhysics::syncStationOnPass(robot, !first_station_synced);
        first_station_synced = true;
    }
    prev_pillar_detected = any_pillar_hit;

    // ------------------------------------------------------------------------
    // BƯỚC 2: BỘ PHÂN XỬ 3 LUỒNG ƯU TIÊN (ARBITRATION LOGIC)
    // ------------------------------------------------------------------------

    // >>> Dừng khẩn cấp từ điện thoại (Ưu tiên tuyệt đối)
    if (robot.flag_emergency_stop) {
        motion.hardBrake();
        robot.current_pwm_avg = 0;
        return;
    }

    // >>> LUỒNG 1: AN TOÀN & TRÁNH VA CHẠM (ƯU TIÊN SỐ 1)
    // Không kích hoạt khi đang đứng im giữ trạm
    if (robot.current_state != STATE_HOLD_STATION && robot.front_obstacle_m < OBSTACLE_STOP_DIST_M) {
        if (robot.current_state != STATE_OBSTACLE_STOP) {
            robot.current_state = STATE_OBSTACLE_STOP;
            robot.obstacle_timer_ms = now;
        }
        motion.hardBrake();
        robot.current_pwm_avg = 0;

        // Nếu vật cản đứng yên chặn đường quá 2.5 giây -> Tự động đảo chiều chạy
        if (now - robot.obstacle_timer_ms >= OBSTACLE_REVERSE_MS) {
            robot.direction = -robot.direction; // Đảo chiều (+1 <-> -1)
            robot.current_state = STATE_SCAN_ZONE;
            robot.obstacle_timer_ms = now;
        }
        return; // Ngắt ngang mọi luồng dưới
    } else if (robot.current_state == STATE_OBSTACLE_STOP) {
        // Vật cản đã rời đi trước 2.5s -> Tiếp tục hành trình
        robot.current_state = STATE_SCAN_ZONE;
    }

    // >>> LUỒNG 2: ĐIỀU PHỐI NÚT BẤM ĐIỆN THOẠI & LỆNH AI (ƯU TIÊN SỐ 2)
    if (robot.current_state == STATE_HOLD_STATION) {
        motion.hardBrake(); // Ghì cứng bánh xe giữ Load Port READY
        robot.current_pwm_avg = 0;

        if (robot.flag_release_btn) {
            robot.flag_release_btn = false; // Xóa cờ nút bấm
            // Chuyển sang vùng chạy nhanh nếu đang ở trạm cuối cụm (Trạm 4 hoặc Trạm 9)
            if (STATION_MAP[robot.current_station_id].is_fast_zone_after) {
                robot.current_state = STATE_CRUISE_FAST;
            } else {
                robot.current_state = STATE_SCAN_ZONE;
            }
        }
        return;
    }

    // ------------------------------------------------------------------------
    // BƯỚC 3: MÁY TRẠNG THÁI TỰ HÀNH (LUỒNG 3 - FSM)
    // ------------------------------------------------------------------------
    switch (robot.current_state) {

        // 1. CHẠY TỐC ĐỘ CAO QUA VÙNG TRỐNG (5h-7h và 11h-1h)
        case STATE_CRUISE_FAST: {
            motion.updateLineFollowing(robot, PWM_CRUISE_FAST);
            // Khi đi được > 2.8m trong vùng trống (sắp chạm cụm trạm tiếp theo) -> Giảm tốc quét
            if (robot.dist_since_last_st >= 2.80f) {
                robot.current_state = STATE_SCAN_ZONE;
            }
            break;
        }

        // 2. GIẢM TỐC QUÉT Ô DIE TRỐNG
        case STATE_SCAN_ZONE: {
            motion.updateLineFollowing(robot, PWM_SCAN_ZONE);

            // Kiểm tra điều kiện chiếm trạm:
            // Nếu có lệnh AI chỉ định -> chỉ dừng đúng trạm AI yêu cầu
            // Nếu không có lệnh AI -> dừng ở bất kỳ trạm nào có ô Die trống
            bool should_claim = false;
            if (robot.ai_target_station != -1) {
                should_claim = (robot.current_station_id == robot.ai_target_station && any_pillar_hit);
            } else {
                should_claim = (robot.die_empty_detected && any_pillar_hit);
            }

            if (should_claim) {
                robot.vel_at_brake_start = robot.current_vel_mps;
                robot.target_brake_dist  = KinematicsPhysics::computeBrakingDistance(robot.vel_at_brake_start, robot.dynamic_brake_k);
                robot.brake_traveled_m   = 0.0f;
                robot.current_state      = STATE_BRAKING;
            }
            break;
        }

        // 3. PHANH THẲNG TỚI TÂM TRỤ LOAD PORT
        case STATE_BRAKING: {
            robot.brake_traveled_m += robot.current_vel_mps * DT_SEC;
            float dist_rem = robot.target_brake_dist - robot.brake_traveled_m;
            int brake_pwm = KinematicsPhysics::computeBrakingPWM(dist_rem, robot.target_brake_dist);

            motion.updateLineFollowing(robot, brake_pwm);

            // Khi đã hết quãng đường phanh và xe dừng hẳn
            if (brake_pwm == 0) {
                motion.hardBrake();
                robot.align_error_m = 0.0f;

                // Kiểm tra ngay 2 mắt VL53L0X xem đã khóa trúng tâm trụ chưa
                if (lox_a_hit && lox_b_hit) {
                    // Trúng tâm ngay lập tức! Không cần nhích sửa sai
                    robot.ai_target_station = -1;
                    robot.current_state = STATE_HOLD_STATION;
                } else {
                    // Lệch tâm -> Chuyển sang lưới an toàn ALIGN_CENTER để nhích sửa sai & học k
                    robot.current_state = STATE_ALIGN_CENTER;
                }
            }
            break;
        }

        // 4. LƯỚI AN TOÀN: VI CHỈNH TÂM TRỤ & ĐO SAI SỐ TỰ HỌC HỆ SỐ k
        case STATE_ALIGN_CENTER: {
            if (lox_a_hit && lox_b_hit) {
                // Đã khóa trúng cả 2 mắt -> Dừng hẳn, cập nhật hệ số k và chuyển sang giữ trạm
                motion.hardBrake();
                robot.current_pwm_avg = 0;
                KinematicsPhysics::calibrateBrakeCoeff(robot);
                robot.ai_target_station = -1;
                robot.current_state = STATE_HOLD_STATION;
            } else {
                // Xác định hướng nhích sửa sai và dấu sai số (+ lố, - non)
                // Mắt A nằm phía Đầu A (+1), Mắt B nằm phía Đầu B (-1)
                if (lox_a_hit && !lox_b_hit) {
                    // Chỉ mắt A thấy trụ -> Trụ đang lệch về phía Đầu A -> Cần tiến về phía A (+1)
                    align_move_dir = 1;
                } else if (!lox_a_hit && lox_b_hit) {
                    // Chỉ mắt B thấy trụ -> Trụ đang lệch về phía Đầu B -> Cần lùi về phía B (-1)
                    align_move_dir = -1;
                } else {
                    // Cả 2 mắt đều vượt qua trụ -> Đảo ngược chiều chạy trước đó để lùi lại tìm trụ
                    align_move_dir = -robot.direction;
                }

                motion.creepMove(align_move_dir);
                robot.current_pwm_avg = PWM_CREEP_SPEED;

                // Tính quãng đường nhích được trong chu kỳ 2ms
                float v_creep = KinematicsPhysics::estimateLinearVelocity(PWM_CREEP_SPEED, robot.battery_voltage);
                float step_m  = v_creep * DT_SEC;

Use code with caution.
// Nếu hướng nhích ngược với hướng chạy ban đầu -> Xe bị trôi lố (+), ngược lại là dừng non (-)
if (align_move_dir != robot.direction) {
robot.align_error_m += step_m; // Lố -> tăng k
} else {
robot.align_error_m -= step_m; // Non -> giảm k
}
}
break;
}
default:
break;
}
}