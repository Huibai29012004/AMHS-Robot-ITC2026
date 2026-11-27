#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <Arduino.h>

// ============================================================================
// 1. SƠ ĐỒ PHÂN CHÂN PHẦN CỨNG (ARDUINO UNO Q - STM32U585 3.3V LOGIC)
// ============================================================================

// --- Cụm Động cơ & Mạch cầu H BTS7960 (PWM 3.3V) ---
#define PIN_MOTOR_L_RPWM      3     // PWM Bánh Trái - Tiến (Đầu A) [Lưu ý: PB0 max 3.6V]
#define PIN_MOTOR_L_LPWM      5     // PWM Bánh Trái - Lùi (Đầu B)
#define PIN_MOTOR_R_RPWM      6     // PWM Bánh Phải - Tiến (Đầu A)
#define PIN_MOTOR_R_LPWM      9     // PWM Bánh Phải - Lùi (Đầu B)
// R_EN và L_EN nối cứng lên 3.3V để kích hoạt phanh động lực học khi PWM = 0

// --- Dò line 2 chiều qua MUX 16 kênh CD74HC4067 (Cấp nguồn VCC = 3.3V) ---
#define PIN_IR_MUX_SIG        A0    // Tín hiệu Analog trả về (PA4 - Tuyệt đối max 3.3V)
#define PIN_IR_MUX_S0         2
#define PIN_IR_MUX_S1         4
#define PIN_IR_MUX_S2         7
#define PIN_IR_MUX_S3         8

// --- Siêu âm 2 chiều HC-SR04 (Qua cầu phân áp 1k/2k xuống 3.3V) ---
#define PIN_US_A_TRIG         11    // Siêu âm Đầu A
#define PIN_US_A_ECHO         12
#define PIN_US_B_TRIG         13    // Siêu âm Đầu B
#define PIN_US_B_ECHO         A1    // PA5 - Tuyệt đối max 3.3V

// --- ToF Đa điểm VL53L5CX (Bus I2C2 chính: D20 SDA, D21 SCL) ---
#define PIN_TOF_L5_A_LPN      A2    // Chân bật/tắt I2C cho mắt ToF 8x8 phía Đầu A
#define PIN_TOF_L5_B_LPN      A3    // Chân bật/tắt I2C cho mắt ToF 8x8 phía Đầu B
#define TOF_L5_A_I2C_ADDR     0x30  // Địa chỉ I2C gán lại cho mắt A
#define TOF_L5_B_I2C_ADDR     0x32  // Địa chỉ I2C gán lại cho mắt B

// --- ToF Đơn điểm VL53L0X (Bắt mép trụ Load Port - 2 Bus I2C độc lập) ---
// L0X_A dùng I2C3 (A4 SDA, A5 SCL) | L0X_B dùng cổng QWIIC (I2C4)
#define TOF_L0_DISTANCE_MAX   160   // Ngưỡng khoảng cách phát hiện trụ Load Port (mm)

// ============================================================================
// 2. THÔNG SỐ CƠ KHÍ SÂN ĐẤU & KHUNG XE (ĐƠN VỊ CHUẨN: MÉT, KG, GIÂY)
// ============================================================================
constexpr float TRACK_LINE_DIAMETER   = 4.075f; // Tâm vạch line 50mm (mép trong 4050mm, ngoài 4100mm)
constexpr float TRACK_RADIUS_R        = TRACK_LINE_DIAMETER / 2.0f; // Bán kính quỹ đạo tâm xe R = 2.0375 m
constexpr float TRACK_CIRCUMFERENCE   = PI * TRACK_LINE_DIAMETER;   // Chu vi 1 vòng track = 12.802 m

constexpr float ROBOT_MASS_KG         = 3.00f;  // Khối lượng toàn tải của robot (kg)
constexpr float ROBOT_TRACK_WIDTH_W   = 0.280f; // Khoảng cách tâm 2 bánh Trái - Phải (m)
constexpr float WHEEL_DIAMETER_M      = 0.065f; // Đường kính bánh cao su 65mm (m)
constexpr float WHEEL_RADIUS_M        = WHEEL_DIAMETER_M / 2.0f;

// Bán kính quỹ đạo bánh ngoài và bánh trong khi ôm cua tròn
constexpr float RADIUS_WHEEL_OUTER    = TRACK_RADIUS_R + (ROBOT_TRACK_WIDTH_W / 2.0f);
constexpr float RADIUS_WHEEL_INNER    = TRACK_RADIUS_R - (ROBOT_TRACK_WIDTH_W / 2.0f);
// Tỷ lệ vận tốc vòng cung (Feedforward): Bánh ngoài / Tâm và Bánh trong / Tâm
constexpr float RATIO_SPEED_OUTER     = RADIUS_WHEEL_OUTER / TRACK_RADIUS_R; // ~1.0687
constexpr float RATIO_SPEED_INNER     = RADIUS_WHEEL_INNER / TRACK_RADIUS_R; // ~0.9313

// ============================================================================
// 3. THÔNG SỐ ĐỘNG CƠ, PIN & HIỆU CHỈNH THỰC NGHIỆM (CẬP NHẬT NGÀY 30/11)
// ============================================================================
constexpr float MOTOR_RATED_RPM       = 421.0f; // Tốc độ định mức động cơ 36GP-555 (RPM)
constexpr float MOTOR_RATED_VOLTAGE   = 12.0f;  // Điện áp định mức động cơ (V)
constexpr float BATTERY_NOMINAL_VOLT  = 14.8f;  // Điện áp danh định khối pin LiPo 4S (V)
constexpr float MOTOR_STALL_TORQUE    = 0.441f; // Mô-men xoắn định mức ~4.5 kg.cm = 0.441 N.m
constexpr float GRAVITY_G             = 9.81f;

// Các thông số hiệu chỉnh thực tế trên sân decal cán mờ:
constexpr float FLOOR_FRICTION_COEFF  = 0.55f;  // Hệ số ma sát nghỉ ngang bánh - sàn (đo ngày 30/11)
constexpr float DRIVETRAIN_EFFICIENCY = 0.85f;  // Hệ số hiệu suất truyền động thực tế (0.0 - 1.0)
constexpr int   PWM_DEADZONE          = 40;     // Mức PWM tối thiểu để xe bắt đầu lăn bánh (0-255)
constexpr int   PWM_MAX_LIMIT         = 255;

// Vận tốc lý thuyết tối đa tại 100% PWM (12V): v = (RPM / 60) * PI * D * hiệu suất
constexpr float V_MAX_THEORETICAL     = (MOTOR_RATED_RPM / 60.0f) * PI * WHEEL_DIAMETER_M * DRIVETRAIN_EFFICIENCY;

// Tham số Phanh thẳng tới tâm trụ & Tự học (Self-Calibration)
constexpr float INITIAL_BRAKE_K       = 0.16f;  // Hệ số phanh khởi tạo: D_phanh = k * v^2
constexpr float BRAKE_LEARNING_RATE   = 0.30f;  // Tốc độ tự học alpha (0.1 - 0.5): Chỉnh cao để học nhanh hơn

// ============================================================================
// 4. THAM SỐ ĐIỀU KHIỂN THỜI GIAN THỰC & CẢM BIẾN
// ============================================================================
constexpr uint32_t LOOP_PERIOD_MS     = 2;      // Chu kỳ vòng lặp chính (2ms = 500Hz)
constexpr float    DT_SEC             = LOOP_PERIOD_MS / 1000.0f;

// Dải tốc độ PWM theo từng trạng thái
constexpr int PWM_CRUISE_FAST         = 215;    // Chạy nhanh ở 2 vùng trống (5h-7h, 11h-1h)
constexpr int PWM_SCAN_ZONE           = 145;    // Giảm tốc để ToF soi ô Die chính xác
constexpr int PWM_CREEP_SPEED         = 65;     // Tốc độ bò cực chậm khi vi chỉnh sửa sai tâm trụ
constexpr int PWM_RAMP_STEP           = 4;      // Bước tăng/giảm PWM mỗi 2ms (Chống trượt bánh)

// Ngưỡng siêu âm & Tránh va chạm (Luồng 1)
constexpr float OBSTACLE_WARN_DIST_M  = 0.45f;  // Bắt đầu giảm tốc khi gặp vật cản < 45cm
constexpr float OBSTACLE_STOP_DIST_M  = 0.25f;  // Phanh chết khẩn cấp khi vật cản < 25cm
constexpr uint32_t OBSTACLE_REVERSE_MS= 2500;   // Đảo chiều nếu bị chặn quá 2.5 giây (2500ms)

// Ngưỡng nhận diện ô Die trống trên ToF VL53L5CX (Đơn vị: mm)
constexpr uint16_t DIE_EMPTY_DIST_MIN = 550;    // Cần đo thực tế góc nghiêng 15 độ trên sân
constexpr uint16_t DIE_EMPTY_DIST_MAX = 950;

// ============================================================================
// 5. BẢN ĐỒ TỌA ĐỘ 10 TRẠM LOAD PORT (ĐỐI XỨNG TÂM 180 ĐỘ)
// ============================================================================
struct StationMapNode {
    uint8_t id;
    float   arc_position_m;     // Tọa độ cung tròn tính từ Trạm 0 (m)
    bool    is_fast_zone_after; // True nếu sau trạm này là khoảng trống lớn (5h-7h hoặc 11h-1h)
    uint8_t tof_roi_col_start;  // Cột bắt đầu của cửa sổ quét Dynamic ROI (0..7)
    uint8_t tof_roi_col_end;    // Cột kết thúc của cửa sổ quét Dynamic ROI (0..7)
};

// 5 trạm cụm bên trái (ID 0..4) -> Khoảng trống 12h -> 5 trạm cụm bên phải (ID 5..9) -> Khoảng trống 6h
const StationMapNode STATION_MAP = {
    {0, 0.00f, false, 2, 5}, // Trạm 0 (Gần góc xuất phát)
    {1, 0.64f, false, 2, 5}, // Trạm 1
    {2, 1.28f, false, 3, 6}, // Trạm 2 (Góc 9h - Dịch ROI)
    {3, 1.92f, false, 2, 5}, // Trạm 3
    {4, 2.56f, true,  2, 5}, // Trạm 4 -> Sau trạm này là vùng trống lớn 11h-1h (CRUISE_FAST)
    {5, 6.40f, false, 2, 5}, // Trạm 5 (Đối xứng 180 độ với Trạm 0: ~6.40m = nửa chu vi)
    {6, 7.04f, false, 2, 5}, // Trạm 6
    {7, 7.68f, false, 3, 6}, // Trạm 7 (Góc 3h - Dịch ROI)
    {8, 8.32f, false, 2, 5}, // Trạm 8
    {9, 8.96f, true,  2, 5}  // Trạm 9 -> Sau trạm này là vùng trống lớn 5h-7h (CRUISE_FAST)
};

// ============================================================================
// 6. HỢP ĐỒNG DỮ LIỆU DÙNG CHUNG (DATA CONTRACT - GLOBAL STATE STRUCT)
// ============================================================================
enum RobotFSMState {
    STATE_CRUISE_FAST = 0,    // 1. Chạy tốc độ cao qua vùng trống
    STATE_SCAN_ZONE,          // 2. Giảm tốc quét ToF tìm ô Die trống
    STATE_BRAKING,            // 3. Phanh thẳng tới tâm trụ Load Port theo D_phanh = k * v^2
    STATE_ALIGN_CENTER,       // 4. Lưới an toàn: Vi chỉnh & đo sai số dừng nếu chưa trúng tâm trụ
    STATE_HOLD_STATION,       // 5. Đứng im kích hoạt Load Port chờ nút nhả trạm từ điện thoại
    STATE_OBSTACLE_STOP       // 6. Luồng 1 ghi đè: Phanh khẩn cấp / Đảo chiều tránh vật cản
};

struct RobotState {
    // --- Trạng thái hệ thống & Chiều di chuyển ---
    RobotFSMState current_state;
    int8_t        direction;          // +1: Tiến về Đầu A | -1: Lùi về Đầu B
    float         battery_voltage;    // Điện áp pin hiện tại (V)

    // --- Dữ liệu từ Tầng Cảm biến (sensors_driver) ---
    float         line_error;         // Độ lệch tâm line (-1.0 đến +1.0)
    bool          line_lost;          // Cờ báo mất vạch line
    float         front_obstacle_m;   // Khoảng cách siêu âm theo hướng đang chạy (m)
    bool          die_empty_detected; // ToF VL53L5CX phát hiện ô Die trống
    uint16_t      lox_a_dist_mm;      // Khoảng cách mép trụ mắt VL53L0X A (mm)
    uint16_t      lox_b_dist_mm;      // Khoảng cách mép trụ mắt VL53L0X B (mm)

    // --- Dữ liệu Động học & Định vị (kinematics_physics) ---
    int           current_pwm_avg;    // Mức PWM trung bình hiện tại (0..255)
    float         current_vel_mps;    // Vận tốc dài ước lượng tại tâm xe (m/s)
    float         current_omega;      // Vận tốc góc quanh tâm sân (rad/s)
    float         arc_traveled_m;     // Tổng quãng đường cung tròn đã đi từ gốc (m)
    float         dist_since_last_st; // Quãng đường đi được kể từ trạm gần nhất (m)
    uint8_t       current_station_id; // ID trạm hiện tại/gần nhất (0..9)
    
    // --- Biến phục vụ Phanh thẳng & Tự học k ---
    float         dynamic_brake_k;    // Hệ số phanh k (được tự học sau mỗi trạm)
    float         vel_at_brake_start; // Vận tốc xe ngay thời điểm bắt đầu bóp phanh (m/s)
    float         target_brake_dist;  // Quãng đường phanh mục tiêu D_phanh = k * v^2 (m)
    float         brake_traveled_m;   // Quãng đường thực tế đã trôi trong lúc phanh (m)
Use code with caution.
float         align_error_m;      // Quãng đường nhích sửa sai đo được (+ lố, - non) (m)
// --- Cờ Giao tiếp 3 Luồng Ưu tiên (Arbitration Flags) ---
bool          flag_obstacle_stop; // Luồng 1: Yêu cầu dừng khẩn cấp từ siêu âm
uint32_t      obstacle_timer_ms;  // Luồng 1: Bộ đếm thời gian bị chặn để đảo chiều 2.5s
bool          flag_release_btn;   // Luồng 2: Tín hiệu bấm nút từ điện thoại nhả trạm
bool          flag_emergency_stop;// Luồng 2: Tín hiệu dừng khẩn cấp từ điện thoại
int8_t        ai_target_station;  // Luồng 2: ID trạm AI chỉ định (-1 nếu chạy tự do)
};
#endif // ROBOT_CONFIG_H