/**
 * Giro-Robot — конфигурация
 * =========================
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Adafruit_MPU6050.h>

// ========== ПИНЫ ==========
#define STEPPER_1_STEP_PIN   3
#define STEPPER_1_DIR_PIN    5
#define STEPPER_2_STEP_PIN   9
#define STEPPER_2_DIR_PIN   10
#define STEPPER_ENABLE_PIN   8   // ENABLE TMC2208. При ошибке: HIGH = моторы мягкие. 0 = отключить (если пин не подключён)

// ========== IMU (MPU6050) ==========
#define IMU_ACCEL_RANGE     MPU6050_RANGE_2_G
#define IMU_GYRO_RANGE      MPU6050_RANGE_250_DEG
#define IMU_FILTER_BW       MPU6050_BAND_21_HZ

// ========== УПРАВЛЕНИЕ ==========
#define CONTROL_LOOP_HZ     100
#define CONTROL_DT          (1.0f / CONTROL_LOOP_HZ)

// ========== ОРИЕНТАЦИЯ (комплементарный фильтр) ==========
#define ORIENTATION_ALPHA   0.995f   // 0=только акселерометр, 1=только гироскоп
#define GYRO_PITCH_AXIS     0       // 0=gx, 1=gy, 2=gz — ось гироскопа для pitch (зависит от установки MPU)

// ========== МОТОРЫ ==========
#define MOTOR_SPEED_LIMIT   15625   // макс. шаг/с (ограничение Timer2), оба мотора одинаково

// ========== СТАБИЛИЗАЦИЯ ==========
#define LEAN_SCALE         5.0f     // linear [-1..1] → целевой угол (град)
#define BALANCE_SIGN       1       // -1 если мотор крутится не в ту сторону при попытке баланса
#define MOTOR2_INVERT       0  // 1 если одно колесо крутится в обратную сторону — оба должны ехать вперёд
#define MOTOR1_SCALE        1.0f
#define MOTOR2_SCALE        1.0f
#define MOTOR_DRIFT_CORRECTION 0.0f  // коррекция прокрутки при движении: +0.02 если крутит влево, -0.02 если вправо
#define TURN_SCALE 0.5f  // коэффициент поворота: при 1 на входе реально 0.5 (меньше = плавнее, без дёрганий)
#define TURN_BALANCE_BLEND 0.25f  // при повороте: 0=вращение вокруг оси (оба колеса), 1=опора на 1 колесо (меньше=крутится на месте)

// ========== КАСКАД: Speed PID (внешний) → Angle PID (внутренний) ==========
#define MAX_TARGET_SPEED_MPS  1.5f   // макс. целевая скорость м/с (ввод в м/с)
#define TARGET_SPEED_RAMP_MPS 0.25f  // макс. м/с² — меньше = плавнее, без рывков при остановке
#define SPEED_PID_KP         3.5f   // скорость (м/с) → угол (2.5 слабо для заднего хода)
#define SPEED_PID_KI         0.05f  // интеграл
#define SPEED_PID_KD         0.02f  // производная
#define SPEED_PID_ANGLE_LIMIT 10.0f  // макс. угол наклона от Speed PID (град)
#define ANGLE_OFFSET_SMOOTH  0.35f  // сглаживание angleOffset (0.2=плавнее, 0.5=быстрее)

// ========== PID (угол → моторы), внутренний контур ==========
#define PID_KP        280.0f
#define PID_KI        0.005f
#define PID_KD        0.0045f
#define PID_LIMIT     15625.0f   // макс. шаг/с на мотор (синхронно с MOTOR_SPEED_LIMIT)

// ========== ПАДЕНИЕ ==========
#define FALL_ANGLE_DEG      45.0f
#define FALL_RECOVERY_DEG   20.0f
#define FALL_DEBOUNCE_MS    80   // угол > FALL_ANGLE столько мс подряд — падение (фильтр ложных срабатываний)
#define RECOVERY_DEBOUNCE_MS 300 // угол < FALL_RECOVERY столько мс — можно восстанавливаться

// ========== СМЯГЧЕНИЕ при большой ошибке ==========
// При ошибке > порога — уменьшаем выход, моторы не дёргаются резко
#define SOFT_ERR_THRESHOLD   6.0f   // при ошибке > этого — начинаем смягчать выход
#define SOFT_ERR_MAX        22.0f   // при ошибке >= этого — минимум выхода
#define SOFT_ERR_MIN_SCALE  0.2f    // мин. коэффициент (20% выхода при большой ошибке)

// ========== ПОДЪЁМ (lift detection) ==========
// Вход: ошибка накапливается длительно (не уменьшается). Выход: ошибка начинает уменьшаться.
#define LIFT_ANGLE_ERROR_DEG  8.0f   // мин. ошибка угла (град) для входа в блок
#define LIFT_DEBOUNCE_MS      150    // ошибка не уменьшается столько мс → подъём
#define LIFT_RECOVERY_SAMPLES 5      // ошибка уменьшается столько отсчётов подряд → выход

// ========== ОТЛАДКА ==========
#define DEBUG_PRINT_MS      100
#define DEBUG_ENABLED       0
#define GRAPH_INTERVAL_MS   10   // Интервал вывода для Serial Plotter (мс)

#endif
