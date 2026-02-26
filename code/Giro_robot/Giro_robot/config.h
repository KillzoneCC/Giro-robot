/**
 * Giro-Robot — конфигурация
 * =========================
 * Все параметры робота и настройки в одном месте.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Adafruit_MPU6050.h>

// ========== ХАРАКТЕРИСТИКИ РОБОТА ==========
#define ROBOT_WHEEL_DIAMETER_MM    72    // Диаметр колеса (мм)
#define ROBOT_WEIGHT_KG            1.1f  // Масса (кг)
#define ROBOT_WIDTH_MM             134   // Ширина (мм)
#define ROBOT_HEIGHT_MM            146   // Высота (мм)

// ========== ПИНЫ ==========
#define STEPPER_1_STEP_PIN   3
#define STEPPER_1_DIR_PIN    5
#define STEPPER_2_STEP_PIN   9
#define STEPPER_2_DIR_PIN   10

// ========== IMU (MPU6050) ==========
#define IMU_ACCEL_RANGE     MPU6050_RANGE_2_G
#define IMU_GYRO_RANGE      MPU6050_RANGE_250_DEG
#define IMU_FILTER_BW       MPU6050_BAND_21_HZ

// ========== УПРАВЛЕНИЕ ==========
#define CONTROL_LOOP_HZ     100
#define CONTROL_DT          (1.0f / CONTROL_LOOP_HZ)

// Twist linear: -1..1 (нормализовано, НЕ м/с). 1 = макс. скорость.
// Внутри: linear*LINEAR_TO_STEPS -> steps/s для PID угла.
#define LINEAR_TO_STEPS     2000.0f   // stabilizer: targetSpeed = linear * this
#define MAX_STEPS_PER_SEC   14000.0f  // моторы: steps/s при linear=1

// Ось баланса: 0 = roll (влево-направо), 1 = pitch (вперёд-назад)
#define USE_PITCH_AXIS      0

// Знак обратной связи: если робот ускоряет падение при наклоне — поставь -1
#define BALANCE_SIGN        1

// Инверсия второго мотора (если крутится на месте вместо движения)
#define MOTOR2_INVERT       0

// Масштаб моторов (если один крутится медленнее)
#define MOTOR1_SCALE        1.0f
#define MOTOR2_SCALE        1.0f

// ========== PID — каскад (скорость → угол → моторы) ==========
// Внешний контур: targetSpeed → targetAngle
#define PID_ANGLE_KP        0.0006f
#define PID_ANGLE_KI        0.00002f
#define PID_ANGLE_KD        0.00001f
#define PID_ANGLE_LIMIT     5.0f

// Внутренний контур: targetAngle → motorSpeed (steps/s)
#define PID_MOTOR_KP        280.0f
#define PID_MOTOR_KI        0.005f
#define PID_MOTOR_KD        0.0045f
#define PID_MOTOR_LIMIT     7500.0f

// ========== МЁРТВАЯ ЗОНА ==========
#define DEADBAND_DEG        1.5f

// ========== ОБНАРУЖЕНИЕ ПАДЕНИЯ ==========
#define FALL_ANGLE_DEG      45.0f
#define FALL_RECOVERY_DEG   20.0f

// ========== Slew rate (плавность изменения скорости моторов) ==========
#define SLEW_RATE_MAX       1100.0f

// ========== ОТЛАДКА ==========
#define DEBUG_PRINT_MS      100
#define DEBUG_ENABLED      1

#endif
