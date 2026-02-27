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

// ========== IMU (MPU6050) ==========
#define IMU_ACCEL_RANGE     MPU6050_RANGE_2_G
#define IMU_GYRO_RANGE      MPU6050_RANGE_250_DEG
#define IMU_FILTER_BW       MPU6050_BAND_21_HZ

// ========== УПРАВЛЕНИЕ ==========
#define CONTROL_LOOP_HZ     100
#define CONTROL_DT          (1.0f / CONTROL_LOOP_HZ)

// ========== ОРИЕНТАЦИЯ (комплементарный фильтр) ==========
#define ORIENTATION_ALPHA   0.995f   // 0=только акселерометр, 1=только гироскоп

// ========== СТАБИЛИЗАЦИЯ ==========
#define LEAN_SCALE         5.0f     // linear [-1..1] → целевой угол (град)
#define BALANCE_SIGN       1
#define MOTOR2_INVERT       0
#define MOTOR1_SCALE        1.0f
#define MOTOR2_SCALE        1.0f

// ========== PID (угол → моторы) ==========
// Значения по умолчанию. Сохраняются в EEPROM после калибровки.
#define PID_KP        280.0f
#define PID_KI        0.005f
#define PID_KD        0.0045f
#define PID_LIMIT     7500.0f

// ========== ПАДЕНИЕ ==========
#define FALL_ANGLE_DEG      45.0f
#define FALL_RECOVERY_DEG   20.0f

// ========== ОТЛАДКА ==========
#define DEBUG_PRINT_MS      100
#define DEBUG_ENABLED       0
#define GRAPH_INTERVAL_MS   10   // Интервал вывода для Serial Plotter (мс)

#endif
