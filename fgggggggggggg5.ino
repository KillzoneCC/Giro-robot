#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "pidbalanse.h"
#include "balancer.h"
#include "shag.h"  // Модуль управления шаговым двигателем

Adafruit_MPU6050 mpu;

// ================= НАСТРОЙКИ ВЫВОДА =================
// 0: старый формат (ax,ay,az,gx,gy,gz) — но gx/gy/gz в deg/s
// 1: только гироскоп (gx,gy,gz) в deg/s
// 2: углы (roll,pitch,yaw) ТОЛЬКО по гироскопу, в градусах
// 3: углы (roll,pitch,yaw) + выход PID (u_roll,u_pitch)
// 4: Serial Plotter: roll,target_roll,u_roll,pwm (ТОЛЬКО числа)
static constexpr int OUTPUT_MODE = 4;
static constexpr bool SERIAL_PLOTTER_MODE = (OUTPUT_MODE == 4);

static constexpr uint16_t CALIB_SAMPLES = 1500; // калибровка нуля гироскопа (больше = точнее)
static constexpr uint16_t LOOP_DELAY_MS = 5;   // 5ms ≈ 200Hz (50ms было ≈ 20Hz)
static constexpr float CALIB_MAX_STD_DPS = 0.5f; // максимальное стандартное отклонение для "хорошей" калибровки

// В Arduino уже есть макрос RAD_TO_DEG, поэтому своё имя не используем.
static constexpr float RAD_TO_DEG_F = 57.29577951308232f;
static constexpr float DEG_TO_RAD_F = 0.017453292519943295f;

// --- Параметры модели с фото (Free fall: масса m на колесе M, радиус R, стержень L) ---
// Момент инерции колеса I = (1/2)*M*R² (диск)
static constexpr float MODEL_M = 0.5f;   // масса колеса (кг)
static constexpr float MODEL_m = 0.3f;   // масса верхней части (кг)
static constexpr float MODEL_R = 0.05f;  // радиус колеса (м)
static constexpr float MODEL_L = 0.2f;   // длина стержня (м)
static constexpr float MODEL_g = 9.81f;  // g (м/с²)
static constexpr float MODEL_I = 0.5f * MODEL_M * MODEL_R * MODEL_R;  // момент инерции колеса
// Коэффициент масштаба формулы с фото к диапазону PWM (подбирается по железу)
static constexpr float MODEL_FF_GAIN = 0.02f;

// --- АНТИ-ДРЕЙФ (только по гироскопу) ---
// Если плата НЕ вращается, считаем что истинная угловая скорость ~0,
// значит текущие значения гироскопа ≈ bias. Обновляем bias медленно.
static constexpr float STATIONARY_THRESH_DPS = 1.0f;   // порог "не вращаемся"
static constexpr uint16_t STATIONARY_MIN_MS = 700;     // сколько держать в покое
static constexpr float BIAS_LPF_ALPHA = 0.01f;         // скорость подстройки bias (0..1)
uint32_t stationary_ms = 0;
// ---------------------------------------

float gyro_bias_x_dps = 0.0f;
float gyro_bias_y_dps = 0.0f;
float gyro_bias_z_dps = 0.0f;

float roll_deg = 0.0f;
float pitch_deg = 0.0f;
float yaw_deg = 0.0f;

uint32_t last_us = 0;
// =====================================================

// ================= PID (пример для стабилизации) =================
// Цель: удерживать roll/pitch около 0 градусов. Выход PID — "управление"
// (например PWM/тяга), здесь просто считаем и (в OUTPUT_MODE==3) печатаем.
static constexpr float PID_LIMIT = 255.0f; // типичный диапазон PWM
static constexpr float PID_ROLL_KP = 6.0f;
static constexpr float PID_ROLL_KI = 0.0f;
static constexpr float PID_ROLL_KD = 0.15f;
static constexpr float PID_PITCH_KP = 6.0f;
static constexpr float PID_PITCH_KI = 0.0f;
static constexpr float PID_PITCH_KD = 0.15f;

Pid pid_roll(PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, PID_LIMIT);
Pid pid_pitch(PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, PID_LIMIT);

static float target_roll_deg = 0.0f;
static float target_pitch_deg = 0.0f;

// ================== ШАГОВЫЙ ДВИГАТЕЛЬ ===================
// Экземпляр управления шаговым двигателем (из shag.h)
BalanceStepper balanceStepper;
// ========================================================
// ================================================================

// ================== PID → ШАГОВИКИ ===================
// Управляющий сигнал PID подаётся на ОБА шаговика при отклонении roll от цели.
// u_roll_out = выход PID (roll + feedforward), оба мотора вращаются от него.
static constexpr float CTRL_X_DEADBAND_DEG = 0.5f;  // порог: моторы только если |roll - target| > этого (град)
static constexpr uint8_t CTRL_X_DEADBAND_PWM = 3;   // мёртвая зона (для Serial Plotter)
// ====================================================

static void calibrateGyro() {
  if (!SERIAL_PLOTTER_MODE) {
    Serial.println("CALIBRATING_GYRO: keep device still...");
  }

  float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
  float sum_x2 = 0.0f, sum_y2 = 0.0f, sum_z2 = 0.0f;

  // сбросим буферные/первые "шумные" чтения
  for (int i = 0; i < 100; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    delay(2);
  }

  // Собираем данные с проверкой на выбросы
  uint16_t valid_samples = 0;
  for (uint16_t i = 0; i < CALIB_SAMPLES; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // В Adafruit Unified Sensor гироскоп в rad/s → переводим в deg/s
    const float gx_dps = g.gyro.x * RAD_TO_DEG_F;
    const float gy_dps = g.gyro.y * RAD_TO_DEG_F;
    const float gz_dps = g.gyro.z * RAD_TO_DEG_F;

    // Фильтр выбросов: если уже есть данные, проверяем что новое значение не слишком далеко
    if (valid_samples > 10) {
      const float mean_x = sum_x / valid_samples;
      const float mean_y = sum_y / valid_samples;
      const float mean_z = sum_z / valid_samples;
      
      // Пропускаем выбросы (больше 3 deg/s от среднего)
      if ((fabs(gx_dps - mean_x) > 3.0f) ,
          (fabs(gy_dps - mean_y) > 3.0f) ,
          (fabs(gz_dps - mean_z) > 3.0f)) {
        continue; // пропускаем этот сэмпл
      }
    }

    sum_x += gx_dps;
    sum_y += gy_dps;
    sum_z += gz_dps;
    sum_x2 += gx_dps * gx_dps;
    sum_y2 += gy_dps * gy_dps;
    sum_z2 += gz_dps * gz_dps;
    valid_samples++;
    delay(2);
  }

  if (valid_samples < CALIB_SAMPLES / 2) {
    if (!SERIAL_PLOTTER_MODE) {
      Serial.println("WARNING: Too many outliers during calibration!");
    }
  }

  gyro_bias_x_dps = sum_x / valid_samples;
  gyro_bias_y_dps = sum_y / valid_samples;
  gyro_bias_z_dps = sum_z / valid_samples;

  // Вычисляем стандартное отклонение для проверки качества
  const float mean_x2 = sum_x2 / valid_samples;
  const float mean_y2 = sum_y2 / valid_samples;
  const float mean_z2 = sum_z2 / valid_samples;
  const float std_x = sqrtf(mean_x2 - gyro_bias_x_dps * gyro_bias_x_dps);
  const float std_y = sqrtf(mean_y2 - gyro_bias_y_dps * gyro_bias_y_dps);
  const float std_z = sqrtf(mean_z2 - gyro_bias_z_dps * gyro_bias_z_dps);

  if (!SERIAL_PLOTTER_MODE) {
    Serial.print("GYRO_BIAS_DPS:");
    Serial.print(gyro_bias_x_dps, 6); Serial.print(",");
    Serial.print(gyro_bias_y_dps, 6); Serial.print(",");
    Serial.print(gyro_bias_z_dps, 6);
    Serial.print(" | STD:");
    Serial.print(std_x, 4); Serial.print(",");
    Serial.print(std_y, 4); Serial.print(",");
    Serial.println(std_z, 4);

    if ((std_x > CALIB_MAX_STD_DPS) , (std_y > CALIB_MAX_STD_DPS) , (std_z > CALIB_MAX_STD_DPS)) {
      Serial.println("WARNING: High noise during calibration! Keep device stiller next time.");
    } else {
      Serial.println("Calibration OK!");
    }
  }
}

void setup(void) {
  Serial.begin(4800);
  if (!mpu.begin()) {
    while (1) yield();
  }

  // Максимальная чувствительность для гироскопа
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Инициализация шаговых двигателей (shag.h) — оба управляются от PID
  balanceStepper.begin();

  delay(200);
  calibrateGyro();
  pid_roll.resetPID();
  pid_pitch.resetPID();
  last_us = micros();
}


void loop() {
  // Проверка команды перекалибровки из Serial
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'c' , cmd == 'C') {
      if (!SERIAL_PLOTTER_MODE) {
        Serial.println("RECALIBRATING...");
      }
      calibrateGyro();
      // Сброс углов после перекалибровки
      roll_deg = pitch_deg = yaw_deg = 0.0f;
      stationary_ms = 0;
      pid_roll.resetPID();
      pid_pitch.resetPID();
    }
  }

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // dt в секундах
  const uint32_t now_us = micros();
  uint32_t dt_us = now_us - last_us;
  last_us = now_us;
  if (dt_us > 100000) dt_us = 100000; // clamp 0.1s
  const float dt = dt_us * 1e-6f;

  // Гироскоп в deg/s (сырые)
  const float gx_raw_dps = (g.gyro.x * RAD_TO_DEG_F);
  const float gy_raw_dps = (g.gyro.y * RAD_TO_DEG_F);
  const float gz_raw_dps = (g.gyro.z * RAD_TO_DEG_F);

  // Детектор "не вращаемся" (по гироскопу)
  const bool stationary =
    (fabs(gx_raw_dps) < STATIONARY_THRESH_DPS) &&
    (fabs(gy_raw_dps) < STATIONARY_THRESH_DPS) &&
    (fabs(gz_raw_dps) < STATIONARY_THRESH_DPS);

  if (stationary) {
    stationary_ms += (uint32_t)(dt * 1000.0f);
  } else {
    stationary_ms = 0;
  }

  // Онлайн-подстройка bias только когда долго в покое
  if (stationary_ms >= STATIONARY_MIN_MS) {
    gyro_bias_x_dps = (1.0f - BIAS_LPF_ALPHA) * gyro_bias_x_dps + BIAS_LPF_ALPHA * gx_raw_dps;
    gyro_bias_y_dps = (1.0f - BIAS_LPF_ALPHA) * gyro_bias_y_dps + BIAS_LPF_ALPHA * gy_raw_dps;
    gyro_bias_z_dps = (1.0f - BIAS_LPF_ALPHA) * gyro_bias_z_dps + BIAS_LPF_ALPHA * gz_raw_dps;
  }

  // Гироскоп в deg/s (с учётом bias)
  const float gx_dps = gx_raw_dps - gyro_bias_x_dps;
  const float gy_dps = gy_raw_dps - gyro_bias_y_dps;
  const float gz_dps = gz_raw_dps - gyro_bias_z_dps;

  // ОРИЕНТАЦИЯ ТОЛЬКО ПО ГИРОСКОПУ: угол += скорость * dt
  roll_deg  += gx_dps * dt;
  pitch_deg += gy_dps * dt;
  yaw_deg   += gz_dps * dt;

  // PID по углам: выход в диапазоне [-PID_LIMIT .. +PID_LIMIT]
  const float u_roll = pid_roll.updatePID(target_roll_deg, roll_deg, dt);
  const float u_pitch = pid_pitch.updatePID(target_pitch_deg, pitch_deg, dt);

  // Управляющий сигнал по формуле с фото (модель Free fall):
  // Φ̈ = (m L θ̇² sin(θ) - m g R sin(θ) cos(θ)) / (m R² sin(θ)² + I)
  // Числитель даёт "естественное" ускорение колеса; добавляем его как feedforward.
  const float theta_rad = roll_deg * DEG_TO_RAD_F;
  const float theta_dot_rad_s = gx_dps * DEG_TO_RAD_F;
  const float sin_theta = sinf(theta_rad);
  const float cos_theta = cosf(theta_rad);
  const float numerator_phi = MODEL_m * MODEL_L * theta_dot_rad_s * theta_dot_rad_s * sin_theta
                            - MODEL_m * MODEL_g * MODEL_R * sin_theta * cos_theta;
  const float u_feedforward = MODEL_FF_GAIN * numerator_phi;
  float u_roll_total = u_roll + u_feedforward;
  u_roll_total = constrain(u_roll_total, -PID_LIMIT, PID_LIMIT);

  // УПРАВЛЯЮЩИЙ СИГНАЛ ОТ PID — только при отклонении roll от цели
  const float deviation_x_deg = fabsf(roll_deg - target_roll_deg);
  const float u_roll_out = (deviation_x_deg > CTRL_X_DEADBAND_DEG) ? u_roll_total : 0.0f;

  // Для вывода (Serial Plotter) — pwm как величина 0..255
  const bool dir = (u_roll_out >= 0.0f);
  int pwm = (int)lroundf(fabsf(u_roll_out));
  pwm = constrain(pwm, 0, 255);
  if (pwm <= (int)CTRL_X_DEADBAND_PWM) pwm = 0;
  
  // === ОБА ШАГОВИКА УПРАВЛЯЮТСЯ ОТ PID КОНТРОЛЛЕРА ===
  // u_roll_out — выход PID (roll + feedforward), диапазон [-PID_LIMIT .. +PID_LIMIT]
  // Оба мотора получают один сигнал: вращаются в противоположных направлениях для баланса
  balanceStepper.setFromPID(u_roll_out, PID_LIMIT);
  balanceStepper.update();  // Обязательно вызывать как можно чаще!

  if (OUTPUT_MODE == 4) {
    // Формат для Serial Plotter: roll, target, u_total (PID+формула с фото), pwm
    Serial.print(roll_deg, 6);        Serial.print(",");
    Serial.print(target_roll_deg, 6); Serial.print(",");
    Serial.print(u_roll_total, 6);    Serial.print(",");
    Serial.println(pwm);
  } else if (OUTPUT_MODE == 3) {
    Serial.print(roll_deg, 6);  Serial.print(",");
    Serial.print(pitch_deg, 6); Serial.print(",");
    Serial.print(yaw_deg, 6);   Serial.print(",");
    Serial.print(u_roll, 6);    Serial.print(",");
    Serial.println(u_pitch, 6);
  } else if (OUTPUT_MODE == 2) {
    Serial.print(roll_deg, 6);  Serial.print(",");
    Serial.print(pitch_deg, 6); Serial.print(",");
    Serial.println(yaw_deg, 6);
  } else if (OUTPUT_MODE == 1) {
    Serial.print(gx_dps, 6); Serial.print(",");
    Serial.print(gy_dps, 6); Serial.print(",");
    Serial.println(gz_dps, 6);
  } else {
    // Старый формат: ax,ay,az,gx,gy,gz
    Serial.print(a.acceleration.x, 6); Serial.print(",");
    Serial.print(a.acceleration.y, 6); Serial.print(",");
    Serial.print(a.acceleration.z, 6); Serial.print(",");
    Serial.print(gx_dps, 6);           Serial.print(",");
    Serial.print(gy_dps, 6);           Serial.print(",");
    Serial.println(gz_dps, 6);
  }

  delay(LOOP_DELAY_MS);
}