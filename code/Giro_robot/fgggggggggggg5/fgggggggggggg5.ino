#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "pidbalanse.h"
#include "balancer.h"
#include "shag.h"  // Модуль управления шаговым двигателем
#include "pid_autotune.h"

Adafruit_MPU6050 mpu;

#ifndef SERIAL_BAUD
#define SERIAL_BAUD 115200
#endif

// ================= НАСТРОЙКИ ВЫВОДА =================
// 0: старый формат (ax,ay,az,gx,gy,gz) — но gx/gy/gz в deg/s
// 1: только гироскоп (gx,gy,gz) в deg/s
// 2: углы (roll,pitch,yaw) ТОЛЬКО по гироскопу, в градусах
// 3: углы (roll,pitch,yaw) + выход PID (u_roll,u_pitch)
// 4: Serial Plotter: roll,target_roll,u_roll,pwm (ТОЛЬКО числа)
static constexpr int OUTPUT_MODE = 4;
static constexpr bool SERIAL_PLOTTER_MODE = (OUTPUT_MODE == 4);

static constexpr uint16_t CALIB_SAMPLES = 1500; // калибровка нуля гироскопа (больше = точнее)
static constexpr uint16_t LOOP_DELAY_MS = 4;   // 4ms ≈ 250Hz
static constexpr float CALIB_MAX_STD_DPS = 0.5f; // максимальное стандартное отклонение для "хорошей" калибровки

// В Arduino уже есть макрос RAD_TO_DEG, поэтому своё имя не используем.
static constexpr float RAD_TO_DEG_F = 57.29577951308232f;
static constexpr float DEG_TO_RAD_F = 0.017453292519943295f;

// --- Параметры модели с фото (Free fall: масса m на колесе M, радиус R, стержень L) ---
// Момент инерции колеса I = (1/2)*M*R² (диск)
static constexpr float MODEL_M = 0.5f;   // масса колеса (кг)
static constexpr float MODEL_m = 0.3f;   // масса верхней части (кг)
static constexpr float MODEL_R = 0.039f;  // радиус колеса (м)
static constexpr float MODEL_L = 0.2f;   // длина стержня (м)
static constexpr float MODEL_g = 9.81f;  // g (м/с²)
static constexpr float MODEL_I = 0.5f * MODEL_M * MODEL_R * MODEL_R;  // момент инерции колеса
// Коэффициент масштаба формулы с фото к диапазону PWM
static constexpr float MODEL_FF_GAIN = 0.025f;

// balansing_robot: калибровка гироскопа ТОЛЬКО при старте — без онлайн-подстройки!
// Онлайн-подстройка bias портила углы при балансе (робот "стоял" — гиро ≈0 — bias менялся)

float gyro_bias_x_dps = 0.0f;
float gyro_bias_y_dps = 0.0f;
float gyro_bias_z_dps = 0.0f;

float roll_deg = 0.0f;
float pitch_deg = 0.0f;
float yaw_deg = 0.0f;
float currentLeanAngle_roll = 0.0f;   // сглаженный угол для PID (как в balansing_robot)
float currentLeanAngle_pitch = 0.0f;

// Комплементарный фильтр (acc+gyro) — как в balansing_robot
static constexpr float COMPL_FILTER_ALPHA = 0.995f;  // 0.995 = 99.5% гироскоп, 0.5% акселерометр

uint32_t last_us = 0;
// =====================================================

// ================= PID ДЛЯ БАЛАНСИРОВКИ (настройка с нуля) =================
// НАСТРОЙКА ПО ШАГАМ:
// 1) Держи робота в руках. Kp=0 → моторы не реагируют. Постепенно увеличивай Kp (например +1),
//    пока при наклоне моторы чётко пытаются вернуть в ноль. Если уже при малом Kp сильная тряска — уменьши.
// 2) Добавь Kd (0.05..0.3): уменьшает колебания и «звон». Слишком большой Kd — вялая реакция.
// 3) Ki оставь 0, пока не нужна компенсация постоянного смещения (часто для баланса не нужен).
static constexpr float PID_LIMIT = 255.0f;
// Kp, Kd — как было. Ki подбираем понемногу: убирает постоянный уход и цикл "спокойно → колебания"
static constexpr float PID_ROLL_KP = 7.0f;
static constexpr float PID_ROLL_KI = 0.0f;   // начальное значение; увеличивай по 0.01 при необходимости
static constexpr float PID_ROLL_KD = 0.12f;
static constexpr float PID_PITCH_KP = 7.0f;
static constexpr float PID_PITCH_KI = 0.0f;
static constexpr float PID_PITCH_KD = 0.12f;

Pid pid_roll(PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, PID_LIMIT);
Pid pid_pitch(PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, PID_LIMIT);

static float target_roll_deg = 0.0f;
static float target_pitch_deg = 0.0f;

// Запомнить начальное положение через 1 сек после включения
#ifndef MEMORIZE_POSITION_MS
#define MEMORIZE_POSITION_MS 1000
#endif
static bool balance_target_memorized = false;

PidAutotune pidAutotune;
static bool autotune_result_printed = false;

// ================== ШАГОВЫЙ ДВИГАТЕЛЬ ===================
// Экземпляр управления шаговым двигателем (из shag.h)
BalanceStepper balanceStepper;
// ========================================================
// ================================================================

// ================== БАЛАНСИРОВКА ===================
// 0 = баланс по roll (наклон влево-вправо), 1 = по pitch (вперёд-назад)
#ifndef USE_PITCH_FOR_BALANCE
#define USE_PITCH_FOR_BALANCE 0
#endif
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
      if (fabs(gx_dps - mean_x) > 3.0f ||
          fabs(gy_dps - mean_y) > 3.0f ||
          fabs(gz_dps - mean_z) > 3.0f) {
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

    if (std_x > CALIB_MAX_STD_DPS || std_y > CALIB_MAX_STD_DPS || std_z > CALIB_MAX_STD_DPS) {
      Serial.println("WARNING: High noise during calibration! Keep device stiller next time.");
    } else {
      Serial.println("Calibration OK!");
    }
  }
}

void setup(void) {
  Serial.begin(SERIAL_BAUD);
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

  // Фаза запоминания: 1 сек — держи робота в нужном положении, оно станет "нулём"
  if (!SERIAL_PLOTTER_MODE) {
    Serial.println("HOLD_POSITION: 1 sec...");
  }
  uint32_t memorize_start = millis();
  while (millis() - memorize_start < MEMORIZE_POSITION_MS) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    uint32_t now_us = micros();
    uint32_t dt_us = now_us - last_us;
    last_us = now_us;
    if (dt_us > 100000) dt_us = 100000;
    float dt = dt_us * 1e-6f;

    const float gx = (g.gyro.x * RAD_TO_DEG_F) - gyro_bias_x_dps;
    const float gy = (g.gyro.y * RAD_TO_DEG_F) - gyro_bias_y_dps;
    const float gz = (g.gyro.z * RAD_TO_DEG_F) - gyro_bias_z_dps;
    const float ax = a.acceleration.x, ay = a.acceleration.y, az = a.acceleration.z;
    const float accR = atan2f(ay, sqrtf(ax*ax + az*az)) * RAD_TO_DEG_F;
    const float accP = atan2f(-ax, sqrtf(ay*ay + az*az)) * RAD_TO_DEG_F;

    roll_deg  = COMPL_FILTER_ALPHA * (roll_deg  + gx * dt) + (1.0f - COMPL_FILTER_ALPHA) * accR;
    pitch_deg = COMPL_FILTER_ALPHA * (pitch_deg + gy * dt) + (1.0f - COMPL_FILTER_ALPHA) * accP;
    yaw_deg   += gz * dt;

    delay(2);
  }
  target_roll_deg = roll_deg;
  target_pitch_deg = pitch_deg;
  currentLeanAngle_roll = roll_deg;
  currentLeanAngle_pitch = pitch_deg;
  balance_target_memorized = true;
  if (!SERIAL_PLOTTER_MODE) {
    Serial.print("TARGET_SAVED: roll=");
    Serial.print(target_roll_deg);
    Serial.print(" pitch=");
    Serial.println(target_pitch_deg);
  }
}


void loop() {
  // Проверка команды перекалибровки из Serial
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'c' || cmd == 'C') {
      if (!SERIAL_PLOTTER_MODE) {
        Serial.println("RECALIBRATING...");
      }
      calibrateGyro();
      roll_deg = pitch_deg = yaw_deg = 0.0f;
      currentLeanAngle_roll = 0.0f;
      currentLeanAngle_pitch = 0.0f;
      pid_roll.resetPID();
      pid_pitch.resetPID();
    }
    if (cmd == 'z' || cmd == 'Z') {
      // Запомнить текущее положение как новую цель (перезаписать target)
      target_roll_deg = roll_deg;
      target_pitch_deg = pitch_deg;
      if (!SERIAL_PLOTTER_MODE) {
        Serial.print("NEW_TARGET: roll=");
        Serial.print(target_roll_deg);
        Serial.print(" pitch=");
        Serial.println(target_pitch_deg);
      }
    }
    if (cmd == 'T' || cmd == 't') {
      if (!balance_target_memorized) {
        Serial.println("AUTOTUNE: wait for target (1s after start), then send T");
      } else if (pidAutotune.isRunning()) {
        Serial.println("AUTOTUNE: already running, wait ~4s");
      } else {
        const float tgt = (USE_PITCH_FOR_BALANCE ? target_pitch_deg : target_roll_deg);
        const float kp = USE_PITCH_FOR_BALANCE ? pid_pitch.getP() : pid_roll.getP();
        const float ki = USE_PITCH_FOR_BALANCE ? pid_pitch.getI() : pid_roll.getI();
        const float kd = USE_PITCH_FOR_BALANCE ? pid_pitch.getD() : pid_roll.getD();
        pidAutotune.start(tgt, (bool)USE_PITCH_FOR_BALANCE, kp, ki, kd);
        Serial.println("AUTOTUNE_START: hold robot steady ~4s");
      }
    }
    if (cmd == 'A' || cmd == 'a') {
      if (pidAutotune.isDone()) {
        const float kp = pidAutotune.getSuggestedKp();
        const float ki = pidAutotune.getSuggestedKi();
        const float kd = pidAutotune.getSuggestedKd();
        if (USE_PITCH_FOR_BALANCE) {
          pid_pitch.setP(kp);
          pid_pitch.setI(ki);
          pid_pitch.setD(kd);
        } else {
          pid_roll.setP(kp);
          pid_roll.setI(ki);
          pid_roll.setD(kd);
        }
        Serial.print("APPLIED: Kp="); Serial.print(kp);
        Serial.print(" Ki="); Serial.print(ki);
        Serial.print(" Kd="); Serial.println(kd);
        pidAutotune.resetState();
        autotune_result_printed = false;
      }
    }
  }
  if (pidAutotune.isRunning())
    autotune_result_printed = false;

  if (pidAutotune.isRunning()) {
    if (USE_PITCH_FOR_BALANCE)
      target_pitch_deg = pidAutotune.getCurrentTarget();
    else
      target_roll_deg = pidAutotune.getCurrentTarget();
  }

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // dt в секундах
  const uint32_t now_us = micros();
  uint32_t dt_us = now_us - last_us;
  last_us = now_us;
  if (dt_us > 100000) dt_us = 100000; // clamp 0.1s
  const float dt = dt_us * 1e-6f;

  // Гироскоп в deg/s — bias фиксирован при старте (как balansing_robot, БЕЗ онлайн-подстройки)
  const float gx_dps = (g.gyro.x * RAD_TO_DEG_F) - gyro_bias_x_dps;
  const float gy_dps = (g.gyro.y * RAD_TO_DEG_F) - gyro_bias_y_dps;
  const float gz_dps = (g.gyro.z * RAD_TO_DEG_F) - gyro_bias_z_dps;

  // Углы из акселерометра (градусы) — как в balansing_robot
  const float ax = a.acceleration.x;
  const float ay = a.acceleration.y;
  const float az = a.acceleration.z;
  const float accRoll_deg  = atan2f(ay, sqrtf(ax*ax + az*az)) * RAD_TO_DEG_F;
  const float accPitch_deg = atan2f(-ax, sqrtf(ay*ay + az*az)) * RAD_TO_DEG_F;

  // Комплементарный фильтр: angle = 0.995*(angle + rate*dt) + 0.005*accAngle (balansing_robot)
  roll_deg  = COMPL_FILTER_ALPHA * (roll_deg  + gx_dps * dt) + (1.0f - COMPL_FILTER_ALPHA) * accRoll_deg;
  pitch_deg = COMPL_FILTER_ALPHA * (pitch_deg + gy_dps * dt) + (1.0f - COMPL_FILTER_ALPHA) * accPitch_deg;
  yaw_deg += gz_dps * dt;

  // Сглаживание: при большом отклонении — быстрее (0.8), при малом — плавнее (0.7)
  const float err_roll  = fabsf(roll_deg  - target_roll_deg);
  const float err_pitch = fabsf(pitch_deg - target_pitch_deg);
  const float alpha_smooth = (err_roll > 5.0f || err_pitch > 5.0f) ? 0.8f : 0.7f;
  currentLeanAngle_roll  = roll_deg  * alpha_smooth + currentLeanAngle_roll  * (1.0f - alpha_smooth);
  currentLeanAngle_pitch = pitch_deg * alpha_smooth + currentLeanAngle_pitch * (1.0f - alpha_smooth);

  if (pidAutotune.isRunning()) {
    const float angle = USE_PITCH_FOR_BALANCE ? currentLeanAngle_pitch : currentLeanAngle_roll;
    pidAutotune.update(angle, dt, millis());
  }
  if (pidAutotune.isDone()) {
    if (!autotune_result_printed) {
      Serial.print("AUTOTUNE_DONE overshoot=");
      Serial.print(pidAutotune.getMaxOvershoot());
      Serial.print(" settle_ms=");
      Serial.println(pidAutotune.getSettleMs());
      Serial.print("SUGGESTED: Kp=");
      Serial.print(pidAutotune.getSuggestedKp());
      Serial.print(" Ki=");
      Serial.print(pidAutotune.getSuggestedKi());
      Serial.print(" Kd=");
      Serial.println(pidAutotune.getSuggestedKd());
      Serial.println("Send A to apply.");
    }
    autotune_result_printed = true;
  }

  // ========== ЦЕПОЧКА БАЛАНСИРОВКИ: IMU → сглаженные углы → PID → моторы ==========
  // PID по сглаженным углам (currentLeanAngle) — как в balansing_robot
  const float u_roll = pid_roll.updatePID(target_roll_deg, currentLeanAngle_roll, dt);
  const float u_pitch = pid_pitch.updatePID(target_pitch_deg, currentLeanAngle_pitch, dt);

  #if USE_PITCH_FOR_BALANCE
    const float angle_deg = currentLeanAngle_pitch;
    const float omega_dps = gy_dps;
    const float u_pid_raw = u_pitch;
  #else
    const float angle_deg = currentLeanAngle_roll;
    const float omega_dps = gx_dps;
    const float u_pid_raw = u_roll;
  #endif

  // Feedforward по модели Free fall
  const float theta_rad = angle_deg * DEG_TO_RAD_F;
  const float theta_dot_rad_s = omega_dps * DEG_TO_RAD_F;
  const float sin_theta = sinf(theta_rad);
  const float cos_theta = cosf(theta_rad);
  const float numerator_phi = MODEL_m * MODEL_L * theta_dot_rad_s * theta_dot_rad_s * sin_theta
                            - MODEL_m * MODEL_g * MODEL_R * sin_theta * cos_theta;
  const float u_feedforward = MODEL_FF_GAIN * numerator_phi;

  // Итоговый управляющий сигнал: PID + feedforward
  // Лёгкий буст при ударе о препятствие (|error| > 6°) — больше газа для ловли
  float u_balance = u_pid_raw + u_feedforward;
  const float angle_err = angle_deg - (USE_PITCH_FOR_BALANCE ? target_pitch_deg : target_roll_deg);
  if (fabsf(angle_err) > 6.0f) u_balance *= 1.12f;
  u_balance = constrain(u_balance, -PID_LIMIT, PID_LIMIT);

  // Подаём на моторы: таймеры (Timer1, Timer2) генерируют STEP в фоне — как balansing_robot
  balanceStepper.setFromPID(u_balance, PID_LIMIT);
  balanceStepper.enableMotors();

  // Для Serial Plotter
  const float u_roll_total = u_balance;  // для совместимости вывода
  const int pwm = (int)lroundf(fabsf(u_balance));

  if (OUTPUT_MODE == 4) {
    #if USE_PITCH_FOR_BALANCE
      Serial.print(pitch_deg, 6);         Serial.print(",");
      Serial.print(target_pitch_deg, 6);  Serial.print(",");
    #else
      Serial.print(roll_deg, 6);          Serial.print(",");
      Serial.print(target_roll_deg, 6);   Serial.print(",");
    #endif
    Serial.print(u_roll_total, 6);        Serial.print(",");
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

// ========== ISR для шаговиков (из balansing_robot) ==========
// Timer1 — мотор 1, STEP на D3 (PD3)
ISR(TIMER1_COMPA_vect) {
  TCNT1 = 0;
  if (_directionMotor1 == 0) return;
  PORTD |= (1 << 3);   // STEP pin 3 = PD3
  delay_05us();
  PORTD &= ~(1 << 3);
}

// Timer2 — мотор 2, STEP на D9 (PB1)
ISR(TIMER2_COMPA_vect) {
  TCNT2 = 0;
  if (_directionMotor2 == 0) return;
  PORTB |= (1 << 1);   // STEP pin 9 = PB1
  delay_05us();
  PORTB &= ~(1 << 1);
}