#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <EEPROM.h>

#include "pidbalanse.h"
#include "shag.h"

#define EEPROM_MAGIC       0xCA1B
#define EEPROM_CALIB_ADDR  0

Adafruit_MPU6050 mpu;

#ifndef SERIAL_BAUD
#define SERIAL_BAUD 115200
#endif

#define CALIB_SAMPLES_PER_POS  600   // сэмплов на каждую плоскость
#define MEMORIZE_MS            2000  // 2 сек держать вертикально для нуля
#define FLAT_SURFACE_MAX_TILT_DEG  5.0f   // макс. допустимый наклон поверхности
#define LOOP_DELAY_MS          4
#define RAD_TO_DEG_F           57.29577951308232f
#define DEG_TO_RAD_F           0.017453292519943295f
#define COMPL_FILTER_ALPHA     0.995f
#define GRAVITY_MS2            9.80665f

// Строки в PROGMEM (flash) — экономим RAM
static void printPosName(int i) {
  switch (i) {
    case 0: Serial.println(F("ВЕРТИКАЛЬНО - колёса вниз")); break;
    case 1: Serial.println(F("НА СПИНЕ - колёса вверх")); break;
    case 2: Serial.println(F("НА ЛЕВОМ БОКУ")); break;
    case 3: Serial.println(F("НА ПРАВОМ БОКУ")); break;
    case 4: Serial.println(F("НОС ВВЕРХ")); break;
    default: Serial.println(F("НОС ВНИЗ")); break;
  }
}
static void printPosAction(int i) {
  switch (i) {
    case 0: Serial.print(F("Поставьте вертикально")); break;
    case 1: Serial.print(F("ПЕРЕВЕРНИТЕ: на спину")); break;
    case 2: Serial.print(F("ПЕРЕВЕРНИТЕ: на левый бок")); break;
    case 3: Serial.print(F("ПЕРЕВЕРНИТЕ: на правый бок")); break;
    case 4: Serial.print(F("ПЕРЕВЕРНИТЕ: нос вверх")); break;
    default: Serial.print(F("ПЕРЕВЕРНИТЕ: нос вниз")); break;
  }
}

// Компактное хранение: *100 для int16_t (экономия RAM)
static int16_t acc_means[6][3];
static int16_t gyro_means[6][3];

// Калибровка (результаты 6-плоскостной калибровки)
float gyro_bias_x_dps = 0, gyro_bias_y_dps = 0, gyro_bias_z_dps = 0;
float accel_offset_x = 0, accel_offset_y = 0, accel_offset_z = 0;
float accel_scale_x = 1.0f, accel_scale_y = 1.0f, accel_scale_z = 1.0f;
float roll_deg = 0, pitch_deg = 0, yaw_deg = 0;
float currentLeanAngle_roll = 0, currentLeanAngle_pitch = 0;
float target_roll_deg = 0, target_pitch_deg = 0;
uint32_t last_us = 0;

// PID (как в основном коде)
#define PID_LIMIT 255.0f
#define PID_KP    7.0f
#define PID_KI    0.0f
#define PID_KD    0.12f
#define USE_PITCH_FOR_BALANCE 0   // 0=roll, 1=pitch

Pid pid_balance(PID_KP, PID_KI, PID_KD, PID_LIMIT);
BalanceStepper balanceStepper;

// Feedforward (упрощённо)
#define MODEL_m 0.3f
#define MODEL_g 9.81f
#define MODEL_R 0.039f
#define MODEL_L 0.2f
#define MODEL_FF_GAIN 0.025f

static bool promptAndCollectPosition(int posIndex) {
  int posNum = posIndex + 1;
  Serial.println();
  Serial.println(F("========================================"));
  Serial.print(F("  ПОЗИЦИЯ "));
  Serial.print(posNum);
  Serial.print(F(" из 6: "));
  printPosName(posIndex);
  Serial.println(F("========================================"));
  Serial.println();
  Serial.print(F(">>> "));
  printPosAction(posIndex);
  Serial.println(F(" <<<"));
  Serial.println();
  Serial.println(F("Обратный отсчёт 5 сек. Успейте перевернуть!"));
  Serial.println();

  for (int t = 5; t >= 1; t--) {
    Serial.print(F("  "));
    Serial.print(t);
    Serial.println(F("..."));
    delay(1000);
  }

  Serial.println(F("  Сбор данных..."));
  Serial.print(F("  "));

  float sum_ax = 0, sum_ay = 0, sum_az = 0;
  float sum_gx = 0, sum_gy = 0, sum_gz = 0;
  uint32_t count = 0;

  for (uint32_t i = 0; i < CALIB_SAMPLES_PER_POS; i++) {
    sensors_event_t a, g, temp;
    if (mpu.getEvent(&a, &g, &temp)) {
      sum_ax += a.acceleration.x;
      sum_ay += a.acceleration.y;
      sum_az += a.acceleration.z;
      sum_gx += g.gyro.x * RAD_TO_DEG_F;
      sum_gy += g.gyro.y * RAD_TO_DEG_F;
      sum_gz += g.gyro.z * RAD_TO_DEG_F;
      count++;
    }
    if (i % 100 == 0 && i > 0) Serial.print('.');
    delay(3);
  }

  float mean_ax = sum_ax / count, mean_ay = sum_ay / count, mean_az = sum_az / count;
  acc_means[posIndex][0] = (int16_t)constrain(mean_ax * 100, -32700, 32700);
  acc_means[posIndex][1] = (int16_t)constrain(mean_ay * 100, -32700, 32700);
  acc_means[posIndex][2] = (int16_t)constrain(mean_az * 100, -32700, 32700);
  gyro_means[posIndex][0] = (int16_t)constrain(sum_gx / count * 100, -32700, 32700);
  gyro_means[posIndex][1] = (int16_t)constrain(sum_gy / count * 100, -32700, 32700);
  gyro_means[posIndex][2] = (int16_t)constrain(sum_gz / count * 100, -32700, 32700);

  if (posIndex == 0) {
    float horiz = sqrtf(mean_ax * mean_ax + mean_ay * mean_ay);
    float vert = fabsf(mean_az);
    float tilt_deg = (vert > 0.1f) ? (atan2f(horiz, vert) * RAD_TO_DEG_F) : 90.0f;
    if (tilt_deg > FLAT_SURFACE_MAX_TILT_DEG) {
      Serial.println();
      Serial.println(F("!!! ПОВЕРХНОСТЬ НЕ РОВНАЯ !!!"));
      Serial.print(F("Наклон: ")); Serial.print(tilt_deg, 1); Serial.println(F(" град."));
      Serial.println(F("Переместите на ровную поверхность. Повторите через 5 сек..."));
      delay(5000);
      return false;
    }
    Serial.print(F("  Поверхность ровная (")); Serial.print(tilt_deg, 1); Serial.println(F(" град.)"));
  }
  Serial.println(F(" OK"));
  return true;
}

static void computeCalibration() {
  float gx = 0, gy = 0, gz = 0;
  for (int i = 0; i < 6; i++) {
    gx += gyro_means[i][0] / 100.0f;
    gy += gyro_means[i][1] / 100.0f;
    gz += gyro_means[i][2] / 100.0f;
  }
  gyro_bias_x_dps = gx / 6.0f;
  gyro_bias_y_dps = gy / 6.0f;
  gyro_bias_z_dps = gz / 6.0f;

  float ax2 = acc_means[2][0] / 100.0f, ax3 = acc_means[3][0] / 100.0f;
  float ay4 = acc_means[4][1] / 100.0f, ay5 = acc_means[5][1] / 100.0f;
  float az0 = acc_means[0][2] / 100.0f, az1 = acc_means[1][2] / 100.0f;

  accel_offset_x = (ax2 + ax3) / 2.0f;
  accel_offset_y = (ay4 + ay5) / 2.0f;
  accel_offset_z = (az0 + az1) / 2.0f;

  float rx = ax2 - ax3, ry = ay4 - ay5, rz = az0 - az1;
  accel_scale_x = (fabsf(rx) > 0.1f) ? (2.0f * GRAVITY_MS2 / rx) : 1.0f;
  accel_scale_y = (fabsf(ry) > 0.1f) ? (2.0f * GRAVITY_MS2 / ry) : 1.0f;
  accel_scale_z = (fabsf(rz) > 0.1f) ? (2.0f * GRAVITY_MS2 / rz) : 1.0f;

  Serial.println();
  Serial.println(F("--- Калибровка завершена ---"));
  Serial.print(F("GYRO: ")); Serial.print(gyro_bias_x_dps, 3); Serial.print(',');
  Serial.print(gyro_bias_y_dps, 3); Serial.print(','); Serial.println(gyro_bias_z_dps, 3);
  Serial.print(F("ACC: ")); Serial.print(accel_offset_x, 2); Serial.print(',');
  Serial.print(accel_offset_y, 2); Serial.print(','); Serial.println(accel_offset_z, 2);
}

static void runSixPlaneCalibration() {
  Serial.println();
  Serial.println(F("########################################"));
  Serial.println(F("#  КАЛИБРОВКА ПО 6 ПЛОСКОСТЯМ       #"));
  Serial.println(F("########################################"));
  Serial.println();
  Serial.println(F("Поставьте робота ВЕРТИКАЛЬНО. Через 5 сек - старт."));
  delay(3000);

  for (int i = 0; i < 6; i++) {
    while (!promptAndCollectPosition(i)) { }
    if (i < 5) {
      Serial.println();
      Serial.println(F(">>> Следующая позиция <<<"));
      delay(2000);
    }
  }
  computeCalibration();
}

// --- EEPROM: сохранение и загрузка калибровки ---
struct CalibData {
  uint16_t magic;
  float gyro_bias_x, gyro_bias_y, gyro_bias_z;
  float accel_off_x, accel_off_y, accel_off_z;
  float accel_scale_x, accel_scale_y, accel_scale_z;
  float target_roll, target_pitch;
};

static void saveCalibrationToEEPROM() {
  CalibData d;
  d.magic = EEPROM_MAGIC;
  d.gyro_bias_x = gyro_bias_x_dps;
  d.gyro_bias_y = gyro_bias_y_dps;
  d.gyro_bias_z = gyro_bias_z_dps;
  d.accel_off_x = accel_offset_x;
  d.accel_off_y = accel_offset_y;
  d.accel_off_z = accel_offset_z;
  d.accel_scale_x = accel_scale_x;
  d.accel_scale_y = accel_scale_y;
  d.accel_scale_z = accel_scale_z;
  d.target_roll = target_roll_deg;
  d.target_pitch = target_pitch_deg;

  EEPROM.put(EEPROM_CALIB_ADDR, d);
  Serial.println(F("Сохранено в EEPROM. После перезагрузки встанет сам."));
}

static bool loadCalibrationFromEEPROM() {
  CalibData d;
  EEPROM.get(EEPROM_CALIB_ADDR, d);
  if (d.magic != EEPROM_MAGIC) return false;

  gyro_bias_x_dps = d.gyro_bias_x;
  gyro_bias_y_dps = d.gyro_bias_y;
  gyro_bias_z_dps = d.gyro_bias_z;
  accel_offset_x = d.accel_off_x;
  accel_offset_y = d.accel_off_y;
  accel_offset_z = d.accel_off_z;
  accel_scale_x = d.accel_scale_x;
  accel_scale_y = d.accel_scale_y;
  accel_scale_z = d.accel_scale_z;
  target_roll_deg = d.target_roll;
  target_pitch_deg = d.target_pitch;
  roll_deg = target_roll_deg;
  pitch_deg = target_pitch_deg;
  currentLeanAngle_roll = target_roll_deg;
  currentLeanAngle_pitch = target_pitch_deg;

  return true;
}

// Калиброванные значения акселерометра
static inline void getCalibratedAccel(float ax, float ay, float az,
    float* out_ax, float* out_ay, float* out_az) {
  *out_ax = (ax - accel_offset_x) * accel_scale_x;
  *out_ay = (ay - accel_offset_y) * accel_scale_y;
  *out_az = (az - accel_offset_z) * accel_scale_z;
}

static void memorizeTarget() {
  Serial.println();
  Serial.println(F("ЗАПОМИНАНИЕ НУЛЯ: держите ВЕРТИКАЛЬНО 2 сек..."));
  roll_deg = pitch_deg = yaw_deg = 0;
  currentLeanAngle_roll = currentLeanAngle_pitch = 0;

  uint32_t start = millis();
  while (millis() - start < MEMORIZE_MS) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    uint32_t now_us = micros();
    float dt = (now_us - last_us) * 1e-6f;
    if (dt > 0.1f) dt = 0.1f;
    last_us = now_us;

    float gx = (g.gyro.x * RAD_TO_DEG_F) - gyro_bias_x_dps;
    float gy = (g.gyro.y * RAD_TO_DEG_F) - gyro_bias_y_dps;
    float gz = (g.gyro.z * RAD_TO_DEG_F) - gyro_bias_z_dps;
    float cax, cay, caz;
    getCalibratedAccel(a.acceleration.x, a.acceleration.y, a.acceleration.z, &cax, &cay, &caz);
    float accR = atan2f(cay, sqrtf(cax*cax + caz*caz)) * RAD_TO_DEG_F;
    float accP = atan2f(-cax, sqrtf(cay*cay + caz*caz)) * RAD_TO_DEG_F;

    roll_deg  = COMPL_FILTER_ALPHA * (roll_deg  + gx * dt) + (1.0f - COMPL_FILTER_ALPHA) * accR;
    pitch_deg = COMPL_FILTER_ALPHA * (pitch_deg + gy * dt) + (1.0f - COMPL_FILTER_ALPHA) * accP;
    yaw_deg += gz * dt;

    delay(2);
  }

  target_roll_deg = roll_deg;
  target_pitch_deg = pitch_deg;
  currentLeanAngle_roll = roll_deg;
  currentLeanAngle_pitch = pitch_deg;

  Serial.print(F("ЦЕЛЬ: roll="));
  Serial.print(target_roll_deg);
  Serial.print(F(" pitch="));
  Serial.println(target_pitch_deg);
  Serial.println(F(">>> Отпустите - PID ловит баланс <<<"));
  Serial.println();

  saveCalibrationToEEPROM();
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  if (!mpu.begin()) {
    Serial.println(F("ОШИБКА: MPU6050 не найден!"));
    while (1) yield();
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  balanceStepper.begin();
  last_us = micros();

  // Пробуем загрузить калибровку из EEPROM
  if (loadCalibrationFromEEPROM()) {
    Serial.println();
    Serial.println(F("########################################"));
    Serial.println(F("#  Калибровка из EEPROM - встаёт!    #"));
    Serial.println(F("########################################"));
    Serial.println();
    pid_balance.resetPID();
    return;
  }

  Serial.println(F("Первая калибровка. Следуйте подсказкам."));
  runSixPlaneCalibration();
  pid_balance.resetPID();
  memorizeTarget();
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'c' || c == 'C') {
      Serial.println(F("Перекалибровка..."));
      runSixPlaneCalibration();
      roll_deg = pitch_deg = yaw_deg = 0;
      currentLeanAngle_roll = currentLeanAngle_pitch = 0;
      pid_balance.resetPID();
      memorizeTarget();
    }
    if (c == 'z' || c == 'Z') {
      target_roll_deg = roll_deg;
      target_pitch_deg = pitch_deg;
      saveCalibrationToEEPROM();
      Serial.print(F("Ноль: roll=")); Serial.print(target_roll_deg);
      Serial.print(F(" pitch=")); Serial.println(target_pitch_deg);
    }
    if (c == 'e' || c == 'E') {
      EEPROM.put(EEPROM_CALIB_ADDR, (uint16_t)0);
      Serial.println(F("EEPROM очищен. Перезагрузка - новая калибровка."));
    }
  }

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  uint32_t now_us = micros();
  float dt = (now_us - last_us) * 1e-6f;
  last_us = now_us;
  if (dt > 0.1f) dt = 0.1f;

  float gx_dps = (g.gyro.x * RAD_TO_DEG_F) - gyro_bias_x_dps;
  float gy_dps = (g.gyro.y * RAD_TO_DEG_F) - gyro_bias_y_dps;
  float gz_dps = (g.gyro.z * RAD_TO_DEG_F) - gyro_bias_z_dps;
  float ax, ay, az;
  getCalibratedAccel(a.acceleration.x, a.acceleration.y, a.acceleration.z, &ax, &ay, &az);
  float accRoll  = atan2f(ay, sqrtf(ax*ax + az*az)) * RAD_TO_DEG_F;
  float accPitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * RAD_TO_DEG_F;

  roll_deg  = COMPL_FILTER_ALPHA * (roll_deg  + gx_dps * dt) + (1.0f - COMPL_FILTER_ALPHA) * accRoll;
  pitch_deg = COMPL_FILTER_ALPHA * (pitch_deg + gy_dps * dt) + (1.0f - COMPL_FILTER_ALPHA) * accPitch;
  yaw_deg += gz_dps * dt;

  float err_roll = fabsf(roll_deg - target_roll_deg);
  float err_pitch = fabsf(pitch_deg - target_pitch_deg);
  float alpha = (err_roll > 5.0f || err_pitch > 5.0f) ? 0.8f : 0.7f;
  currentLeanAngle_roll  = roll_deg  * alpha + currentLeanAngle_roll  * (1.0f - alpha);
  currentLeanAngle_pitch = pitch_deg * alpha + currentLeanAngle_pitch * (1.0f - alpha);

  #if USE_PITCH_FOR_BALANCE
    float angle_deg = currentLeanAngle_pitch;
    float omega_dps = gy_dps;
    float u_pid_raw = pid_balance.updatePID(target_pitch_deg, currentLeanAngle_pitch, dt);
  #else
    float angle_deg = currentLeanAngle_roll;
    float omega_dps = gx_dps;
    float u_pid_raw = pid_balance.updatePID(target_roll_deg, currentLeanAngle_roll, dt);
  #endif

  float theta_rad = angle_deg * DEG_TO_RAD_F;
  float theta_dot = omega_dps * DEG_TO_RAD_F;
  float sin_t = sinf(theta_rad), cos_t = cosf(theta_rad);
  float ff = MODEL_FF_GAIN * (MODEL_m * MODEL_L * theta_dot * theta_dot * sin_t
                              - MODEL_m * MODEL_g * MODEL_R * sin_t * cos_t);
  float u_balance = u_pid_raw + ff;
  u_balance = constrain(u_balance, -PID_LIMIT, PID_LIMIT);

  balanceStepper.setFromPID(u_balance, PID_LIMIT);
  balanceStepper.enableMotors();

  Serial.print(angle_deg, 2);
  Serial.print(',');
  Serial.print(USE_PITCH_FOR_BALANCE ? target_pitch_deg : target_roll_deg, 2);
  Serial.print(',');
  Serial.println(u_balance, 1);

  delay(LOOP_DELAY_MS);
}

ISR(TIMER1_COMPA_vect) {
  TCNT1 = 0;
  if (_directionMotor1 == 0) return;
  PORTD |= (1 << 3);
  delay_05us();
  PORTD &= ~(1 << 3);
}
ISR(TIMER2_COMPA_vect) {
  TCNT2 = 0;
  if (_directionMotor2 == 0) return;
  PORTB |= (1 << 1);
  delay_05us();
  PORTB &= ~(1 << 1);
}
