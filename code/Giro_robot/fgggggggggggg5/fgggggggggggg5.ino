/*
  Giro-Robot v2 — архитектура balansing_robot (VL-Systems)
  Интеграция: каскад PID + комплементарный фильтр.
  IMU: Adafruit MPU6050 (без изменений).
  Платформа: Arduino Nano (Timer2 вместо Timer3, оба на 2 MHz).
  Колёса: 3D-печать + шарики для сцепления.
*/
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "pidbalanse.h"
#include "shag.h"

Adafruit_MPU6050 mpu;

#define SERIAL_BAUD 115200

// ========== КОНФИГ (как balansing_robot config.h) ==========
// ПИД скорости (targetAngle → motorSpeed). Смягчено: меньше перерегулирование после ~2 сек
static constexpr float Kp_a = 280.0f;   // было 350 — слишком жёстко
static constexpr float Ki_a = 0.005f;   // было 0.009 — меньше накопление интеграла
static constexpr float Kd_a = 0.0045f;
// Timer2 (8-bit) макс 7800 шагов/с — ограничиваем чтобы оба мотора равны
static constexpr float limit_a = 7500.0f;

// ПИД угла (targetSpeed → targetAngle)
static constexpr float Kp_s = 0.0006f;
static constexpr float Ki_s = 0.00002f;
static constexpr float Kd_s = 0.00001f;
static constexpr float limit_s = 5.0f;

// Цикл ~100 Гц (10 мс)
static constexpr float TARGET_DELTA_TIME = 0.01f;

// Смещение нуля (подбирать под механику, 3D-колёса)
static float zero = 0.0f;

// Целевой угол = запомненное положение при старте
static float targetAngleOffset = 0.0f;

// 1 = pitch (вперёд-назад), 0 = roll (влево-вправо). У тебя баланс по roll!
#define USE_PITCH_AXIS 0

// Фаза запоминания: ПОСТАВЬ РОБОТА НА ЗЕМЛЮ, затем ждём (мс)
#define MEMORIZE_MS 2500

// 0 = оба мотора в одну сторону движения робота (shag.h уже инвертирует DIR мотор 2)
// 1 = если робот крутится на месте вместо движения вперёд-назад
#define MOTOR2_INVERT 0

// Если при наклоне робот ускоряет падение — поставь -1
#define BALANCE_SIGN 1

// Мёртвая зона (град): при |ошибка| < этого — моторы стоп (не мчится вперёд)
#define DEADBAND_DEG 1.5f

// Обнаружение падения: если угол отклонения от вертикали > этого — робот упал, стоп
#define FALL_ANGLE_DEG 45.0f

// Масштаб скорости: мотор 1 (D3), мотор 2 (D9). Оба 1.0 = одинаково.
// Если один крутится медленнее — увеличь его (1.2, 1.3, 1.5) чтобы сравнять.
#define MOTOR1_SCALE 1.0f
#define MOTOR2_SCALE 1.0f

// ========== ПЕРЕМЕННЫЕ (как balansing_robot) ==========
float targetAngle = 0.0f;
float currentLeanAngle = 0.0f;
int targetSpeed = 0;   // 0 = стоять на месте
float motorSpeed = 0.0f;
float motorSpeedPrev = 0.0f;  // для slew rate
float angle = 0.0f;    // выход комплементарного фильтра

// IMU: bias гироскопа (калибровка при старте)
float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;

// Обнаружение падения: true = робот упал, моторы остановлены
static bool isFallen = false;

// ПИД-регуляторы (каскад)
Pid pid_s(Kp_s, Ki_s, Kd_s, limit_s);
Pid pid_a(Kp_a, Ki_a, Kd_a, limit_a);

BalanceStepper balanceStepper;

// ========== КОМПЛЕМЕНТАРНЫЙ ФИЛЬТР (0.995 гиро + 0.005 аксель) ==========
static float filterAngle = 0.0f;
float complementaryFilter(float accAngle_deg, float gyroRate_dps, float dt) {
  filterAngle = 0.995f * (filterAngle + gyroRate_dps * dt) + 0.005f * accAngle_deg;
  return filterAngle;
}

// ========== КАЛИБРОВКА ГИРОСКОПА ==========
void calibrateGyro() {
  Serial.println("CALIBRATING: hold still 3 sec...");
  float sx = 0, sy = 0, sz = 0;
  const int n = 500;
  for (int i = 0; i < n; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sx += g.gyro.x * 57.2958f;
    sy += g.gyro.y * 57.2958f;
    sz += g.gyro.z * 57.2958f;
    delay(2);
  }
  gyro_bias_x = sx / n;
  gyro_bias_y = sy / n;
  gyro_bias_z = sz / n;
  filterAngle = 0.0f;
  Serial.println("Calibration OK");
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  Wire.setClock(400000);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1) yield();
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Шаговики (shag.h): Timer1 + Timer2 оба 2 MHz
  balanceStepper.begin();

  delay(200);
  calibrateGyro();
  pid_s.resetPID();
  pid_a.resetPID();

  // Фаза запоминания: ПОСТАВЬ РОБОТА НА ЗЕМЛЮ вертикально, не трогай 2.5 сек!
  Serial.println("PUT ROBOT ON GROUND now! Stay still 2.5 sec...");
  uint32_t t0 = millis();
  float sumAngle = 0.0f;
  int cnt = 0;
  while (millis() - t0 < MEMORIZE_MS) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    const float ax = a.acceleration.x / 9.81f;
    const float ay = a.acceleration.y / 9.81f;
    const float az = a.acceleration.z / 9.81f;
    const float gy = g.gyro.y * 57.2958f - gyro_bias_y;
    const float gx = g.gyro.x * 57.2958f - gyro_bias_x;
    #if USE_PITCH_AXIS
    float accA = atan2f(-ax, sqrtf(ay*ay + az*az + 0.001f)) * 57.2958f;
    filterAngle = complementaryFilter(accA, gy, 0.01f);
    #else
    float accA = atan2f(ay, sqrtf(ax*ax + az*az + 0.001f)) * 57.2958f;
    filterAngle = complementaryFilter(accA, gx, 0.01f);
    #endif
    sumAngle += filterAngle;
    cnt++;
    delay(10);
  }
  targetAngleOffset = (cnt > 0) ? (sumAngle / cnt) : 0.0f;
  zero = targetAngleOffset;
  currentLeanAngle = targetAngleOffset;
  filterAngle = targetAngleOffset;
  Serial.print("TARGET_SAVED: ");
  Serial.println(targetAngleOffset);

  Serial.println("READY. Put robot on ground.");
}

void loop() {
  // 1. Чтение IMU (Adafruit)
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // 2. Формат как balansing_robot: аксель в G, гиро в deg/s
  const float ax = a.acceleration.x / 9.81f;
  const float ay = a.acceleration.y / 9.81f;
  const float az = a.acceleration.z / 9.81f;
  const float gx = g.gyro.x * 57.2958f - gyro_bias_x;
  const float gy = g.gyro.y * 57.2958f - gyro_bias_y;
  const float gz = g.gyro.z * 57.2958f - gyro_bias_z;

  // 3. Угол из акселерометра (roll = влево-вправо, pitch = вперёд-назад)
  #if USE_PITCH_AXIS
  float accAngle = atan2f(-ax, sqrtf(ay*ay + az*az + 0.001f)) * 57.2958f;
  float gyroRate = gy;  // pitch: вращение вокруг Y
  #else
  float accAngle = atan2f(ay, sqrtf(ax*ax + az*az + 0.001f)) * 57.2958f;
  float gyroRate = gx;  // roll: вращение вокруг X
  #endif

  // 4. Комплиментарный фильтр
  angle = complementaryFilter(accAngle, gyroRate, TARGET_DELTA_TIME);
  currentLeanAngle = angle * 0.7f + currentLeanAngle * 0.3f;

  // 4.5. Обнаружение падения: угол отклонения от целевого > порога
  const float angleError = fabsf(currentLeanAngle - targetAngleOffset);
  if (angleError > FALL_ANGLE_DEG) {
    if (!isFallen) {
      isFallen = true;
      pid_s.resetPID();
      pid_a.resetPID();
      motorSpeed = 0.0f;
      motorSpeedPrev = 0.0f;
    }
  }
  // Восстановление: если угол вернулся в норму (< 20°) — можно снова балансировать
  if (isFallen && angleError < 20.0f) {
    isFallen = false;
    pid_s.resetPID();
    pid_a.resetPID();
    motorSpeed = 0.0f;
    motorSpeedPrev = 0.0f;
  }

  // 5. Каскад ПИД (пропускаем при падении — не накапливаем интеграл)
  if (!isFallen) {
    targetAngle = pid_s.updatePID((float)targetSpeed, motorSpeed, TARGET_DELTA_TIME) + targetAngleOffset;
    motorSpeed = BALANCE_SIGN * (-pid_a.updatePID(targetAngle, currentLeanAngle, TARGET_DELTA_TIME));

    // Мёртвая зона: при малой ошибке — стоп
    const float err = currentLeanAngle - targetAngle;
    if (fabsf(err) < DEADBAND_DEG * 0.5f) {
      motorSpeed = 0.0f;
    } else if (fabsf(err) < DEADBAND_DEG) {
      motorSpeed *= 0.4f;
    }

    // Slew rate: плавное изменение — меньше дёрганий
    const float slewMax = 1100.0f;  // было 800 — плавнее реакция, меньше дёрганий
    float delta = motorSpeed - motorSpeedPrev;
    if (fabsf(delta) > slewMax) {
      delta = (delta > 0) ? slewMax : -slewMax;
      motorSpeed = motorSpeedPrev + delta;
    }
    motorSpeedPrev = motorSpeed;
  } else {
    motorSpeed = 0.0f;
  }

  // 6. Управление моторами (или стоп при падении)
  if (isFallen) {
    balanceStepper.stop();
    // Не включаем моторы — робот упал
  } else {
    int16_t spd = (int16_t)(motorSpeed * MOTOR1_SCALE);
    int16_t spd2 = (int16_t)(motorSpeed * MOTOR2_SCALE);
    balanceStepper.setMotorSpeed(spd, 1);
    balanceStepper.setMotorSpeed(MOTOR2_INVERT ? -spd2 : spd2, 2);
    balanceStepper.enableMotors();
  }

  // Отладка (закомментируй для быстрого цикла)
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    Serial.print("angle:");
    Serial.print(currentLeanAngle);
    Serial.print(" target:");
    Serial.print(targetAngle);
    Serial.print(" motor:");
    Serial.print(motorSpeed);
    if (isFallen) Serial.print(" FALLEN!");
    Serial.println();
  }

  delay(10);  // ~100 Гц
}

// ========== ISR для шаговиков (Timer1, Timer2) ==========
ISR(TIMER1_COMPA_vect) {
  TCNT1 = 0;
  if (_directionMotor1 == 0) return;
  PORTD |= (1 << 3);   // STEP D3
  delay_05us();
  PORTD &= ~(1 << 3);
}
ISR(TIMER2_COMPA_vect) {
  TCNT2 = 0;
  if (_directionMotor2 == 0) return;
  PORTB |= (1 << 1);   // STEP D9
  delay_05us();
  PORTB &= ~(1 << 1);
}
