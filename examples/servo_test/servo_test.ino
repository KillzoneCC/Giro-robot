/*
  Тестовый скетч для проверки шаговых двигателей Giro-Robot.
  Запускайте этот скетч для проверки подключения моторов БЕЗ баланса.
  Управление: '+' — ускорить, '-' — замедлить (через Serial Monitor, 115200 бод).
  Вывод: скорость колёс в шаг/с и м/с (совпадает с Giro_robot/speed_motor.h).
*/
#include <Wire.h>
#include <SoftwareSerial.h>
#include <TMC2208Stepper.h>

// === НАСТРОЙКИ ПОЛЬЗОВАТЕЛЯ ===
#define TOTAL_STEPS    4240  // Максимальное количество шагов

// === СКОРОСТЬ КОЛЁС (как в speed_motor.h) ===
#define STEPS_PER_REV  3200  // 200*16 микрошагов (TMC2208 microsteps(16))
#define WHEEL_DIAMETER_M  0.078f  // 78 мм
#define PI_F  3.14159265358979f

inline float stepsPerSecToMps(float stepsPerSec) {
  if (STEPS_PER_REV <= 0 || WHEEL_DIAMETER_M <= 0.0f) return 0.0f;
  float revPerSec = stepsPerSec / (float)STEPS_PER_REV;
  return revPerSec * (PI_F * WHEEL_DIAMETER_M);
}

// === ПИНЫ ===
#define DIR1_PIN   5
#define STEP1_PIN  3
#define DIR2_PIN   10
#define STEP2_PIN  9

// === UART ===
#define UART1_RX   7
#define UART1_TX   6
#define UART2_RX   4
#define UART2_TX   2

// === ТЕХНИЧЕСКИЕ КОНСТАНТЫ ===
#define PULSE_US       10    // Длительность импульса шага
#define DIR_SETUP_US   5     // Пауза смены направления
#define STEP_GAP_US    10    // Микро-пауза между мотором 1 и 2
#define R_SENSE        0.11f // Для TMC22xx

// === ПЕРЕМЕННЫЕ ДЛЯ СКОРОСТИ ===
int stepDelay = 800;         // Начальная скорость (задержка в мкс)
const int minDelay = 100;    // Максимальная скорость (минимальная задержка)
const int maxDelay = 4000;   // Минимальная скорость (максимальная задержка)
const int speedStep = 50;    // Шаг изменения скорости при нажатии

SoftwareSerial TMCuart1(UART1_RX, UART1_TX);
SoftwareSerial TMCuart2(UART2_RX, UART2_TX);
TMC2208Stepper driver1(&TMCuart1, false);
TMC2208Stepper driver2(&TMCuart2, false);

void setup() {
  Serial.begin(115200);
  Serial.println("System Start. Use '+' to speed up, '-' to slow down.");

  pinMode(DIR1_PIN, OUTPUT);
  pinMode(STEP1_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(STEP2_PIN, OUTPUT);
  digitalWrite(STEP1_PIN, LOW);
  digitalWrite(STEP2_PIN, LOW);

  // --- Инициализация драйверов ---
  TMCuart1.begin(115200);
  delay(100);
  driver1.pdn_disable(true);
  driver1.mstep_reg_select(true);
  driver1.microsteps(16);
  driver1.rms_current(1200, 0.5, R_SENSE);
  driver1.push();
  delay(50);

  TMCuart2.begin(115200);
  delay(100);
  driver2.pdn_disable(true);
  driver2.mstep_reg_select(true);
  driver2.microsteps(16);
  driver2.rms_current(1200, 0.5, R_SENSE);
  driver2.push();
  delay(50);

  digitalWrite(DIR1_PIN, HIGH);
  digitalWrite(DIR2_PIN, HIGH);
  delay(100);

  Serial.println("Speed: steps/s | m/s (press + or - to change)");
}

// Период одного цикла шага (мкс) → шаги/с на один мотор
inline float cycleUsToStepsPerSec(long cycleUs) {
  if (cycleUs <= 0) return 0.0f;
  return 1000000.0f / (float)cycleUs;
}

inline void stepMotor1(bool forward) {
  digitalWrite(DIR1_PIN, forward ? HIGH : LOW);
  delayMicroseconds(DIR_SETUP_US);
  digitalWrite(STEP1_PIN, HIGH);
  delayMicroseconds(PULSE_US);
  digitalWrite(STEP1_PIN, LOW);
}

inline void stepMotor2(bool forward) {
  digitalWrite(DIR2_PIN, forward ? HIGH : LOW);
  delayMicroseconds(DIR_SETUP_US);
  digitalWrite(STEP2_PIN, HIGH);
  delayMicroseconds(PULSE_US);
  digitalWrite(STEP2_PIN, LOW);
}

void stepBothOpposite(bool mainDir) {
  stepMotor1(mainDir);
  delayMicroseconds(STEP_GAP_US);
  stepMotor2(!mainDir);
}

void checkInput() {
  if (Serial.available() > 0) {
    char key = Serial.read();
    if (key == '\n' || key == '\r') return;

    if (key == '+' || key == '=') {
      stepDelay -= speedStep;
      if (stepDelay < minDelay) stepDelay = minDelay;
      Serial.print("FASTER! Delay: ");
      Serial.println(stepDelay);
    } else if (key == '-' || key == '_') {
      stepDelay += speedStep;
      if (stepDelay > maxDelay) stepDelay = maxDelay;
      Serial.print("SLOWER. Delay: ");
      Serial.println(stepDelay);
    }
  }
}

void loop() {
  static uint32_t lastPrint = 0;
  uint32_t t0 = micros();

  for (unsigned long i = 0; i < TOTAL_STEPS; i++) {
    checkInput();
    stepBothOpposite(true);
    delayMicroseconds(stepDelay);
  }

  uint32_t elapsed = micros() - t0;
  // Один цикл = 1 шаг каждого мотора, период = elapsed/TOTAL_STEPS мкс
  long cycleUs = (long)(elapsed / TOTAL_STEPS);
  float sps = cycleUsToStepsPerSec(cycleUs);
  float mps = stepsPerSecToMps(sps);

  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    Serial.print("steps/s: ");
    Serial.print(sps, 0);
    Serial.print("  |  m/s: ");
    Serial.println(mps, 3);
  }

  Serial.println("Direction Change -> Back");
  delay(1000);
}
