/*
  Модуль управления шаговыми двигателями TMC2208 для балансирующего робота.
  Два мотора вращаются в противоположных направлениях для балансировки.
  Использует TMC2208Stepper (UART) + ручная генерация STEP/DIR.
*/

#ifndef Shag_h
#define Shag_h

#include <SoftwareSerial.h>
#include <TMC2208Stepper.h>

// ================= ПИНЫ МОТОР 1 =================
#ifndef DIR1_PIN
#define DIR1_PIN   5
#endif
#ifndef STEP1_PIN
#define STEP1_PIN  3
#endif
#ifndef UART1_RX
#define UART1_RX   7
#endif
#ifndef UART1_TX
#define UART1_TX   6
#endif

// ================= ПИНЫ МОТОР 2 =================
#ifndef DIR2_PIN
#define DIR2_PIN   10
#endif
#ifndef STEP2_PIN
#define STEP2_PIN  9
#endif
#ifndef UART2_RX
#define UART2_RX   4
#endif
#ifndef UART2_TX
#define UART2_TX   2
#endif

// ================= НАСТРОЙКИ TMC2208 =================
#ifndef R_SENSE
#define R_SENSE 0.11f  // Резистор токоизмерения
#endif
#ifndef TMC_RMS_CURRENT
#define TMC_RMS_CURRENT 1200  // Ток двигателя (мА RMS)
#endif
#ifndef TMC_MICROSTEPS
#define TMC_MICROSTEPS 16  // Микрошаги
#endif

// ================= НАСТРОЙКИ ВРЕМЕНИ =================
#ifndef PULSE_US
#define PULSE_US 10  // Длительность импульса шага (мкс)
#endif
#ifndef DIR_SETUP_US
#define DIR_SETUP_US 5  // Пауза смены направления (мкс)
#endif
#ifndef STEP_GAP_US
#define STEP_GAP_US 10  // Пауза между шагами моторов (мкс)
#endif

// ================= НАСТРОЙКИ СКОРОСТИ =================
#ifndef STEPPER_MAX_SPEED
#define STEPPER_MAX_SPEED 3000.0f  // шагов в секунду
#endif
#ifndef STEPPER_MIN_STEP_INTERVAL_US
#define STEPPER_MIN_STEP_INTERVAL_US 100  // минимальный интервал между шагами (мкс)
#endif

// Глобальные SoftwareSerial для TMC2208
static SoftwareSerial TMCuart1(UART1_RX, UART1_TX);
static SoftwareSerial TMCuart2(UART2_RX, UART2_TX);

class BalanceStepper {
public:
  BalanceStepper()
    : _driver1(&TMCuart1, false),
      _driver2(&TMCuart2, false),
      _currentSpeed(0.0f),
      _stepInterval(0),
      _lastStepTime(0),
      _direction(true)
  {
  }

  // Инициализация (вызвать в setup())
  void begin() {
    // Настройка пинов STEP/DIR
    pinMode(DIR1_PIN, OUTPUT);
    pinMode(STEP1_PIN, OUTPUT);
    pinMode(DIR2_PIN, OUTPUT);
    pinMode(STEP2_PIN, OUTPUT);
    
    digitalWrite(STEP1_PIN, LOW);
    digitalWrite(STEP2_PIN, LOW);
    digitalWrite(DIR1_PIN, HIGH);
    digitalWrite(DIR2_PIN, HIGH);
    
    // --- Инициализация драйвера 1 ---
    TMCuart1.begin(115200);
    delay(100);
    _driver1.pdn_disable(true);
    _driver1.mstep_reg_select(true);
    _driver1.microsteps(TMC_MICROSTEPS);
    _driver1.rms_current(TMC_RMS_CURRENT, 0.5, R_SENSE);
    _driver1.push();
    delay(50);
    
    // --- Инициализация драйвера 2 ---
    TMCuart2.begin(115200);
    delay(100);
    _driver2.pdn_disable(true);
    _driver2.mstep_reg_select(true);
    _driver2.microsteps(TMC_MICROSTEPS);
    _driver2.rms_current(TMC_RMS_CURRENT, 0.5, R_SENSE);
    _driver2.push();
    delay(50);
    
    _lastStepTime = micros();
  }

  // Шаг мотора 1
  inline void stepMotor1(bool forward) {
    digitalWrite(DIR1_PIN, forward ? HIGH : LOW);
    delayMicroseconds(DIR_SETUP_US);
    digitalWrite(STEP1_PIN, HIGH);
    delayMicroseconds(PULSE_US);
    digitalWrite(STEP1_PIN, LOW);
  }

  // Шаг мотора 2
  inline void stepMotor2(bool forward) {
    digitalWrite(DIR2_PIN, forward ? HIGH : LOW);
    delayMicroseconds(DIR_SETUP_US);
    digitalWrite(STEP2_PIN, HIGH);
    delayMicroseconds(PULSE_US);
    digitalWrite(STEP2_PIN, LOW);
  }

  // Шаг обоих моторов в противоположных направлениях (для баланса)
  // mainDir = true:  Мотор 1 вперед, Мотор 2 назад
  // mainDir = false: Мотор 1 назад,  Мотор 2 вперед
  inline void stepBothOpposite(bool mainDir) {
    stepMotor1(mainDir);
    delayMicroseconds(STEP_GAP_US);
    stepMotor2(!mainDir);  // Инверсия для противоположного вращения
  }

  // Установить скорость в шагах/секунду (положительная = вперёд, отрицательная = назад)
  void setSpeed(float speedStepsPerSec) {
    _currentSpeed = constrain(speedStepsPerSec, -STEPPER_MAX_SPEED, STEPPER_MAX_SPEED);
    
    if (fabs(_currentSpeed) < 1.0f) {
      _stepInterval = 0;  // Остановка
    } else {
      _stepInterval = (uint32_t)(1000000.0f / fabs(_currentSpeed));
      if (_stepInterval < STEPPER_MIN_STEP_INTERVAL_US) {
        _stepInterval = STEPPER_MIN_STEP_INTERVAL_US;
      }
    }
    
    _direction = (_currentSpeed >= 0.0f);
  }

  // Установить скорость на основе PWM-значения (0-255) и направления
  void setSpeedFromPWM(int pwmValue, bool direction) {
    float speed = map(abs(pwmValue), 0, 255, 0, (int)STEPPER_MAX_SPEED);
    if (!direction) speed = -speed;
    setSpeed(speed);
  }

  // === УПРАВЛЕНИЕ ОТ PID КОНТРОЛЛЕРА ===
  // Прямой вход: выход PID [-pidLimit .. +pidLimit]
  // Положительное значение → моторы в одну сторону, отрицательное → в противоположную
  // Оба шаговика вращаются синхронно от одного PID-сигнала (в противоположных направлениях для баланса)
  void setFromPID(float u_pid, float pidLimit = 255.0f) {
    if (fabsf(u_pid) < 1.0f) {
      stop();
      return;
    }
    // Нормализуем PID-выход к скорости: u_pid/pidLimit * STEPPER_MAX_SPEED
    float normalized = constrain(u_pid, -pidLimit, pidLimit) / pidLimit;
    float speed = normalized * STEPPER_MAX_SPEED;
    setSpeed(speed);
  }

  // Обновление моторов (вызывать как можно чаще в loop()!)
  void update() {
    if (_stepInterval == 0) return;  // Моторы остановлены
    
    uint32_t now = micros();
    if ((now - _lastStepTime) >= _stepInterval) {
      stepBothOpposite(_direction);
      _lastStepTime = now;
      _position += _direction ? 1 : -1;
    }
  }

  // Получить текущую скорость
  float getSpeed() const {
    return _currentSpeed;
  }

  // Получить текущую позицию (в шагах)
  long currentPosition() const {
    return _position;
  }

  // Сбросить позицию в 0
  void resetPosition() {
    _position = 0;
  }

  // Остановить моторы
  void stop() {
    _currentSpeed = 0;
    _stepInterval = 0;
  }

  // Установить ток (мА RMS) для обоих драйверов
  void setCurrent(uint16_t mA) {
    _driver1.rms_current(mA, 0.5, R_SENSE);
    _driver1.push();
    _driver2.rms_current(mA, 0.5, R_SENSE);
    _driver2.push();
  }

  // Установить микрошаги для обоих драйверов
  void setMicrosteps(uint16_t ms) {
    _driver1.microsteps(ms);
    _driver1.push();
    _driver2.microsteps(ms);
    _driver2.push();
  }

  // Доступ к драйверам для расширенных настроек
  TMC2208Stepper& getDriver1() { return _driver1; }
  TMC2208Stepper& getDriver2() { return _driver2; }

private:
  TMC2208Stepper _driver1;
  TMC2208Stepper _driver2;
  float _currentSpeed;
  uint32_t _stepInterval;
  uint32_t _lastStepTime;
  bool _direction;
  long _position = 0;
};

#endif // Shag_h