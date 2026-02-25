/*
  Управление шаговыми двигателями — извлечено из balansing_robot.
  Аппаратные таймеры (Timer1, Timer2) для точной генерации STEP без блокировок.
  Интерфейс: setMotorSpeed(steps_per_sec, motorID), setFromPID(u_pid, pidLimit).
  Пины STEP/DIR как в balansing_robot (A4988/DRV8825/TMC2208).
  Адаптировано для Arduino Nano: Timer3 заменён на Timer2 (Nano не имеет Timer3).
*/
#ifndef Shag_h
#define Shag_h

#include "Arduino.h"

// ================= ПИНЫ (как balansing_robot pinConfig) =================
// Arduino Nano: D3,D5 — мотор 1; D9,D10 — мотор 2
#ifndef STEPPER_1_STEP_PIN
#define STEPPER_1_STEP_PIN  3
#endif
#ifndef STEPPER_1_DIR_PIN
#define STEPPER_1_DIR_PIN   5
#endif
#ifndef STEPPER_2_STEP_PIN
#define STEPPER_2_STEP_PIN  9
#endif
#ifndef STEPPER_2_DIR_PIN
#define STEPPER_2_DIR_PIN   10
#endif
// ENABLE (опционально): активный LOW. Если не определён — не используется.
#ifdef STEPPER_1_ENABLE_PIN
#define HAS_ENABLE_PINS 1
#endif
#ifdef STEPPER_2_ENABLE_PIN
#define HAS_ENABLE_PINS 1
#endif

#ifndef MAX_STEPS_PER_SEC
#define MAX_STEPS_PER_SEC 14000.0f   // макс. скорость (steps/s), как в balansing_robot
#endif

// Таймер: 2 MHz (prescaler 8) для Timer1. Timer2 на Nano: 8-bit, prescaler 1024 => 15625 Hz.
#define TIMER1_BASE_HZ 2000000
#define TIMER2_BASE_HZ 15625

// Глобальные переменные для ISR (как в balansing_robot)
extern volatile int8_t _directionMotor1;
extern volatile int8_t _directionMotor2;

// 0.5 us задержка (8 nop при 16 MHz)
inline void delay_05us() {
  __asm__ __volatile__ (
    "nop" "\n\t" "nop" "\n\t" "nop" "\n\t" "nop" "\n\t"
    "nop" "\n\t" "nop" "\n\t" "nop" "\n\t" "nop");
}


class BalanceStepper {
public:
  BalanceStepper() : _lastPid(0.0f), _pidLimit(255.0f) {}

  void begin() {
    pinMode(STEPPER_1_STEP_PIN, OUTPUT);
    pinMode(STEPPER_1_DIR_PIN, OUTPUT);
    pinMode(STEPPER_2_STEP_PIN, OUTPUT);
    pinMode(STEPPER_2_DIR_PIN, OUTPUT);
    digitalWrite(STEPPER_1_STEP_PIN, LOW);
    digitalWrite(STEPPER_2_STEP_PIN, LOW);

#ifdef HAS_ENABLE_PINS
    pinMode(STEPPER_1_ENABLE_PIN, OUTPUT);
    pinMode(STEPPER_2_ENABLE_PIN, OUTPUT);
    digitalWrite(STEPPER_1_ENABLE_PIN, HIGH);  // disable
    digitalWrite(STEPPER_2_ENABLE_PIN, HIGH);
#endif

    // Timer1 — мотор 1 (как balansing_robot)
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11);   // CTC, prescaler 8 => 2 MHz
    OCR1A = 65535;
    TCNT1 = 0;
    TIMSK1 |= (1 << OCIE1A);

    // Timer2 — мотор 2 (Nano: 8-bit, CTC mode 2, prescaler 1024 => 15625 Hz)
    TCCR2A = (1 << WGM21);  // CTC mode
    TCCR2B = (1 << WGM22) | (1 << CS22) | (1 << CS21) | (1 << CS20);  // prescaler 1024
    OCR2A = 255;
    TCNT2 = 0;
    TIMSK2 |= (1 << OCIE2A);

    _directionMotor1 = 0;
    _directionMotor2 = 0;
  }

  // Установить скорость мотора (steps/s). Положительная = вперёд, отрицательная = назад.
  void setMotorSpeed(int16_t stepsPerSec, int motorID) {
    long timer_period;
    int16_t speed = stepsPerSec;

    if (motorID == 1) {
      if (speed > 0) {
        timer_period = TIMER1_BASE_HZ / speed;
        _directionMotor1 = 1;
        digitalWrite(STEPPER_1_DIR_PIN, LOW);
      } else if (speed < 0) {
        timer_period = TIMER1_BASE_HZ / (-speed);
        _directionMotor1 = -1;
        digitalWrite(STEPPER_1_DIR_PIN, HIGH);
      } else {
        timer_period = 65535;
        _directionMotor1 = 0;
      }
      if (timer_period > 65535) timer_period = 65535;
      OCR1A = (uint16_t)timer_period;
      if ((uint16_t)TCNT1 > (uint16_t)OCR1A) TCNT1 = 0;

    } else if (motorID == 2) {
      // Timer2: 15625 Hz. Мотор 2 зеркально — инвертируем DIR, чтобы колёса крутились в одну сторону
      if (speed > 0) {
        timer_period = TIMER2_BASE_HZ / speed;
        _directionMotor2 = 1;
        digitalWrite(STEPPER_2_DIR_PIN, HIGH);  // инвертировано: мотор 2 с другой стороны
      } else if (speed < 0) {
        timer_period = TIMER2_BASE_HZ / (-speed);
        _directionMotor2 = -1;
        digitalWrite(STEPPER_2_DIR_PIN, LOW);   // инвертировано
      } else {
        timer_period = 255;
        _directionMotor2 = 0;
      }
      if (timer_period > 255) timer_period = 255;
      if (timer_period < 1) timer_period = 1;
      OCR2A = (uint8_t)timer_period;
      if (TCNT2 > OCR2A) TCNT2 = 0;
    }
  }

  // Вход от PID: u_pid в [-pidLimit, +pidLimit]. Конвертирует в steps/s и вызывает setMotorSpeed.
  // Оба мотора для баланса: один вперёд, другой назад (противоположные направления).
  void setFromPID(float u_pid, float pidLimit = 255.0f) {
    _lastPid = u_pid;
    _pidLimit = (pidLimit > 0.1f) ? pidLimit : 255.0f;

    if (fabsf(u_pid) < 1.0f) {
      setMotorSpeed(0, 1);
      setMotorSpeed(0, 2);
      return;
    }

    // u_pid/pidLimit -> [-1, 1], умножаем на MAX_STEPS_PER_SEC
    float norm = constrain(u_pid / _pidLimit, -1.0f, 1.0f);
    int16_t speed = (int16_t)(norm * MAX_STEPS_PER_SEC);

    // Оба мотора в одну сторону (колёса крутятся одинаково для баланса)
    setMotorSpeed(speed, 1);
    setMotorSpeed(speed, 2);
  }

  void enableMotors() {
#ifdef HAS_ENABLE_PINS
    digitalWrite(STEPPER_1_ENABLE_PIN, LOW);
    digitalWrite(STEPPER_2_ENABLE_PIN, LOW);
#endif
  }

  void stop() {
    _lastPid = 0.0f;
    setMotorSpeed(0, 1);
    setMotorSpeed(0, 2);
  }

  float getSpeed() const { return fabsf(_lastPid); }

private:
  float _lastPid;
  float _pidLimit;
};

// Глобальные для ISR
volatile int8_t _directionMotor1 = 0;
volatile int8_t _directionMotor2 = 0;

#endif // Shag_h
