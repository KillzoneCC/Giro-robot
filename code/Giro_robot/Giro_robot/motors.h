/**
 * Giro-Robot — модуль управления двигателями
 * ==========================================
 * Timer1: мотор 1. Timer2: мотор 2 (инициализация исправлена для надёжной работы).
 */

#ifndef MOTORS_H
#define MOTORS_H

#include "Arduino.h"
#include "config.h"

#define TIMER1_BASE_HZ      2000000
#define TIMER2_BASE_HZ      15625

// Глобальные для ISR
extern volatile int8_t _directionMotor1;
extern volatile int8_t _directionMotor2;

// Как в старой версии (shag.h) — 8 NOP ≈ 0.5 µs
inline void delay_05us() {
  __asm__ __volatile__ (
    "nop" "\n\t" "nop" "\n\t" "nop" "\n\t" "nop" "\n\t"
    "nop" "\n\t" "nop" "\n\t" "nop" "\n\t" "nop");
}

class Motors {
public:
  Motors() : _leftSpeed(0), _rightSpeed(0), _enabled(false) {}

  void begin() {
    pinMode(STEPPER_1_STEP_PIN, OUTPUT);
    pinMode(STEPPER_1_DIR_PIN, OUTPUT);
    pinMode(STEPPER_2_STEP_PIN, OUTPUT);
    pinMode(STEPPER_2_DIR_PIN, OUTPUT);
    digitalWrite(STEPPER_1_STEP_PIN, LOW);
    digitalWrite(STEPPER_2_STEP_PIN, LOW);

    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11);
    OCR1A = 65535;
    TCNT1 = 0;
    TIMSK1 |= (1 << OCIE1A);

    TCCR2A = 0;
    TCCR2B = 0;
    TIMSK2 &= ~(1 << OCIE2A);
    TCNT2 = 0;
    OCR2A = 255;
    TCCR2A = (1 << WGM21);
    TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);
    TIMSK2 |= (1 << OCIE2A);

    _directionMotor1 = 0;
    _directionMotor2 = 0;
  }

  /** Оба мотора одинаково (баланс), ограничено MOTOR_SPEED_LIMIT */
  void setBalanceSpeed(int16_t stepsPerSec) {
    int16_t s = (int16_t)constrain((long)stepsPerSec, -MOTOR_SPEED_LIMIT, MOTOR_SPEED_LIMIT);
    int16_t s1 = (int16_t)(s * MOTOR1_SCALE);
    int16_t s2 = (int16_t)(s * MOTOR2_SCALE);
    if (MOTOR2_INVERT) s2 = -s2;
    _setMotorSpeed(s1, 1);
    _setMotorSpeed(s2, 2);
    _leftSpeed = s1;
    _rightSpeed = s2;
  }

  /** Раздельная установка левый/правый (для поворота), ограничено MOTOR_SPEED_LIMIT */
  void setLeftRight(int16_t leftSteps, int16_t rightSteps) {
    int16_t l = (int16_t)constrain((long)leftSteps, -MOTOR_SPEED_LIMIT, MOTOR_SPEED_LIMIT);
    int16_t r = (int16_t)constrain((long)rightSteps, -MOTOR_SPEED_LIMIT, MOTOR_SPEED_LIMIT);
    int16_t s1 = (int16_t)(l * MOTOR1_SCALE);
    int16_t s2 = (int16_t)(r * MOTOR2_SCALE);
    if (MOTOR2_INVERT) s2 = -s2;
    _setMotorSpeed(s1, 1);
    _setMotorSpeed(s2, 2);
    _leftSpeed = s1;
    _rightSpeed = s2;
  }

  void enable()  { _enabled = true; }
  void disable() { _enabled = false; stop(); }

  void stop() {
    _setMotorSpeed(0, 1);
    _setMotorSpeed(0, 2);
    _leftSpeed = 0;
    _rightSpeed = 0;
  }

  bool isEnabled() const { return _enabled; }
  int16_t getLeftSpeed() const { return _leftSpeed; }
  int16_t getRightSpeed() const { return _rightSpeed; }

private:
  void _setMotorSpeed(int16_t stepsPerSec, int motorID) {
    int16_t sps = abs(stepsPerSec);

    if (motorID == 1) {
      long period = (sps > 0) ? (TIMER1_BASE_HZ / sps) : 65535;
      if (period > 65535) period = 65535;
      _directionMotor1 = (stepsPerSec > 0) ? 1 : (stepsPerSec < 0) ? -1 : 0;
      digitalWrite(STEPPER_1_DIR_PIN, (stepsPerSec < 0) ? HIGH : LOW);
      OCR1A = (uint16_t)period;
      if ((uint16_t)TCNT1 > (uint16_t)OCR1A) TCNT1 = 0;
    } else {
      long period2 = (sps > 0) ? (TIMER2_BASE_HZ / sps) : 255;
      if (period2 > 255) period2 = 255;
      if (period2 < 1) period2 = 1;
      _directionMotor2 = (stepsPerSec > 0) ? 1 : (stepsPerSec < 0) ? -1 : 0;
      digitalWrite(STEPPER_2_DIR_PIN, (stepsPerSec < 0) ? LOW : HIGH);
      OCR2A = (uint8_t)(period2 - 1);
      if (TCNT2 > OCR2A) TCNT2 = 0;
    }
  }

  int16_t _leftSpeed, _rightSpeed;
  bool _enabled;
};

volatile int8_t _directionMotor1 = 0;
volatile int8_t _directionMotor2 = 0;

#endif
