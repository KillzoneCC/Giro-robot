/**
 * Giro-Robot — модуль управления двигателями
 * ==========================================
 * Два шаговых мотора, управление через twist (linear + angular).
 * Arduino Nano: Timer1 (16-bit) + Timer2 (8-bit), оба на 2 MHz эквиваленте.
 */

#ifndef MOTORS_H
#define MOTORS_H

#include "Arduino.h"
#include "config.h"

#define TIMER_BASE_HZ       2000000

// Глобальные для ISR
extern volatile int8_t _directionMotor1;
extern volatile int8_t _directionMotor2;

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

    TCCR2A = (1 << WGM21);
    TCCR2B = (1 << CS22);
    OCR2A = 255;
    TCNT2 = 0;
    TIMSK2 |= (1 << OCIE2A);

    _directionMotor1 = 0;
    _directionMotor2 = 0;
  }

  /**
   * Twist-управление: linear [-1..1] вперёд/назад, turn [-1..1] влево/вправо.
   * turn=0 — прямо, turn=±1 — разворот на месте с заданной скоростью.
   */
  void setTwist(float linear, float turn) {
    float leftNorm = linear - turn;
    float rightNorm = linear + turn;

    // Масштабирование при выходе за [-1, 1]
    float maxVal = fmaxf(fmaxf(fabsf(leftNorm), fabsf(rightNorm)), 0.001f);
    if (maxVal > 1.0f) {
      leftNorm /= maxVal;
      rightNorm /= maxVal;
    }

    int16_t leftSteps = (int16_t)(leftNorm * MAX_STEPS_PER_SEC * MOTOR1_SCALE);
    int16_t rightSteps = (int16_t)(rightNorm * MAX_STEPS_PER_SEC * MOTOR2_SCALE);

    if (MOTOR2_INVERT) rightSteps = -rightSteps;

    _setMotorSpeed(leftSteps, 1);
    _setMotorSpeed(rightSteps, 2);
    _leftSpeed = leftSteps;
    _rightSpeed = rightSteps;
  }

  /** Прямая установка скорости для баланса (оба мотора одинаково) */
  void setBalanceSpeed(int16_t stepsPerSec) {
    int16_t s1 = (int16_t)(stepsPerSec * MOTOR1_SCALE);
    int16_t s2 = (int16_t)(stepsPerSec * MOTOR2_SCALE);
    if (MOTOR2_INVERT) s2 = -s2;
    _setMotorSpeed(s1, 1);
    _setMotorSpeed(s2, 2);
    _leftSpeed = s1;
    _rightSpeed = s2;
  }

  /** Раздельная установка левый/правый (для поворота) */
  void setLeftRight(int16_t leftSteps, int16_t rightSteps) {
    int16_t s1 = (int16_t)(leftSteps * MOTOR1_SCALE);
    int16_t s2 = (int16_t)(rightSteps * MOTOR2_SCALE);
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
    long period = (stepsPerSec != 0) ? (TIMER_BASE_HZ / abs(stepsPerSec)) : 65535;

    if (motorID == 1) {
      _directionMotor1 = (stepsPerSec > 0) ? 1 : (stepsPerSec < 0) ? -1 : 0;
      digitalWrite(STEPPER_1_DIR_PIN, (stepsPerSec < 0) ? HIGH : LOW);
      if (period > 65535) period = 65535;
      OCR1A = (uint16_t)period;
      if ((uint16_t)TCNT1 > (uint16_t)OCR1A) TCNT1 = 0;
    } else {
      _directionMotor2 = (stepsPerSec > 0) ? 1 : (stepsPerSec < 0) ? -1 : 0;
      digitalWrite(STEPPER_2_DIR_PIN, (stepsPerSec < 0) ? LOW : HIGH);
      long period2 = period / 4;
      if (period2 > 255) period2 = 255;
      if (period2 < 1) period2 = 1;
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
