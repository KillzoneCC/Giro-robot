#ifndef Shag_h
#define Shag_h

#include "Arduino.h"

#define STEPPER_1_STEP_PIN  3
#define STEPPER_1_DIR_PIN   5
#define STEPPER_2_STEP_PIN  9
#define STEPPER_2_DIR_PIN   10

#ifndef MAX_STEPS_PER_SEC
#define MAX_STEPS_PER_SEC 14000.0f
#endif

#define TIMER1_BASE_HZ 2000000
#define TIMER2_BASE_HZ 15625

extern volatile int8_t _directionMotor1;
extern volatile int8_t _directionMotor2;

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

    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11);
    OCR1A = 65535;
    TCNT1 = 0;
    TIMSK1 |= (1 << OCIE1A);

    TCCR2A = (1 << WGM21);  // CTC mode
    TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);  // prescaler 1024, WGM22=0
    OCR2A = 255;
    TCNT2 = 0;
    TIMSK2 |= (1 << OCIE2A);

    _directionMotor1 = 0;
    _directionMotor2 = 0;
  }

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
      if (speed > 0) {
        timer_period = TIMER2_BASE_HZ / speed;
        _directionMotor2 = 1;
        digitalWrite(STEPPER_2_DIR_PIN, HIGH);
      } else if (speed < 0) {
        timer_period = TIMER2_BASE_HZ / (-speed);
        _directionMotor2 = -1;
        digitalWrite(STEPPER_2_DIR_PIN, LOW);
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

  void setFromPID(float u_pid, float pidLimit = 255.0f) {
    _lastPid = u_pid;
    _pidLimit = (pidLimit > 0.1f) ? pidLimit : 255.0f;
    if (fabsf(u_pid) < 1.0f) {
      setMotorSpeed(0, 1);
      setMotorSpeed(0, 2);
      return;
    }
    float norm = constrain(u_pid / _pidLimit, -1.0f, 1.0f);
    int16_t speed = (int16_t)(norm * MAX_STEPS_PER_SEC);
    setMotorSpeed(speed, 1);
    setMotorSpeed(speed, 2);
  }

  void enableMotors() {}

  void stop() {
    _lastPid = 0.0f;
    setMotorSpeed(0, 1);
    setMotorSpeed(0, 2);
  }

private:
  float _lastPid;
  float _pidLimit;
};

volatile int8_t _directionMotor1 = 0;
volatile int8_t _directionMotor2 = 0;

#endif
