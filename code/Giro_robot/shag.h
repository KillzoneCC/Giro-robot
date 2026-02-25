/*
  Управление шаговыми двигателями для гиро-балансира.
  Цепочка: IMU → угол → PID → u_balance → setFromPID → step().
  Пины как в robot_bag / balansing_robot pinConfig.
*/
#ifndef SERIAL_BAUD
#define SERIAL_BAUD 115200
#endif

#ifndef Shag_h
#define Shag_h

#include <Stepper.h>

// Пины 4-фазных шаговиков (ULN2003/28BYJ-48): порядок фаз важен
// Мотор 1: 3, 7, 8, 5
// Мотор 2: 9, 7, 8, 10  — пины 7,8 общие
#define MOTOR1_PIN1  3
#define MOTOR1_PIN2  7
#define MOTOR1_PIN3  8
#define MOTOR1_PIN4  5
#define MOTOR2_PIN1  9
#define MOTOR2_PIN2  7
#define MOTOR2_PIN3  8
#define MOTOR2_PIN4  10

const int stepsPerRevolution = 200;
Stepper myStepper(stepsPerRevolution, MOTOR2_PIN1, MOTOR2_PIN2, MOTOR2_PIN3, MOTOR2_PIN4);
Stepper myStepper2(stepsPerRevolution, MOTOR1_PIN1, MOTOR1_PIN2, MOTOR1_PIN3, MOTOR1_PIN4);

#ifndef STEPS_PER_TICK_MAX
#define STEPS_PER_TICK_MAX 140  // макс. шагов за цикл
#endif
#ifndef STEP_GAIN
#define STEP_GAIN 6.0f
#endif
#ifndef MAX_STEPS_PER_UPDATE
#define MAX_STEPS_PER_UPDATE 70
#endif
#ifndef BALANCE_INVERT
#define BALANCE_INVERT 0       // 1 = инвертировать, 0 = не инвертировать (исправлено: ехал не туда)
#endif
#ifndef STEPPER_SPEED_LIMIT
#define STEPPER_SPEED_LIMIT 4000   // предел RPM (выше ~4000 моторы не ускоряются, ~11000 steps/s)
#endif
#ifndef MAX_VELOCITY_STEPS_PER_SEC
#define MAX_VELOCITY_STEPS_PER_SEC 11000.0f  // макс. скорость по тесту (steps/s)
#endif

class BalanceStepper {
public:
  BalanceStepper() : _lastPid(0.0f), _pidLimit(255.0f) {}

  void begin() {
    myStepper.setSpeed(STEPPER_SPEED_LIMIT);
    myStepper2.setSpeed(STEPPER_SPEED_LIMIT);
  }

  // u_pid: выход PID+feedforward → целевая скорость (steps/s). Положительный = одна сторона.
  void setFromPID(float u_pid, float pidLimit = 255.0f) {
    _lastPid = (BALANCE_INVERT) ? -u_pid : u_pid;
    _pidLimit = (pidLimit > 0.1f) ? pidLimit : 255.0f;
  }

  // dt в секундах. Число шагов = target_velocity * dt (управление по скорости)
  void update(float dt) {
    const float u = _lastPid;
    if (fabsf(u) < 1.0f) return;
    if (dt <= 0.0f) return;

    // Целевая скорость (steps/s): u/pidLimit в [-1,1] → [-MAX_VELOCITY, +MAX_VELOCITY]
    float target_velocity = (u / _pidLimit) * MAX_VELOCITY_STEPS_PER_SEC;
    target_velocity = constrain(target_velocity, -MAX_VELOCITY_STEPS_PER_SEC, MAX_VELOCITY_STEPS_PER_SEC);

    int n = (int)(fabsf(target_velocity) * dt + 0.5f);
    if (n < 1) return;
    if (n > MAX_STEPS_PER_UPDATE) n = MAX_STEPS_PER_UPDATE;

    int s1 = (target_velocity >= 0.0f) ? 1 : -1;
    int s2 = -s1;

    uint32_t rpm = (uint32_t)(fabsf(target_velocity) * 60.0f / stepsPerRevolution + 100.0f);
    if (rpm > STEPPER_SPEED_LIMIT) rpm = STEPPER_SPEED_LIMIT;
    if (rpm < 200) rpm = 200;
    myStepper.setSpeed(rpm);
    myStepper2.setSpeed(rpm);

    for (int i = 0; i < n; i++) {
      myStepper.step(s1);
      myStepper2.step(s2);
    }
  }

  void stop() { _lastPid = 0.0f; }
  float getSpeed() const { return fabsf(_lastPid); }

private:
  float _lastPid;
  float _pidLimit;
};

#endif // Shag_h
