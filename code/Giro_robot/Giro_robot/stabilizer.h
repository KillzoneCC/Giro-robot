/**
 * Giro-Robot — алгоритм стабилизации
 * ==================================
 * Каскадный PID: целевая скорость → целевой угол → скорость моторов.
 * Подчинённое управление: задаём только скорости, углы подбирает сам.
 */

#ifndef STABILIZER_H
#define STABILIZER_H

#include "Arduino.h"
#include "config.h"
#include "pid.h"
#include <math.h>

class Stabilizer {
public:
  Stabilizer()
    : _pidAngle(PID_ANGLE_KP, PID_ANGLE_KI, PID_ANGLE_KD, PID_ANGLE_LIMIT),
      _pidMotor(PID_MOTOR_KP, PID_MOTOR_KI, PID_MOTOR_KD, PID_MOTOR_LIMIT),
      _targetAngleOffset(0),
      _tuneOffset(0),
      _motorSpeed(0),
      _motorSpeedPrev(0),
      _currentAngle(0) {}

  /**
   * Обновить стабилизацию. Возвращает скорость моторов (steps/s).
   * targetLinear [-1..1], angle — текущий угол баланса.
   * turn учитывается при выводе на моторы (в main loop).
   */
  float update(float targetLinear, float angle, float dt) {
    float targetSpeed = targetLinear * LINEAR_TO_STEPS;

    float targetAngle = _pidAngle.update(targetSpeed, _motorSpeed, dt) + _targetAngleOffset + _tuneOffset;
    float motorSpeed = BALANCE_SIGN * (-_pidMotor.update(targetAngle, angle, dt));

    // Мёртвая зона
    float err = angle - targetAngle;
    if (fabsf(err) < DEADBAND_DEG * 0.5f) motorSpeed = 0;
    else if (fabsf(err) < DEADBAND_DEG) motorSpeed *= 0.4f;

    // Slew rate
    float delta = motorSpeed - _motorSpeedPrev;
    if (fabsf(delta) > SLEW_RATE_MAX) {
      delta = (delta > 0) ? SLEW_RATE_MAX : -SLEW_RATE_MAX;
      motorSpeed = _motorSpeedPrev + delta;
    }
    _motorSpeedPrev = motorSpeed;
    _motorSpeed = motorSpeed;
    _currentAngle = angle;

    return motorSpeed;
  }

  void setTargetAngleOffset(float offset) { _targetAngleOffset = offset; }
  float getTargetAngleOffset() const { return _targetAngleOffset; }
  float getMotorSpeed() const { return _motorSpeed; }
  float getCurrentAngle() const { return _currentAngle; }

  /** Смещение цели для тюнинга (добавляется к targetAngle) */
  void setTuneOffset(float offset) { _tuneOffset = offset; }
  float getTuneOffset() const { return _tuneOffset; }

  /** Установить коэффициенты внутреннего PID (угол → моторы) */
  void setMotorPid(float kp, float ki, float kd, float limit) {
    _pidMotor.setKp(kp);
    _pidMotor.setKi(ki);
    _pidMotor.setKd(kd);
    _pidMotor.setLimit(limit);
  }

  void reset() {
    _pidAngle.reset();
    _pidMotor.reset();
    _tuneOffset = 0;
    _motorSpeed = 0;
    _motorSpeedPrev = 0;
  }

private:
  Pid _pidAngle;
  Pid _pidMotor;
  float _targetAngleOffset;
  float _tuneOffset;
  float _motorSpeed;
  float _motorSpeedPrev;
  float _currentAngle;
};

#endif
