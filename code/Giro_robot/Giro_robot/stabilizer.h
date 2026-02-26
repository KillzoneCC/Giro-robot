/**
 * Giro-Robot — стабилизация
 * =========================
 * Один PID: угол → скорость моторов.
 * targetAngle = offset + linear*LEAN_SCALE (наклон для движения).
 */

#ifndef STABILIZER_H
#define STABILIZER_H

#include "Arduino.h"
#include "config.h"
#include "pid.h"
#include <math.h>

#define LEAN_SCALE  5.0f

class Stabilizer {
public:
  Stabilizer()
    : _pid(PID_KP, PID_KI, PID_KD, PID_LIMIT),
      _targetOffset(0),
      _tuneOffset(0),
      _motorSpeed(0) {}

  float update(float linear, float angle, float dt) {
    float targetAngle = _targetOffset + _tuneOffset + linear * LEAN_SCALE;
    _motorSpeed = BALANCE_SIGN * (-_pid.update(targetAngle, angle, dt));
    return _motorSpeed;
  }

  void setTargetOffset(float offset) { _targetOffset = offset; }
  float getTargetOffset() const { return _targetOffset; }
  float getMotorSpeed() const { return _motorSpeed; }
  void setTuneOffset(float offset) { _tuneOffset = offset; }

  void setPid(float kp, float ki, float kd, float limit) {
    _pid.setKp(kp);
    _pid.setKi(ki);
    _pid.setKd(kd);
    _pid.setLimit(limit);
  }

  void reset() {
    _pid.reset();
    _tuneOffset = 0;
    _motorSpeed = 0;
  }

private:
  Pid _pid;
  float _targetOffset;
  float _tuneOffset;
  float _motorSpeed;
};

#endif
