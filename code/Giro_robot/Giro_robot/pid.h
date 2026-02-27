/**
 * Giro-Robot — PID-регулятор
 * ==========================
 * Простой PID с антивиндапом интеграла.
 */

#ifndef PID_H
#define PID_H

#include "Arduino.h"
#include <math.h>

class Pid {
public:
  Pid(float kp, float ki, float kd, float limit)
    : _kp(kp), _ki(ki), _kd(kd), _limit(fabsf(limit)),
      _integral(0), _lastError(0), _lastP(0), _lastI(0), _lastD(0), _lastErrorOut(0) {}

  float update(float target, float current, float dt) {
    if (dt <= 0.0f) return 0.0f;
    float err = target - current;
    _integral += err * dt;

    if (_ki != 0.0f && _limit > 0.0f) {
      float maxInt = _limit / fabsf(_ki);
      _integral = constrain(_integral, -maxInt, maxInt);
    }

    float deriv = (err - _lastError) / dt;
    _lastError = err;

    _lastP = _kp * err;
    _lastI = _ki * _integral;
    _lastD = _kd * deriv;
    _lastErrorOut = err;

    float out = _lastP + _lastI + _lastD;
    return _limit > 0 ? constrain(out, -_limit, _limit) : out;
  }

  void reset() {
    _integral = 0;
    _lastError = 0;
  }

  void setKp(float kp) { _kp = kp; }
  void setKi(float ki) { _ki = ki; }
  void setKd(float kd) { _kd = kd; }
  void setLimit(float limit) { _limit = fmaxf(fabsf(limit), 1.0f); }

  float getLastP() const { return _lastP; }
  float getLastI() const { return _lastI; }
  float getLastD() const { return _lastD; }
  float getLastError() const { return _lastErrorOut; }

private:
  float _kp, _ki, _kd, _limit;
  float _integral, _lastError;
  float _lastP, _lastI, _lastD, _lastErrorOut;
};

#endif
