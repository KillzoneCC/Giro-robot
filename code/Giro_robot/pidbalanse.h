#ifndef Pid_h
#define Pid_h

#include "Arduino.h"

class Pid
{
  public: 
    Pid(float p, float i, float d, float limit); 
    float updatePID(float target, float current, float deltaTime); 
    void resetPID(); 
    
    void setP(float p); 
    void setI(float i); 
    void setD(float d); 


    float getP(); 
    float getI(); 
    float getD(); 
   

  private: 
    float _P; 
    float _I; 
    float _D; 
    float _limit; 
    float _integratedError; 
    float _lastError; 
    float _result;
};
// Сделано в заголовке, чтобы проект Arduino собирался без отдельного pid.cpp.

inline Pid::Pid(float p, float i, float d, float limit)
  : _P(p),
    _I(i),
    _D(d),
    _limit(fabsf(limit)),
    _integratedError(0.0f),
    _lastError(0.0f),
    _result(0.0f) {}

inline void Pid::resetPID() {
  _integratedError = 0.0f;
  _lastError = 0.0f;
  _result = 0.0f;
}

inline float Pid::updatePID(float target, float current, float deltaTime) {
  // Защита от деления на ноль и "плохого" dt
  if (!(deltaTime > 0.0f)) {
    deltaTime = 0.0f;
  }

  const float error = target - current;

  // Интегральная часть
  if (deltaTime > 0.0f) {
    _integratedError += error * deltaTime;
  }

  // Анти-виндап: ограничиваем интеграл так, чтобы I-составляющая не могла
  // "перетянуть" выход за пределы лимита.
  if (_I != 0.0f && _limit > 0.0f) {
    const float maxInt = _limit / fabsf(_I);
    _integratedError = constrain(_integratedError, -maxInt, maxInt);
  }

  // Дифференциальная часть
  float deriv = 0.0f;
  if (deltaTime > 0.0f) {
    deriv = (error - _lastError) / deltaTime;
  }
  _lastError = error;

  const float out = (_P * error) + (_I * _integratedError) + (_D * deriv);

  if (_limit > 0.0f) {
    _result = constrain(out, -_limit, _limit);
  } else {
    _result = out;
  }

  return _result;
}

inline void Pid::setP(float p) { _P = p; }
inline void Pid::setI(float i) { _I = i; }
inline void Pid::setD(float d) { _D = d; }

inline float Pid::getP() { return _P; }
inline float Pid::getI() { return _I; }
inline float Pid::getD() { return _D; }
// ====================================================================

#endif