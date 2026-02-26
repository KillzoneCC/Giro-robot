/**
 * Giro-Robot — ориентация по одной оси (pitch)
 * ============================================
 * Угол наклона вперёд/назад. Комплементарный фильтр:
 * акселерометр (гравитация) + гироскоп (интеграция).
 * Roll и yaw не используются — робот балансирует только по pitch.
 */

#ifndef ORIENTATION_H
#define ORIENTATION_H

#include "Arduino.h"
#include <math.h>

#define RAD_TO_DEG  57.29577951308232f

class Orientation {
public:
  Orientation() : _angle(0), _alpha(0.995f) {}

  /**
   * Обновить угол. accX,accY,accZ — в g, gyroPitch — °/s.
   * Возвращает угол pitch в градусах (вперёд +, назад -).
   */
  float update(float accX, float accY, float accZ, float gyroPitch, float dt) {
    float accPitch = atan2f(-accY, sqrtf(accX*accX + accZ*accZ + 0.001f)) * RAD_TO_DEG;
    _angle = _alpha * (_angle + gyroPitch * dt) + (1.0f - _alpha) * accPitch;
    return _angle;
  }

  void setAngle(float a) { _angle = a; }
  float getAngle() const { return _angle; }
  void setAlpha(float a) { _alpha = constrain(a, 0.0f, 1.0f); }

private:
  float _angle;
  float _alpha;
};

#endif
