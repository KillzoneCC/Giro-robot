/**
 * Giro-Robot — объединение скорости комплементарным фильтром
 * ===========================================================
 * v_wheel (колёса, без дрейфа) + v_accel (интеграл акселерометра, быстрый отклик).
 * alpha: доверие колёсам на низких частотах. (1-alpha): вклад IMU на высоких.
 * drift_beta: подтягивание vAccel к колёсам против дрейфа интегратора.
 */

#ifndef VELOCITY_FUSION_H
#define VELOCITY_FUSION_H

#include "Arduino.h"
#include "config.h"
#include <math.h>

#define RAD_TO_DEG  57.29577951308232f
#define GRAVITY_MS2 9.80665f

class VelocityFusion {
public:
  VelocityFusion() : _v(0), _vAccel(0), _alpha(0.85f), _driftBeta(0.02f) {}

  /**
   * Обновить оценку скорости.
   * accX,accY,accZ — в g, angle — pitch в градусах, vWheel — м/с от колёс, dt — с.
   * vAccel = интеграл продольного ускорения (с вычетом гравитации при наклоне).
   * Выход: комплементарная смесь vWheel и vAccel, с коррекцией дрейфа.
   */
  float update(float accX, float accY, float accZ, float angle, float vWheel, float dt) {
    if (dt <= 0.0f) return _v;

    float angleRad = angle / RAD_TO_DEG;
    float gForward = sinf(angleRad);
    float accLong = (-accY - gForward) * GRAVITY_MS2;
    accLong = constrain(accLong, -8.0f, 8.0f);

    // Интеграл акселерометра — реальная скорость по IMU
    _vAccel += accLong * dt;

    // Подтягивание vAccel к колёсам против дрейфа
    _vAccel += _driftBeta * (vWheel - _vAccel);

    // Комплементарный выход: колёса (низкие частоты) + IMU (высокие)
    _v = _alpha * vWheel + (1.0f - _alpha) * _vAccel;
    return _v;
  }

  void setVelocity(float v) { _v = v; _vAccel = v; }
  float getVelocity() const { return _v; }
  float getVelocityAccel() const { return _vAccel; }  // скорость только по IMU
  void setAlpha(float a) { _alpha = constrain(a, 0.0f, 1.0f); }
  void setDriftBeta(float b) { _driftBeta = constrain(b, 0.001f, 0.5f); }

private:
  float _v, _vAccel, _alpha, _driftBeta;
};

#endif
