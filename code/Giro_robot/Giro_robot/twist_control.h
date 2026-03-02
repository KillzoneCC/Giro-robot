/**
 * Giro-Robot — управление скоростью и поворотом
 * =============================================
 * Целевая скорость в м/с. Плавный переход к новой цели (rate limit).
 * 0 м/с = баланс без движения (полная остановка).
 */

#ifndef TWIST_CONTROL_H
#define TWIST_CONTROL_H

#include "Arduino.h"
#include "config.h"

class TwistControl {
public:
  TwistControl() : _targetSpeedMps(0), _rampedSpeedMps(0), _turn(0) {}

  /** Установить целевую скорость в м/с (отрицательная = назад). 0 = остановка/баланс. */
  void setTargetSpeedMps(float mps) {
    _targetSpeedMps = constrain(mps, -MAX_TARGET_SPEED_MPS, MAX_TARGET_SPEED_MPS);
  }

  /** Установить linear [-1..1] + turn [-1..1] (для совместимости) */
  void setTwist(float linear, float turn) {
    _targetSpeedMps = constrain(linear * MAX_TARGET_SPEED_MPS, -MAX_TARGET_SPEED_MPS, MAX_TARGET_SPEED_MPS);
    _turn = constrain(turn, -1.0f, 1.0f);
  }

  /** Установить только поворот (скорость не меняется) */
  void setTurn(float turn) { _turn = constrain(turn, -1.0f, 1.0f); }

  /** Обновить плавный переход к целевой скорости. Вызывать каждый цикл. */
  void updateRamp(float dt) {
    if (dt <= 0.0f) return;
    float maxDelta = TARGET_SPEED_RAMP_MPS * dt;
    float diff = _targetSpeedMps - _rampedSpeedMps;
    if (fabsf(diff) <= maxDelta) {
      _rampedSpeedMps = _targetSpeedMps;
    } else {
      _rampedSpeedMps += (diff > 0 ? maxDelta : -maxDelta);
    }
  }

  /** Текущая целевая скорость (с учётом плавного перехода), м/с */
  float getTargetSpeedMps() const { return _rampedSpeedMps; }

  float getTurn() const { return _turn; }

  void stop() {
    _targetSpeedMps = 0;
    _rampedSpeedMps = 0;
    _turn = 0;
  }

  /** Для совместимости: linear как -1..1 */
  float getLinear() const {
    if (MAX_TARGET_SPEED_MPS <= 0.0f) return 0.0f;
    return _targetSpeedMps / MAX_TARGET_SPEED_MPS;
  }

private:
  float _targetSpeedMps;   // целевая скорость (пользователь)
  float _rampedSpeedMps;   // плавно меняющаяся к цели (идёт в PID)
  float _turn;
};

#endif
