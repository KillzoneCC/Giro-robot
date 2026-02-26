/**
 * Giro-Robot — фундамент управления (Twist)
 * =========================================
 * Интерфейс: linear [-1..1], turn [-1..1].
 * Конвертация в команды для стабилизатора и моторов.
 * Заготовка под будущее управление (пульт, Bluetooth и т.д.).
 */

#ifndef TWIST_CONTROL_H
#define TWIST_CONTROL_H

#include "Arduino.h"

struct Twist {
  float linear;   // -1..1 (назад/вперёд)
  float turn;     // -1..1 (влево/вправо)
};

class TwistControl {
public:
  TwistControl() : _linear(0), _turn(0) {}

  void setTwist(float linear, float turn) {
    _linear = constrain(linear, -1.0f, 1.0f);
    _turn = constrain(turn, -1.0f, 1.0f);
  }

  void setTwist(const Twist& t) {
    setTwist(t.linear, t.turn);
  }

  float getLinear() const { return _linear; }
  float getTurn() const { return _turn; }

  /** Для стабилизатора: целевая линейная скорость (эквивалент) */
  float getTargetLinear() const { return _linear; }

  /** Для дифференциального привода: разница левый/правый */
  void getDifferentialSpeeds(float& left, float& right) const {
    left = _linear - _turn;
    right = _linear + _turn;
    float m = fmaxf(fmaxf(fabsf(left), fabsf(right)), 0.001f);
    if (m > 1.0f) { left /= m; right /= m; }
  }

  void stop() { _linear = 0; _turn = 0; }

private:
  float _linear;
  float _turn;
};

#endif
