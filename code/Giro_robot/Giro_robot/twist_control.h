/**
 * Giro-Robot — управление (linear, turn)
 * ======================================
 */

#ifndef TWIST_CONTROL_H
#define TWIST_CONTROL_H

#include "Arduino.h"

class TwistControl {
public:
  TwistControl() : _linear(0), _turn(0) {}

  void setTwist(float linear, float turn) {
    _linear = constrain(linear, -1.0f, 1.0f);
    _turn = constrain(turn, -1.0f, 1.0f);
  }

  float getLinear() const { return _linear; }
  float getTurn() const { return _turn; }
  void stop() { _linear = 0; _turn = 0; }

private:
  float _linear;
  float _turn;
};

#endif
