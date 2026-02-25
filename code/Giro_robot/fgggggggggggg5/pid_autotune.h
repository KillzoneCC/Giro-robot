/*
  PID автотюнер по шаговому отклику.
  По команде даёт небольшой шаг по целевой позиции, смотрит перерегулирование и время
  установления, предлагает скорректированные Kp, Ki, Kd (и при желании применяет их).
*/
#ifndef SERIAL_BAUD
#define SERIAL_BAUD 115200
#endif
/*
  Использование:
  - Отправить 'T' по Serial → запуск тюна по активной оси баланса.
  - Держи робота стабильно; через ~4 сек тюнер выведет предложенные значения и спросит применить (A) или нет.
*/

#ifndef PID_AUTOTUNE_H
#define PID_AUTOTUNE_H

#include "Arduino.h"

// Параметры шага тюнера (небольшой шаг, чтобы не сбить баланс)
#ifndef TUNE_STEP_DEG
#define TUNE_STEP_DEG  1.5f
#endif
#ifndef TUNE_STEP_HOLD_MS
#define TUNE_STEP_HOLD_MS  1500
#endif
#ifndef TUNE_OBSERVE_MS
#define TUNE_OBSERVE_MS  2000
#endif
#ifndef TUNE_SETTLE_THRESH_DEG
#define TUNE_SETTLE_THRESH_DEG  0.5f
#endif
#ifndef TUNE_SETTLE_FOR_MS
#define TUNE_SETTLE_FOR_MS  400
#endif

enum TuneState {
  TUNE_IDLE = 0,
  TUNE_STEP_UP,
  TUNE_STEP_DOWN,
  TUNE_DONE
};

class PidAutotune {
public:
  PidAutotune() : _state(TUNE_IDLE), _base_target(0.0f), _step_offset(0.0f),
                  _start_ms(0), _settle_start_ms(0), _iae(0.0f), _max_overshoot(0.0f), _settle_ms(0),
                  _suggested_kp(0.0f), _suggested_ki(0.0f), _suggested_kd(0.0f),
                  _current_kp(0.0f), _current_ki(0.0f), _current_kd(0.0f), _use_pitch(false) {}

  void start(float current_target_balance_axis, bool balance_axis_is_pitch,
            float kp, float ki, float kd) {
    _state = TUNE_STEP_UP;
    _base_target = current_target_balance_axis;
    _step_offset = TUNE_STEP_DEG;
    _start_ms = millis();
    _iae = 0.0f;
    _max_overshoot = 0.0f;
    _settle_ms = 0;
    _use_pitch = balance_axis_is_pitch;
    _current_kp = kp;
    _current_ki = ki;
    _current_kd = kd;
  }

  bool isRunning() const { return _state != TUNE_IDLE && _state != TUNE_DONE; }
  bool isDone() const { return _state == TUNE_DONE; }

  float getTargetOffset() const {
    if (_state == TUNE_STEP_UP) return _step_offset;
    return 0.0f;
  }

  float getCurrentTarget() const { return _base_target + getTargetOffset(); }

  void update(float angle_balance_axis, float dt, uint32_t now_ms) {
    if (_state == TUNE_IDLE || _state == TUNE_DONE) return;

    const float target = getCurrentTarget();
    const float err = target - angle_balance_axis;
    _iae += fabsf(err) * dt;
    const float abs_err = fabsf(err);
    if (abs_err > _max_overshoot) _max_overshoot = abs_err;

    if (_state == TUNE_STEP_UP) {
      if ((uint32_t)(now_ms - _start_ms) >= TUNE_STEP_HOLD_MS) {
        _state = TUNE_STEP_DOWN;
        _step_offset = 0.0f;
        _start_ms = now_ms;
        _settle_ms = 0;
      }
      return;
    }

    if (_state == TUNE_STEP_DOWN) {
      if (abs_err <= TUNE_SETTLE_THRESH_DEG) {
        if (_settle_start_ms == 0) _settle_start_ms = now_ms;
        uint32_t elapsed = (uint32_t)(now_ms - _settle_start_ms);
        if (elapsed >= TUNE_SETTLE_FOR_MS && _settle_ms == 0)
          _settle_ms = (uint32_t)(now_ms - _start_ms);
      } else {
        _settle_start_ms = 0;
      }
      if ((uint32_t)(now_ms - _start_ms) >= TUNE_OBSERVE_MS) {
        _state = TUNE_DONE;
        computeSuggestions();
      }
    }
  }

  void computeSuggestions() {
    (void)_iae;
    _suggested_kp = _current_kp;
    _suggested_ki = _current_ki;
    _suggested_kd = _current_kd;

    if (_max_overshoot > 1.2f) {
      _suggested_kd *= 1.12f;
      _suggested_kp *= 0.97f;
    } else if (_max_overshoot > 0.8f) {
      _suggested_kd *= 1.06f;
    }

    if (_settle_ms > 0 && _settle_ms < 800) {
      _suggested_kp *= 1.04f;
    } else if (_settle_ms >= 1200) {
      _suggested_kp *= 0.96f;
    }

    if (_iae > 2.0f && _current_ki < 0.01f) {
      _suggested_ki = 0.008f;
    }

    _suggested_kp = constrain(_suggested_kp, 1.0f, 20.0f);
    _suggested_kd = constrain(_suggested_kd, 0.01f, 1.0f);
    _suggested_ki = constrain(_suggested_ki, 0.0f, 0.1f);
  }

  float getSuggestedKp() const { return _suggested_kp; }
  float getSuggestedKi() const { return _suggested_ki; }
  float getSuggestedKd() const { return _suggested_kd; }
  float getMaxOvershoot() const { return _max_overshoot; }
  uint32_t getSettleMs() const { return _settle_ms; }
  void resetState() { _state = TUNE_IDLE; }

private:
  TuneState _state;
  float _base_target;
  float _step_offset;
  uint32_t _start_ms;
  uint32_t _settle_start_ms;
  float _iae;
  float _max_overshoot;
  uint32_t _settle_ms;
  float _suggested_kp, _suggested_ki, _suggested_kd;
  float _current_kp, _current_ki, _current_kd;
  bool _use_pitch;
};

#endif
