/**
 * Giro-Robot — калибровка PID
 * ============================
 * Автотюн по шаговому отклику для внутреннего контура (угол → моторы).
 * Запуск: команда 'T' по Serial (робот должен балансировать!).
 * После тюна: 'A' — применить и сохранить в EEPROM, 'R' — отклонить.
 */

#ifndef PID_TUNE_H
#define PID_TUNE_H

#include "Arduino.h"
#include "config.h"
#include <EEPROM.h>

#define EEPROM_PID_MAGIC    0x5D1D
#define EEPROM_PID_ADDR    64   // после CalibData (~52 байт)

#define TUNE_STEP_DEG       1.5f
#define TUNE_STEP_HOLD_MS   1500
#define TUNE_OBSERVE_MS     2500
#define TUNE_SETTLE_THRESH  0.5f
#define TUNE_SETTLE_FOR_MS  400

struct PidParams {
  uint16_t magic;
  float kp, ki, kd, limit;
};

enum PidTuneState {
  TUNE_IDLE,
  TUNE_STEP_UP,
  TUNE_STEP_DOWN,
  TUNE_DONE,
  TUNE_WAIT_APPLY
};

class PidTune {
public:
  PidTune() : _state(TUNE_IDLE), _startMs(0), _settleStartMs(0),
              _maxOvershoot(0), _settleMs(0), _suggestedKp(0), _suggestedKi(0),
              _suggestedKd(0), _suggestedLimit(0) {}

  void start(float baseTarget, float kp, float ki, float kd, float limit) {
    _state = TUNE_STEP_UP;
    _baseTarget = baseTarget;
    _startMs = millis();
    _settleStartMs = 0;
    _maxOvershoot = 0;
    _settleMs = 0;
    _suggestedKp = kp;
    _suggestedKi = ki;
    _suggestedKd = kd;
    _suggestedLimit = limit;
  }

  bool isRunning() const {
    return _state == TUNE_STEP_UP || _state == TUNE_STEP_DOWN;
  }
  bool isDone() const { return _state == TUNE_DONE; }
  bool isWaitApply() const { return _state == TUNE_WAIT_APPLY; }

  /** Смещение цели: +step при STEP_UP, 0 иначе */
  float getTargetOffset() const {
    return (_state == TUNE_STEP_UP) ? TUNE_STEP_DEG : 0.0f;
  }

  void update(float angle, float dt, uint32_t nowMs) {
    if (_state == TUNE_IDLE || _state == TUNE_WAIT_APPLY) return;

    float target = _baseTarget + getTargetOffset();
    float err = target - angle;
    float absErr = fabsf(err);
    if (absErr > _maxOvershoot) _maxOvershoot = absErr;

    if (_state == TUNE_STEP_UP) {
      if ((uint32_t)(nowMs - _startMs) >= TUNE_STEP_HOLD_MS) {
        _state = TUNE_STEP_DOWN;
        _startMs = nowMs;
        _settleStartMs = 0;
      }
      return;
    }

    if (_state == TUNE_STEP_DOWN) {
      if (absErr <= TUNE_SETTLE_THRESH) {
        if (_settleStartMs == 0) _settleStartMs = nowMs;
        uint32_t elapsed = nowMs - _settleStartMs;
        if (elapsed >= TUNE_SETTLE_FOR_MS && _settleMs == 0)
          _settleMs = nowMs - _startMs;
      } else {
        _settleStartMs = 0;
      }
      if ((uint32_t)(nowMs - _startMs) >= TUNE_OBSERVE_MS) {
        _state = TUNE_DONE;
        _computeSuggestions();
      }
    }
  }

  void finishToWaitApply() { _state = TUNE_WAIT_APPLY; }

  float getSuggestedKp() const { return _suggestedKp; }
  float getSuggestedKi() const { return _suggestedKi; }
  float getSuggestedKd() const { return _suggestedKd; }
  float getSuggestedLimit() const { return _suggestedLimit; }
  float getMaxOvershoot() const { return _maxOvershoot; }
  uint32_t getSettleMs() const { return _settleMs; }

  void resetState() { _state = TUNE_IDLE; }

private:
  void _computeSuggestions() {
    if (_maxOvershoot > 1.2f) {
      _suggestedKd *= 1.15f;
      _suggestedKp *= 0.95f;
    } else if (_maxOvershoot > 0.8f) {
      _suggestedKd *= 1.08f;
    }
    if (_settleMs > 0 && _settleMs < 600) {
      _suggestedKp *= 1.05f;
    } else if (_settleMs >= 1000) {
      _suggestedKp *= 0.94f;
    }
    _suggestedKp = constrain(_suggestedKp, 50.0f, 500.0f);
    _suggestedKi = constrain(_suggestedKi, 0.001f, 0.02f);
    _suggestedKd = constrain(_suggestedKd, 0.001f, 0.02f);
  }

  PidTuneState _state;
  float _baseTarget;
  uint32_t _startMs;
  uint32_t _settleStartMs;
  float _maxOvershoot;
  uint32_t _settleMs;
  float _suggestedKp, _suggestedKi, _suggestedKd, _suggestedLimit;
};

// EEPROM для PID
void savePidToEEPROM(float kp, float ki, float kd, float limit) {
  PidParams p;
  p.magic = EEPROM_PID_MAGIC;
  p.kp = kp; p.ki = ki; p.kd = kd; p.limit = limit;
  EEPROM.put(EEPROM_PID_ADDR, p);
}

bool loadPidFromEEPROM(PidParams& p) {
  EEPROM.get(EEPROM_PID_ADDR, p);
  return p.magic == EEPROM_PID_MAGIC;
}

void clearPidEEPROM() {
  EEPROM.put(EEPROM_PID_ADDR, (uint16_t)0);
}

#endif
