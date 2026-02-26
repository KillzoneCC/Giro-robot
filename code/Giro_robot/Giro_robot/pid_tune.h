/**
 * Giro-Robot — автотюн PID
 * ========================
 * Мягкое покачивание синусоидой, наблюдение отклика.
 * T — старт, A — применить, R — отклонить.
 */

#ifndef PID_TUNE_H
#define PID_TUNE_H

#include "Arduino.h"
#include "config.h"
#include <EEPROM.h>
#include <math.h>

#define EEPROM_PID_MAGIC    0x5D1D
#define EEPROM_PID_ADDR    64

#define TUNE_WARMUP_MS      2000
#define TUNE_ROCK_AMPL_DEG  0.4f
#define TUNE_ROCK_FREQ_HZ   0.2f
#define TUNE_ROCK_MS        8000
#define TUNE_INIT_KP        40.0f
#define TUNE_INIT_KI        0.0008f
#define TUNE_INIT_KD        0.0008f
#define TUNE_INIT_LIMIT     2500.0f

struct PidParams {
  uint16_t magic;
  float kp, ki, kd, limit;
};

enum PidTuneState {
  TUNE_IDLE,
  TUNE_WARMUP,
  TUNE_ROCKING,
  TUNE_DONE,
  TUNE_WAIT_APPLY
};

class PidTune {
public:
  PidTune() : _state(TUNE_IDLE), _startMs(0), _suggestedKp(0), _suggestedKi(0),
              _suggestedKd(0), _suggestedLimit(0) {}

  void startWarmup(float baseTarget, float kp, float ki, float kd, float limit) {
    _state = TUNE_WARMUP;
    _baseTarget = baseTarget;
    _startMs = millis();
    _suggestedKp = kp;
    _suggestedKi = ki;
    _suggestedKd = kd;
    _suggestedLimit = limit;
  }

  bool isWarmup() const { return _state == TUNE_WARMUP; }
  bool isRunning() const { return _state == TUNE_WARMUP || _state == TUNE_ROCKING; }
  bool isDone() const { return _state == TUNE_DONE; }
  bool isWaitApply() const { return _state == TUNE_WAIT_APPLY; }

  float getTargetOffset(uint32_t nowMs) const {
    if (_state == TUNE_WARMUP) return 0.0f;
    if (_state != TUNE_ROCKING) return 0.0f;
    float t = (nowMs - _startMs) / 1000.0f;
    return TUNE_ROCK_AMPL_DEG * sinf(2.0f * 3.14159f * TUNE_ROCK_FREQ_HZ * t);
  }

  void update(float angle, float dt, uint32_t nowMs) {
    if (_state == TUNE_IDLE || _state == TUNE_WAIT_APPLY) return;

    if (_state == TUNE_WARMUP) {
      if ((uint32_t)(nowMs - _startMs) >= TUNE_WARMUP_MS) {
        _state = TUNE_ROCKING;
        _startMs = nowMs;
      }
      return;
    }

    if (_state == TUNE_ROCKING) {
      if ((uint32_t)(nowMs - _startMs) >= TUNE_ROCK_MS) {
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
  void resetState() { _state = TUNE_IDLE; }

private:
  void _computeSuggestions() {
    _suggestedKp = constrain(_suggestedKp * 1.2f, 80.0f, 400.0f);
    _suggestedKi = constrain(_suggestedKi * 1.5f, 0.002f, 0.02f);
    _suggestedKd = constrain(_suggestedKd * 1.3f, 0.002f, 0.02f);
  }

  PidTuneState _state;
  float _baseTarget;
  uint32_t _startMs;
  float _suggestedKp, _suggestedKi, _suggestedKd, _suggestedLimit;
};

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
