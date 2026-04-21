/**
 * Giro-Robot — детекция падения и восстановления
 * ================================================
 * Единый критерий аварии — реальное падение:
 *   |angle - target| > FALL_ANGLE_DEG непрерывно FALL_DEBOUNCE_MS → JUST_FELL.
 * Восстановление — только когда угол вернулся к цели И робот стоит спокойно:
 *   |angle - target| < FALL_RECOVERY_DEG
 *   И |gyroPitchRate| < FALL_RECOVERY_RATE_MAX
 *   непрерывно RECOVERY_DEBOUNCE_MS → JUST_RECOVERED.
 *
 * Все таймеры инкапсулированы — никаких static в loop().
 */

#ifndef SAFETY_H
#define SAFETY_H

#include "Arduino.h"
#include "config.h"
#include <math.h>

enum FallEvent : uint8_t {
  FALL_NO_CHANGE      = 0,
  FALL_JUST_FELL      = 1,
  FALL_JUST_RECOVERED = 2
};

class FallHandler {
public:
  FallHandler()
    : _fallen(false), _fallStartMs(0), _recoverStartMs(0) {}

  /**
   * Обновить состояние.
   *   angle          — текущий pitch (град)
   *   targetOffset   — целевой угол нуля (град)
   *   gyroPitchRate  — угловая скорость по оси pitch (°/с)
   *   nowMs          — millis()
   * Возвращает событие перехода (NO_CHANGE / JUST_FELL / JUST_RECOVERED).
   */
  FallEvent update(float angle, float targetOffset, float gyroPitchRate, uint32_t nowMs) {
    float err = fabsf(angle - targetOffset);

    if (!_fallen) {
      if (err > FALL_ANGLE_DEG) {
        if (_fallStartMs == 0) _fallStartMs = nowMs;
        if ((nowMs - _fallStartMs) >= (uint32_t)FALL_DEBOUNCE_MS) {
          _fallen = true;
          _fallStartMs = 0;
          _recoverStartMs = 0;
          return FALL_JUST_FELL;
        }
      } else {
        _fallStartMs = 0;
      }
      return FALL_NO_CHANGE;
    }

    // _fallen == true — ждём одновременного выполнения обоих условий
    bool nearTarget = (err < FALL_RECOVERY_DEG);
    bool calm       = (fabsf(gyroPitchRate) < FALL_RECOVERY_RATE_MAX);
    if (nearTarget && calm) {
      if (_recoverStartMs == 0) _recoverStartMs = nowMs;
      if ((nowMs - _recoverStartMs) >= (uint32_t)RECOVERY_DEBOUNCE_MS) {
        _fallen = false;
        _fallStartMs = 0;
        _recoverStartMs = 0;
        return FALL_JUST_RECOVERED;
      }
    } else {
      _recoverStartMs = 0;
    }
    return FALL_NO_CHANGE;
  }

  bool isFallen() const { return _fallen; }

  /** Принудительный сброс в NORMAL — например, после `c` калибровки. */
  void forceClear() {
    _fallen = false;
    _fallStartMs = 0;
    _recoverStartMs = 0;
  }

private:
  bool _fallen;
  uint32_t _fallStartMs;
  uint32_t _recoverStartMs;
};

#endif
