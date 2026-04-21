/**
 * Giro-Robot — контроллер скорости (внешний контур)
 * ==================================================
 * Speed PID: target_speed, current_speed → target_angle.
 * Выходной угол передаётся во внутренний контур (стабилизатор по углу).
 *
 * Логика:
 *   target > current → наклониться вперёд (положительный угол)
 *   target < current → наклониться назад (торможение)
 *   target = 0       → угол = 0 (держать баланс без движения)
 *
 * updateSmoothed() дополнительно:
 *   - при смене знака целевой скорости мгновенно сбрасывает PID и переход к новому
 *     выходу без EMA (иначе накопленное состояние мешает развороту);
 *   - при запуске из балансировки на месте (prev=0 → target!=0) тоже мгновенный старт;
 *   - иначе — экспоненциальное сглаживание (ANGLE_OFFSET_SMOOTH).
 * Всё накопленное состояние живёт ВНУТРИ класса и очищается через reset().
 */

#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

#include "Arduino.h"
#include "config.h"
#include "pid.h"
#include <math.h>

class SpeedController {
public:
  SpeedController()
    : _pid(SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, SPEED_PID_ANGLE_LIMIT),
      _angleOutput(0),
      _smoothOffset(0),
      _prevTargetSign(0.0f) {}

  /**
   * Сырой выход PID без сглаживания.
   * targetSpeed, currentSpeed — м/с, dt — с. Возвращает угол (град).
   */
  float update(float targetSpeed, float currentSpeed, float dt) {
    _angleOutput = _pid.update(targetSpeed, currentSpeed, dt);
    return _angleOutput;
  }

  /**
   * Сглаженный выход с обработкой смены направления.
   * Возвращает угол (град) — готовый к сложению с targetOffset.
   */
  float updateSmoothed(float targetSpeed, float currentSpeed, float dt) {
    float s = (targetSpeed >  0.02f) ?  1.0f
            : (targetSpeed < -0.02f) ? -1.0f
            :                           0.0f;
    bool signChanged = (_prevTargetSign != 0.0f && s != 0.0f && _prevTargetSign != s);
    bool fromBalance = (_prevTargetSign == 0.0f && s != 0.0f);
    if (signChanged) _pid.reset();
    _prevTargetSign = s;

    float rawOffset = _pid.update(targetSpeed, currentSpeed, dt);
    if (signChanged || fromBalance) {
      _smoothOffset = rawOffset;
    } else {
      _smoothOffset = ANGLE_OFFSET_SMOOTH * rawOffset
                    + (1.0f - ANGLE_OFFSET_SMOOTH) * _smoothOffset;
    }
    _angleOutput = _smoothOffset;
    return _smoothOffset;
  }

  void setPid(float kp, float ki, float kd, float angleLimit) {
    _pid.setKp(kp);
    _pid.setKi(ki);
    _pid.setKd(kd);
    _pid.setLimit(fabsf(angleLimit));
  }

  /** Полный сброс: PID, сглаживание, признак направления. */
  void reset() {
    _pid.reset();
    _angleOutput = 0;
    _smoothOffset = 0;
    _prevTargetSign = 0.0f;
  }

  float getAngleOutput() const { return _angleOutput; }
  float getPidP() const { return _pid.getLastP(); }
  float getPidI() const { return _pid.getLastI(); }
  float getPidD() const { return _pid.getLastD(); }
  float getPidError() const { return _pid.getLastError(); }

private:
  Pid _pid;
  float _angleOutput;
  float _smoothOffset;
  float _prevTargetSign;
};

#endif
