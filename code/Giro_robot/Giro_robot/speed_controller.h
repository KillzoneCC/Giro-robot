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
      _angleOutput(0) {}

  /**
   * Обновить контроллер скорости.
   * targetSpeed, currentSpeed — м/с, dt — с.
   * Возвращает угол (град) — смещение от нуля для достижения целевой скорости.
   */
  float update(float targetSpeed, float currentSpeed, float dt) {
    _angleOutput = _pid.update(targetSpeed, currentSpeed, dt);
    return _angleOutput;
  }

  void setPid(float kp, float ki, float kd, float angleLimit) {
    _pid.setKp(kp);
    _pid.setKi(ki);
    _pid.setKd(kd);
    _pid.setLimit(fabsf(angleLimit));
  }

  void reset() {
    _pid.reset();
    _angleOutput = 0;
  }

  float getAngleOutput() const { return _angleOutput; }
  float getPidP() const { return _pid.getLastP(); }
  float getPidI() const { return _pid.getLastI(); }
  float getPidD() const { return _pid.getLastD(); }
  float getPidError() const { return _pid.getLastError(); }

private:
  Pid _pid;
  float _angleOutput;
};

#endif
