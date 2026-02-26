/**
 * Giro-Robot — обработка падений
 * ===============================
 * Остановка при критическом угле, черновик автоподъёма.
 */

#ifndef FALL_HANDLER_H
#define FALL_HANDLER_H

#include "config.h"

/** Проверка: робот упал? (угол отклонения > порога) */
inline bool isFallenCheck(float angle, float targetOffset) {
  return fabsf(angle - targetOffset) > FALL_ANGLE_DEG;
}

/** Можно ли восстановиться? (угол вернулся в норму) */
inline bool canRecover(float angle, float targetOffset) {
  return fabsf(angle - targetOffset) < FALL_RECOVERY_DEG;
}

/**
 * Черновик автоподъёма: дополнительная корректирующая скорость при сильном наклоне.
 * Когда угол 20..45° — добавляем "подталкивание" в сторону вертикали.
 * Коэффициент подбирается экспериментально.
 */
inline float autoRaiseBoost(float angle, float targetOffset, float maxBoost) {
  float err = targetOffset - angle;
  float absErr = fabsf(err);
  if (absErr < FALL_RECOVERY_DEG || absErr > FALL_ANGLE_DEG)
    return 0.0f;
  // Зона 20..45°: линейно от 0 до maxBoost
  float t = (absErr - FALL_RECOVERY_DEG) / (FALL_ANGLE_DEG - FALL_RECOVERY_DEG);
  return (err > 0 ? 1.0f : -1.0f) * t * maxBoost;
}

#endif
