/**
 * Giro-Robot — расчёт скорости моторов в м/с
 * ===========================================
 * Пересчёт шагов/с → м/с по диаметру колеса и шагам на оборот.
 * Настройте STEPS_PER_REV и WHEEL_DIAMETER_M под ваши колёса и драйвер.
 */

#ifndef SPEED_MOTOR_H
#define SPEED_MOTOR_H

#include "Arduino.h"
#include <math.h>

// ========== НАСТРОЙКИ (измените под свой робот) ==========
#ifndef STEPS_PER_REV
#define STEPS_PER_REV  3200  // 200×16 микрошагов (TMC2208 microsteps(16))
#endif

#ifndef WHEEL_DIAMETER_M
#define WHEEL_DIAMETER_M  0.078f  // Диаметр колеса в метрах (78 мм)
#endif

#define PI_F  3.14159265358979f

/**
 * Перевести шаги/с в м/с.
 * v = (шаги/с / шаги/об) * π * D
 */
inline float stepsPerSecToMps(float stepsPerSec) {
  if (STEPS_PER_REV <= 0 || WHEEL_DIAMETER_M <= 0.0f) return 0.0f;
  float revPerSec = stepsPerSec / (float)STEPS_PER_REV;
  float circumference = PI_F * WHEEL_DIAMETER_M;
  return revPerSec * circumference;
}

/**
 * Перевести м/с в шаги/с (обратная функция).
 */
inline float mpsToStepsPerSec(float mps) {
  if (WHEEL_DIAMETER_M <= 0.0f) return 0.0f;
  float circumference = PI_F * WHEEL_DIAMETER_M;
  float revPerSec = mps / circumference;
  return revPerSec * (float)STEPS_PER_REV;
}

#endif
