/**
 * Giro-Robot — падение
 * ====================
 */

#ifndef FALL_HANDLER_H
#define FALL_HANDLER_H

#include "config.h"

inline bool isFallenCheck(float angle, float targetOffset) {
  return fabsf(angle - targetOffset) > FALL_ANGLE_DEG;
}

inline bool canRecover(float angle, float targetOffset) {
  return fabsf(angle - targetOffset) < FALL_RECOVERY_DEG;
}

#endif
