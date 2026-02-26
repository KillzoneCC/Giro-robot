/**
 * Giro-Robot — PID в EEPROM
 * =========================
 */

#ifndef PID_EEPROM_H
#define PID_EEPROM_H

#include <EEPROM.h>

#define EEPROM_PID_MAGIC  0x5D1D
#define EEPROM_PID_ADDR  64

struct PidParams {
  uint16_t magic;
  float kp, ki, kd, limit;
};

inline void savePidToEEPROM(float kp, float ki, float kd, float limit) {
  PidParams p;
  p.magic = EEPROM_PID_MAGIC;
  p.kp = kp;
  p.ki = ki;
  p.kd = kd;
  p.limit = limit;
  EEPROM.put(EEPROM_PID_ADDR, p);
}

inline bool loadPidFromEEPROM(PidParams& p) {
  EEPROM.get(EEPROM_PID_ADDR, p);
  return p.magic == EEPROM_PID_MAGIC;
}

inline void clearPidEEPROM() {
  EEPROM.put(EEPROM_PID_ADDR, (uint16_t)0);
}

#endif
