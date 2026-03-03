/**
 * Giro-Robot — PID в EEPROM (Angle + Speed)
 */

#ifndef PID_EEPROM_H
#define PID_EEPROM_H

#include <EEPROM.h>

#define EEPROM_PID_MAGIC       0x5D1D
#define EEPROM_PID_ADDR        64
#define EEPROM_SPEED_PID_MAGIC 0x5D2D
#define EEPROM_SPEED_PID_ADDR  128

struct PidParams {
  uint16_t magic;
  float kp, ki, kd, limit;
};

inline void savePidToEEPROM(float kp, float ki, float kd, float limit) {
  PidParams p;
  p.magic = EEPROM_PID_MAGIC;
  p.kp = kp; p.ki = ki; p.kd = kd; p.limit = limit;
  EEPROM.put(EEPROM_PID_ADDR, p);
}

inline bool loadPidFromEEPROM(PidParams& p) {
  EEPROM.get(EEPROM_PID_ADDR, p);
  return p.magic == EEPROM_PID_MAGIC;
}

inline void saveSpeedPidToEEPROM(float kp, float ki, float kd, float limit) {
  PidParams p;
  p.magic = EEPROM_SPEED_PID_MAGIC;
  p.kp = kp; p.ki = ki; p.kd = kd; p.limit = limit;
  EEPROM.put(EEPROM_SPEED_PID_ADDR, p);
}

inline bool loadSpeedPidFromEEPROM(PidParams& p) {
  EEPROM.get(EEPROM_SPEED_PID_ADDR, p);
  return p.magic == EEPROM_SPEED_PID_MAGIC;
}

inline void clearPidEEPROM() {
  EEPROM.put(EEPROM_PID_ADDR, (uint16_t)0);
  EEPROM.put(EEPROM_SPEED_PID_ADDR, (uint16_t)0);
}

#endif
