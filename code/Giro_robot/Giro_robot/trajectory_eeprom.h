/**
 * Giro-Robot — траектория в EEPROM
 * ================================
 * Сохранение последовательности speed,duration,turn.
 * При включении робот автоматически выполняет сохранённую траекторию.
 * turn — зарезервировано на будущее (пока не используется).
 */

#ifndef TRAJECTORY_EEPROM_H
#define TRAJECTORY_EEPROM_H

#include <EEPROM.h>

#define EEPROM_TRAJ_MAGIC  0x5D3D
#define EEPROM_TRAJ_ADDR   256
#define TRAJ_MAX_STEPS     12

struct TrajStep {
  float speed;
  float duration;
  float turn;  // зарезервировано
};

struct TrajData {
  uint16_t magic;
  uint8_t count;
  TrajStep steps[TRAJ_MAX_STEPS];
};

inline bool loadTrajectoryFromEEPROM(TrajData& t) {
  EEPROM.get(EEPROM_TRAJ_ADDR, t);
  return t.magic == EEPROM_TRAJ_MAGIC && t.count > 0 && t.count <= TRAJ_MAX_STEPS;
}

inline void saveTrajectoryToEEPROM(const TrajData& t) {
  EEPROM.put(EEPROM_TRAJ_ADDR, t);
}

inline void clearTrajectoryEEPROM() {
  EEPROM.put(EEPROM_TRAJ_ADDR, (uint16_t)0);
}

#endif
