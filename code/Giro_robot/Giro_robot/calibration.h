/**
 * Giro-Robot — калибровка IMU
 * ===========================
 * 6-точечная калибровка гироскопа и акселерометра.
 * Сохранение в EEPROM, загрузка при старте.
 * Первая позиция — робот стоит ВЕРТИКАЛЬНО (нулевое положение).
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <EEPROM.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "Arduino.h"
#include "config.h"

#define EEPROM_MAGIC       0xCA1B
#define EEPROM_CALIB_ADDR  0
#define CALIB_SAMPLES      600
#define FLAT_TILT_MAX_DEG   5.0f
#define RAD_TO_DEG          57.29577951308232f
#define GRAVITY_MS2         9.80665f

struct CalibData {
  uint16_t magic;
  float gyroBiasX, gyroBiasY, gyroBiasZ;
  float accelOffX, accelOffY, accelOffZ;
  float accelScaleX, accelScaleY, accelScaleZ;
  float targetAngleOffset;
};

void saveCalibrationToEEPROM(const CalibData& d) {
  EEPROM.put(EEPROM_CALIB_ADDR, d);
}

bool loadCalibrationFromEEPROM(CalibData& d) {
  EEPROM.get(EEPROM_CALIB_ADDR, d);
  return d.magic == EEPROM_MAGIC;
}

void clearCalibrationEEPROM() {
  EEPROM.put(EEPROM_CALIB_ADDR, (uint16_t)0);
}

const char* getCalibPositionName(int i) {
  static const char* names[] = {
    "ВЕРТИКАЛЬНО (колёса вниз)",
    "На спине (колёса вверх)",
    "На ЛЕВОМ боку",
    "На ПРАВОМ боку",
    "Нос вверх",
    "Нос вниз"
  };
  return (i >= 0 && i < 6) ? names[i] : "?";
}

/**
 * Полная 6-позиционная калибровка.
 * Позиция 0 — робот ВЕРТИКАЛЬНО! Это и есть targetAngleOffset.
 * Заполняет CalibData и сохраняет в EEPROM.
 */
bool runFullCalibration(Adafruit_MPU6050* mpu, CalibData& out) {
  if (!mpu) return false;

  float accMeans[6][3], gyroMeans[6][3];

  Serial.println(F("\n=== КАЛИБРОВКА 6 ПОЗИЦИЙ ==="));
  Serial.println(F("Позиция 1: ВЕРТИКАЛЬНО. Через 5 сек - старт."));
  delay(5000);

  for (int pos = 0; pos < 6; pos++) {
    Serial.println();
    Serial.print(F("Позиция ")); Serial.print(pos + 1); Serial.print(F(" из 6: "));
    Serial.println(getCalibPositionName(pos));
    Serial.println(F("Обратный отсчёт 5 сек..."));
    for (int t = 5; t >= 1; t--) {
      Serial.print(t); Serial.println("...");
      delay(1000);
    }
    Serial.println(F("Сбор данных..."));

    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    uint32_t count = 0;

    for (uint32_t i = 0; i < CALIB_SAMPLES; i++) {
      sensors_event_t a, g, temp;
      if (mpu->getEvent(&a, &g, &temp)) {
        sum_ax += a.acceleration.x;
        sum_ay += a.acceleration.y;
        sum_az += a.acceleration.z;
        sum_gx += g.gyro.x * RAD_TO_DEG;
        sum_gy += g.gyro.y * RAD_TO_DEG;
        sum_gz += g.gyro.z * RAD_TO_DEG;
        count++;
      }
      delay(3);
    }

    if (count == 0) {
      Serial.println(F("Ошибка чтения!"));
      return false;
    }

    accMeans[pos][0] = sum_ax / count;
    accMeans[pos][1] = sum_ay / count;
    accMeans[pos][2] = sum_az / count;
    gyroMeans[pos][0] = sum_gx / count;
    gyroMeans[pos][1] = sum_gy / count;
    gyroMeans[pos][2] = sum_gz / count;

    if (pos == 0) {
      float horiz = sqrtf(accMeans[0][0]*accMeans[0][0] + accMeans[0][1]*accMeans[0][1]);
      float vert = fabsf(accMeans[0][2]);
      float tilt = (vert > 0.1f) ? (atan2f(horiz, vert) * RAD_TO_DEG) : 90.0f;
      if (tilt > FLAT_TILT_MAX_DEG) {
        Serial.print(F("!!! Наклон ")); Serial.print(tilt, 1); Serial.println(F(" град. Поставьте ровнее!"));
        delay(5000);
        pos--;
        continue;
      }
      Serial.print(F("Поверхность ровная (")); Serial.print(tilt, 1); Serial.println(F(" град.)"));
    }
    Serial.println(F("OK"));
    if (pos < 5) { Serial.println(F(">>> Следующая позиция <<<")); delay(2000); }
  }

  // Вычисление калибровки
  out.gyroBiasX = out.gyroBiasY = out.gyroBiasZ = 0;
  for (int i = 0; i < 6; i++) {
    out.gyroBiasX += gyroMeans[i][0];
    out.gyroBiasY += gyroMeans[i][1];
    out.gyroBiasZ += gyroMeans[i][2];
  }
  out.gyroBiasX /= 6.0f;
  out.gyroBiasY /= 6.0f;
  out.gyroBiasZ /= 6.0f;

  out.accelOffX = (accMeans[2][0] + accMeans[3][0]) / 2.0f;
  out.accelOffY = (accMeans[4][1] + accMeans[5][1]) / 2.0f;
  out.accelOffZ = (accMeans[0][2] + accMeans[1][2]) / 2.0f;

  float rx = accMeans[2][0] - accMeans[3][0];
  float ry = accMeans[4][1] - accMeans[5][1];
  float rz = accMeans[0][2] - accMeans[1][2];
  out.accelScaleX = (fabsf(rx) > 0.1f) ? (2.0f * GRAVITY_MS2 / rx) : 1.0f;
  out.accelScaleY = (fabsf(ry) > 0.1f) ? (2.0f * GRAVITY_MS2 / ry) : 1.0f;
  out.accelScaleZ = (fabsf(rz) > 0.1f) ? (2.0f * GRAVITY_MS2 / rz) : 1.0f;

  // targetAngleOffset для позиции 0 (вертикально): угол из акселерометра
  float ax = (accMeans[0][0] - out.accelOffX) * out.accelScaleX / GRAVITY_MS2;
  float ay = (accMeans[0][1] - out.accelOffY) * out.accelScaleY / GRAVITY_MS2;
  float az = (accMeans[0][2] - out.accelOffZ) * out.accelScaleZ / GRAVITY_MS2;
#if USE_PITCH_AXIS
  out.targetAngleOffset = atan2f(-ax, sqrtf(ay*ay + az*az + 0.001f)) * RAD_TO_DEG;
#else
  out.targetAngleOffset = atan2f(ay, sqrtf(ax*ax + az*az + 0.001f)) * RAD_TO_DEG;
#endif

  out.magic = EEPROM_MAGIC;
  saveCalibrationToEEPROM(out);

  Serial.println(F("\n--- Калибровка завершена ---"));
  Serial.print(F("targetAngleOffset: ")); Serial.println(out.targetAngleOffset);
  Serial.println(F("Сохранено в EEPROM."));

  return true;
}

#endif
