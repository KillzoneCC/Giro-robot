/**
 * Giro-Robot — модуль датчика IMU (MPU6050)
 * =========================================
 * Обёртка над Adafruit MPU6050. Чтение, калибровка, комплементарный фильтр.
 */

#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "config.h"

#define RAD_TO_DEG  57.29577951308232f
#define GRAVITY_MS2 9.80665f

class ImuSensor {
public:
  ImuSensor() : _filterAngle(0), _alpha(0.995f) {}

  bool begin() {
    if (!_mpu.begin()) return false;
    _mpu.setAccelerometerRange(IMU_ACCEL_RANGE);
    _mpu.setGyroRange(IMU_GYRO_RANGE);
    _mpu.setFilterBandwidth(IMU_FILTER_BW);
    return true;
  }

  /** Сырые данные (после вычета bias гироскопа) */
  bool read(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
    sensors_event_t a, g, temp;
    if (!_mpu.getEvent(&a, &g, &temp)) return false;

    ax = a.acceleration.x / GRAVITY_MS2;
    ay = a.acceleration.y / GRAVITY_MS2;
    az = a.acceleration.z / GRAVITY_MS2;

    gx = g.gyro.x * RAD_TO_DEG - _gyroBiasX;
    gy = g.gyro.y * RAD_TO_DEG - _gyroBiasY;
    gz = g.gyro.z * RAD_TO_DEG - _gyroBiasZ;

    // Калибровка акселерометра (offsets в m/s²)
    if (_calibrated) {
      float rawX = ax * GRAVITY_MS2, rawY = ay * GRAVITY_MS2, rawZ = az * GRAVITY_MS2;
      ax = (rawX - _accelOffX) * _accelScaleX / GRAVITY_MS2;
      ay = (rawY - _accelOffY) * _accelScaleY / GRAVITY_MS2;
      az = (rawZ - _accelOffZ) * _accelScaleZ / GRAVITY_MS2;
    }
    return true;
  }

  /**
   * Угол баланса (roll или pitch) через комплементарный фильтр.
   * accAngle, gyroRate — в градусах, dt — в секундах.
   */
  float updateFilter(float accAngle, float gyroRate, float dt) {
    _filterAngle = _alpha * (_filterAngle + gyroRate * dt) + (1.0f - _alpha) * accAngle;
    return _filterAngle;
  }

  void setFilterAngle(float a) { _filterAngle = a; }
  float getFilterAngle() const { return _filterAngle; }

  // Калибровка (bias гироскопа)
  void setGyroBias(float gx, float gy, float gz) {
    _gyroBiasX = gx; _gyroBiasY = gy; _gyroBiasZ = gz;
  }
  void getGyroBias(float& gx, float& gy, float& gz) const {
    gx = _gyroBiasX; gy = _gyroBiasY; gz = _gyroBiasZ;
  }

  // Калибровка акселерометра (6-point)
  void setAccelCalibration(float ox, float oy, float oz, float sx, float sy, float sz) {
    _accelOffX = ox; _accelOffY = oy; _accelOffZ = oz;
    _accelScaleX = sx; _accelScaleY = sy; _accelScaleZ = sz;
    _calibrated = true;
  }
  void clearAccelCalibration() { _calibrated = false; }
  bool hasAccelCalibration() const { return _calibrated; }

  Adafruit_MPU6050* getMpu() { return &_mpu; }

private:
  Adafruit_MPU6050 _mpu;
  float _filterAngle;
  float _alpha;
  float _gyroBiasX, _gyroBiasY, _gyroBiasZ;
  float _accelOffX, _accelOffY, _accelOffZ;
  float _accelScaleX, _accelScaleY, _accelScaleZ;
  bool _calibrated;
};

#endif
