/**
 * Giro-Robot — IMU (MPU6050)
 * ==========================
 * Чтение и калибровка. Ориентация — в orientation.h
 */

#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "config.h"

#define GRAVITY_MS2 9.80665f

class ImuSensor {
public:
  bool begin() {
    if (!_mpu.begin()) return false;
    _mpu.setAccelerometerRange(IMU_ACCEL_RANGE);
    _mpu.setGyroRange(IMU_GYRO_RANGE);
    _mpu.setFilterBandwidth(IMU_FILTER_BW);
    return true;
  }

  bool read(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
    sensors_event_t a, g, temp;
    if (!_mpu.getEvent(&a, &g, &temp)) return false;

    ax = a.acceleration.x / GRAVITY_MS2;
    ay = a.acceleration.y / GRAVITY_MS2;
    az = a.acceleration.z / GRAVITY_MS2;

    gx = g.gyro.x * 57.2958f - _gyroBiasX;
    gy = g.gyro.y * 57.2958f - _gyroBiasY;
    gz = g.gyro.z * 57.2958f - _gyroBiasZ;

    if (_calibrated) {
      float rawX = ax * GRAVITY_MS2, rawY = ay * GRAVITY_MS2, rawZ = az * GRAVITY_MS2;
      ax = (rawX - _accelOffX) * _accelScaleX / GRAVITY_MS2;
      ay = (rawY - _accelOffY) * _accelScaleY / GRAVITY_MS2;
      az = (rawZ - _accelOffZ) * _accelScaleZ / GRAVITY_MS2;
    }
    return true;
  }

  void setGyroBias(float gx, float gy, float gz) {
    _gyroBiasX = gx; _gyroBiasY = gy; _gyroBiasZ = gz;
  }
  void setAccelCalibration(float ox, float oy, float oz, float sx, float sy, float sz) {
    _accelOffX = ox; _accelOffY = oy; _accelOffZ = oz;
    _accelScaleX = sx; _accelScaleY = sy; _accelScaleZ = sz;
    _calibrated = true;
  }

  Adafruit_MPU6050* getMpu() { return &_mpu; }

private:
  Adafruit_MPU6050 _mpu;
  float _gyroBiasX, _gyroBiasY, _gyroBiasZ;
  float _accelOffX, _accelOffY, _accelOffZ;
  float _accelScaleX, _accelScaleY, _accelScaleZ;
  bool _calibrated = false;
};

#endif
