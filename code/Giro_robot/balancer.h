/*
  Логика баланса робота.
  Вынесено из balansing_robot.ino
  Каскад ПИД: скорость -> целевой угол -> управление моторами по углу наклона.
  Подключено к fgggggggggggg3.ino: используются pidbalanse.h и адаптер MPU.
*/
#ifndef SERIAL_BAUD
#define SERIAL_BAUD 115200
#endif

#ifndef Balancer_h
#define Balancer_h

#include "Arduino.h"
#include "pidbalanse.h"
#include <math.h>

// Константы (раньше в config.h)
#ifndef TARGET_DELTA_TIME
#define TARGET_DELTA_TIME 0.005f
#endif
#ifndef Kp_s
#define Kp_s 40.0f
#endif
#ifndef Ki_s
#define Ki_s 0.0f
#endif
#ifndef Kd_s
#define Kd_s 0.5f
#endif
#ifndef limit_s
#define limit_s 30.0f
#endif
#ifndef Kp_a
#define Kp_a 100.0f
#endif
#ifndef Ki_a
#define Ki_a 0.0f
#endif
#ifndef Kd_a
#define Kd_a 1.0f
#endif
#ifndef limit_a
#define limit_a 500.0f
#endif

// Комплементарный фильтр (acc + gyro) — как в balansing_robot
// angle = alpha * (angle + gyro_dps*dt) + (1-alpha) * accAngle
class ComplementaryFilter
{
public:
  ComplementaryFilter(float alpha = 0.995f) : _alpha(alpha), _angle(0.0f) {}
  // accAngle, gyroRate_dps — в градусах, dt в секундах
  float calculate(float accAngle, float gyroRate_dps, float dt)
  {
    _angle = _alpha * (_angle + gyroRate_dps * dt) + (1.0f - _alpha) * accAngle;
    return _angle;
  }
  void resetValues() { _angle = 0.0f; }
  void setAngle(float a) { _angle = a; }
private:
  float _alpha;
  float _angle;
};

// Адаптер данных MPU для использования с Adafruit_MPU6050 (данные задаются из .ino)
class Mpu6050
{
public:
  void setAccelGyro(float ax, float ay, float az, float gx, float gy, float gz)
  {
    _ax = ax; _ay = ay; _az = az;
    _gx = gx; _gy = gy; _gz = gz;
  }
  float getAccelX() const { return _ax; }
  float getAccelY() const { return _ay; }
  float getAccelZ() const { return _az; }
  float getGyroX() const { return _gx; }
  float getGyroY() const { return _gy; }
  float getGyroZ() const { return _gz; }
private:
  float _ax, _ay, _az, _gx, _gy, _gz;
};

class BalanceController
{
public:
  BalanceController()
    : _pidAngle(Kp_s, Ki_s, Kd_s, limit_s),
      _pidMotor(Kp_a, Ki_a, Kd_a, limit_a),
      _zero(-2.3f),
      _currentLeanAngle(0.0f),
      _motorSpeed(0.0f),
      _angle(0.0f),
      _targetSpeed(0),
      _leftMotorSpeed(0),
      _rightMotorSpeed(0)
  {
  }

  // Обновить логику баланса по данным IMU. Вызывать после imu.updateIMUdata()
  // gyroY должен быть в deg/s (если в rad/s — умножить на 57.3)
  void update(Mpu6050& imu)
  {
    float accX = imu.getAccelX();
    float accY = imu.getAccelY();
    float accZ = imu.getAccelZ();
    float accAngle = atan2f(accY, sqrtf(accX*accX + accZ*accZ)) * 57.29577951308232f;

    float gyroY_dps = imu.getGyroY();
    _angle = _angleFilter.calculate(accAngle, gyroY_dps, TARGET_DELTA_TIME);

    // Сглаживание текущего угла наклона
    _currentLeanAngle = _angle * 0.7f + _currentLeanAngle * 0.3f;

    // Каскад ПИД:
    // 1) ПИД угла: целевая скорость -> целевой угол наклона
    float targetAngle = _pidAngle.updatePID(_targetSpeed, _motorSpeed, TARGET_DELTA_TIME) + _zero;

    // 2) ПИД мотора: целевой угол -> скорость моторов (шаги/сек)
    _motorSpeed = -_pidMotor.updatePID(targetAngle, _currentLeanAngle, TARGET_DELTA_TIME);

    // Оба мотора с одинаковой скоростью (поворот пока не учтён)
    _leftMotorSpeed = (int16_t)_motorSpeed;
    _rightMotorSpeed = (int16_t)_motorSpeed;
  }

  // Скорости моторов после вызова update() (шаги/сек)
  int16_t getLeftMotorSpeed() const { return _leftMotorSpeed; }
  int16_t getRightMotorSpeed() const { return _rightMotorSpeed; }

  // Установка целевой линейной скорости
  void setTargetSpeed(int speed) { _targetSpeed = speed; }
  int getTargetSpeed() const { return _targetSpeed; }

  // Смещение нулевого угла (калибровка)
  void setZero(float zero) { _zero = zero; }
  float getZero() const { return _zero; }

  // Текущий угол наклона (градусы)
  float getCurrentLeanAngle() const { return _currentLeanAngle; }
  float getMotorSpeed() const { return _motorSpeed; }

  void reset()
  {
    _angleFilter.resetValues();
    _pidAngle.resetPID();
    _pidMotor.resetPID();
    _currentLeanAngle = 0.0f;
    _motorSpeed = 0.0f;
    _leftMotorSpeed = 0;
    _rightMotorSpeed = 0;
  }

private:
  ComplementaryFilter _angleFilter;
  Pid _pidAngle;   // ПИД угла (pid_s): targetSpeed -> targetAngle
  Pid _pidMotor;   // ПИД мотора (pid_a): targetAngle -> motorSpeed

  float _zero;
  float _currentLeanAngle;
  float _motorSpeed;
  float _angle;           // угол с комплементарного фильтра
  int _targetSpeed;

  int16_t _leftMotorSpeed;
  int16_t _rightMotorSpeed;
};

#endif
