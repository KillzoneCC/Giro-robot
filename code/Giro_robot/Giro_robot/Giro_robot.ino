/**
 * Giro-Robot — основной файл
 * ==========================
 * Только конфигурация, инициализация, вызовы EEPROM и основной цикл.
 * Управление через Serial: v,linear,turn (например v,0.5,0)
 */

#include "config.h"
#include "motors.h"
#include "imu_sensor.h"
#include "pid.h"
#include "stabilizer.h"
#include "twist_control.h"
#include "calibration.h"
#include "fall_handler.h"
#include "pid_tune.h"
#include <Wire.h>

#define SERIAL_BAUD 115200

// ========== Глобальные объекты ==========
ImuSensor imu;
Motors motors;
Stabilizer stabilizer;
TwistControl twist;

// ========== Состояние ==========
CalibData calib;
PidParams pidParams;
PidTune pidTune;
bool isFallen = false;
bool hasCalibration = false;

// ========== ISR для шаговиков ==========
ISR(TIMER1_COMPA_vect) {
  TCNT1 = 0;
  if (_directionMotor1 == 0) return;
  PORTD |= (1 << 3);
  delay_05us();
  PORTD &= ~(1 << 3);
}
ISR(TIMER2_COMPA_vect) {
  TCNT2 = 0;
  if (_directionMotor2 == 0) return;
  PORTB |= (1 << 1);
  delay_05us();
  PORTB &= ~(1 << 1);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  Wire.setClock(400000);

  if (!imu.begin()) {
    Serial.println(F("MPU6050 not found!"));
    while (1) yield();
  }

  motors.begin();

  // PID: из EEPROM или дефолты из config
  if (loadPidFromEEPROM(pidParams)) {
    stabilizer.setMotorPid(pidParams.kp, pidParams.ki, pidParams.kd, pidParams.limit);
    Serial.print(F("PID from EEPROM: Kp=")); Serial.println(pidParams.kp);
  } else {
    pidParams.kp = PID_MOTOR_KP;
    pidParams.ki = PID_MOTOR_KI;
    pidParams.kd = PID_MOTOR_KD;
    pidParams.limit = PID_MOTOR_LIMIT;
  }

  if (loadCalibrationFromEEPROM(calib)) {
    hasCalibration = true;
    imu.setGyroBias(calib.gyroBiasX, calib.gyroBiasY, calib.gyroBiasZ);
    imu.setAccelCalibration(calib.accelOffX, calib.accelOffY, calib.accelOffZ,
                           calib.accelScaleX, calib.accelScaleY, calib.accelScaleZ);
    stabilizer.setTargetAngleOffset(calib.targetAngleOffset);
    imu.setFilterAngle(calib.targetAngleOffset);
    Serial.println(F("Calibration loaded from EEPROM."));
  } else {
    hasCalibration = false;
    Serial.println(F("========================================"));
    Serial.println(F("EEPROM empty. Full calibration required."));
    Serial.println(F("Send 'c' to start: IMU (6 pos) -> basic PID -> tune."));
    Serial.println(F("========================================"));
  }

  stabilizer.reset();
  Serial.println(F("READY. v,linear,turn | c=calib | z=zero | p/i/d/l,val | f,kp,ki,kd,lim | P=print | w=save | e=erase | s=stop"));
}

void processSerial() {
  if (Serial.available() < 1) return;

  char cmd = Serial.read();
  if (cmd == 'v' || cmd == 'V') {
    while (Serial.available() < 5) { delay(1); }  // ждём "0.5,0" или "-1,1"
    float linear = Serial.parseFloat();
    float turn = Serial.parseFloat();
    twist.setTwist(linear, turn);
  } else if (cmd == 's' || cmd == 'S') {
    twist.stop();
  } else if (cmd == 'c' || cmd == 'C') {
    bool wasEmpty = !hasCalibration;
    Serial.println(F("=== Full IMU calibration (6 positions) ==="));
    if (runFullCalibration(imu.getMpu(), calib)) {
      hasCalibration = true;
      imu.setGyroBias(calib.gyroBiasX, calib.gyroBiasY, calib.gyroBiasZ);
      imu.setAccelCalibration(calib.accelOffX, calib.accelOffY, calib.accelOffZ,
                              calib.accelScaleX, calib.accelScaleY, calib.accelScaleZ);
      stabilizer.setTargetAngleOffset(calib.targetAngleOffset);
      imu.setFilterAngle(calib.targetAngleOffset);
      stabilizer.reset();
      if (wasEmpty) {
        pidParams.kp = PID_MOTOR_KP;
        pidParams.ki = PID_MOTOR_KI;
        pidParams.kd = PID_MOTOR_KD;
        pidParams.limit = PID_MOTOR_LIMIT;
        stabilizer.setMotorPid(pidParams.kp, pidParams.ki, pidParams.kd, pidParams.limit);
        Serial.println(F("=== Basic PID set from config ==="));
        Serial.println(F("Tune: p,val i,val d,val l,val | P=print | w=save to EEPROM"));
      }
    }
  } else if ((cmd == 'z' || cmd == 'Z') && hasCalibration) {
    Serial.println(F("Memorize zero: hold upright 2 sec..."));
    uint32_t t0 = millis();
    float sumA = 0;
    int cnt = 0;
    while (millis() - t0 < 2000) {
      float ax, ay, az, gxr, gyr, gzr;
      if (imu.read(ax, ay, az, gxr, gyr, gzr)) {
#if USE_PITCH_AXIS
        float accA = atan2f(-ax, sqrtf(ay*ay + az*az + 0.001f)) * 57.2958f;
        sumA += imu.updateFilter(accA, gyr, 0.01f);
#else
        float accA = atan2f(ay, sqrtf(ax*ax + az*az + 0.001f)) * 57.2958f;
        sumA += imu.updateFilter(accA, gxr, 0.01f);
#endif
        cnt++;
      }
      delay(10);
    }
    if (cnt > 0) {
      float off = sumA / cnt;
      stabilizer.setTargetAngleOffset(off);
      imu.setFilterAngle(off);
      calib.targetAngleOffset = off;
      calib.magic = EEPROM_MAGIC;
      saveCalibrationToEEPROM(calib);
      Serial.print(F("Zero: ")); Serial.println(off);
    }
  } else if (cmd == 'z' || cmd == 'Z') {
    if (!hasCalibration) Serial.println(F("Calibrate first (c)."));
  } else if (cmd == 'e' || cmd == 'E') {
    clearCalibrationEEPROM();
    clearPidEEPROM();
    hasCalibration = false;
    Serial.println(F("EEPROM cleared. Send 'c' for full calibration."));
  } else if (cmd == 'P') {
    Serial.print(F("PID: Kp=")); Serial.print(pidParams.kp);
    Serial.print(F(" Ki=")); Serial.print(pidParams.ki);
    Serial.print(F(" Kd=")); Serial.print(pidParams.kd);
    Serial.print(F(" limit=")); Serial.println(pidParams.limit);
  } else if (cmd == 'p') {
    delay(30);
    if (Serial.available() >= 1) {
      float v = Serial.parseFloat();
      if (v > 0) { pidParams.kp = v; stabilizer.setMotorPid(pidParams.kp, pidParams.ki, pidParams.kd, pidParams.limit); Serial.print(F("Kp=")); Serial.println(v); }
    }
  } else if (cmd == 'i') {
    delay(30);
    if (Serial.available() >= 1) {
      float v = Serial.parseFloat();
      if (v >= 0) { pidParams.ki = v; stabilizer.setMotorPid(pidParams.kp, pidParams.ki, pidParams.kd, pidParams.limit); Serial.print(F("Ki=")); Serial.println(v); }
    }
  } else if (cmd == 'd') {
    delay(30);
    if (Serial.available() >= 1) {
      float v = Serial.parseFloat();
      if (v >= 0) { pidParams.kd = v; stabilizer.setMotorPid(pidParams.kp, pidParams.ki, pidParams.kd, pidParams.limit); Serial.print(F("Kd=")); Serial.println(v); }
    }
  } else if (cmd == 'l') {
    delay(30);
    if (Serial.available() >= 1) {
      float v = Serial.parseFloat();
      if (v > 0) { pidParams.limit = v; stabilizer.setMotorPid(pidParams.kp, pidParams.ki, pidParams.kd, pidParams.limit); Serial.print(F("limit=")); Serial.println(v); }
    }
  } else if (cmd == 'w' || cmd == 'W') {
    savePidToEEPROM(pidParams.kp, pidParams.ki, pidParams.kd, pidParams.limit);
    Serial.println(F("PID saved to EEPROM."));
  } else if (cmd == 'f' || cmd == 'F') {
    // f,kp,ki,kd,limit — установить все коэффициенты сразу
    delay(50);
    if (Serial.available() >= 5) {
      float kp = Serial.parseFloat();
      float ki = Serial.parseFloat();
      float kd = Serial.parseFloat();
      float lim = Serial.parseFloat();
      if (kp > 0 && ki >= 0 && kd >= 0 && lim > 0) {
        pidParams.kp = kp; pidParams.ki = ki; pidParams.kd = kd; pidParams.limit = lim;
        stabilizer.setMotorPid(pidParams.kp, pidParams.ki, pidParams.kd, pidParams.limit);
        Serial.print(F("PID: ")); Serial.print(kp); Serial.print(','); Serial.print(ki);
        Serial.print(','); Serial.print(kd); Serial.print(','); Serial.println(lim);
      }
    }
  }
}

void loop() {
  processSerial();

  if (!hasCalibration) {
    motors.stop();
    static uint32_t lastMsg = 0;
    if (millis() - lastMsg > 2000) {
      lastMsg = millis();
      Serial.println(F("Waiting for 'c' (full calibration)..."));
    }
    delay(100);
    return;
  }

  float ax, ay, az, gx, gy, gz;
  if (!imu.read(ax, ay, az, gx, gy, gz)) return;

#if USE_PITCH_AXIS
  float accAngle = atan2f(-ax, sqrtf(ay*ay + az*az + 0.001f)) * 57.2958f;
  float gyroRate = gy;
#else
  float accAngle = atan2f(ay, sqrtf(ax*ax + az*az + 0.001f)) * 57.2958f;
  float gyroRate = gx;
#endif

  float angle = imu.updateFilter(accAngle, gyroRate, CONTROL_DT);
  static float angleSmoothed = 0;
  angleSmoothed = angle * 0.7f + angleSmoothed * 0.3f;

  float targetOffset = stabilizer.getTargetAngleOffset();

  // Обнаружение падения — не продолжаем ехать при критическом угле
  if (isFallenCheck(angleSmoothed, targetOffset)) {
    if (!isFallen) {
      isFallen = true;
      stabilizer.reset();
      motors.stop();
      motors.disable();
    }
  } else if (isFallen && canRecover(angleSmoothed, targetOffset)) {
    isFallen = false;
    stabilizer.reset();
  }

  if (isFallen) {
    motors.stop();
  } else {
    float linear = twist.getLinear();
    float turn = twist.getTurn();
    float motorSpeed = stabilizer.update(linear, angleSmoothed, CONTROL_DT);

    // Черновик автоподъёма: boost при сильном наклоне (20..45°)
    motorSpeed += autoRaiseBoost(angleSmoothed, targetOffset, 500.0f);

    // baseNorm [-1..1] -> steps/s. Масштаб = pidParams.limit (как max motorSpeed)
    float baseNorm = motorSpeed / pidParams.limit;
    float leftNorm = baseNorm - turn;
    float rightNorm = baseNorm + turn;
    float m = fmaxf(fmaxf(fabsf(leftNorm), fabsf(rightNorm)), 0.001f);
    if (m > 1.0f) { leftNorm /= m; rightNorm /= m; }
    int16_t leftSteps = (int16_t)(leftNorm * pidParams.limit);
    int16_t rightSteps = (int16_t)(rightNorm * pidParams.limit);
    motors.setLeftRight(leftSteps, rightSteps);
    motors.enable();
  }

  // Отладка
#if DEBUG_ENABLED
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > DEBUG_PRINT_MS) {
    lastPrint = millis();
    Serial.print(F("a:")); Serial.print(angleSmoothed);
    Serial.print(F(" t:")); Serial.print(targetOffset);
    Serial.print(F(" m:")); Serial.print(stabilizer.getMotorSpeed());
    Serial.print(F(" L:")); Serial.print(twist.getLinear());
    Serial.print(F(" T:")); Serial.print(twist.getTurn());
    if (isFallen) Serial.print(F(" FALL!"));
    Serial.println();
  }
#endif

  delay((int)(1000.0f / CONTROL_LOOP_HZ));
}
