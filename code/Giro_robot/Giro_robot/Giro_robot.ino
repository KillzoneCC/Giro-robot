/**
 * Giro-Robot — балансирующий робот
 * ================================
 * Ориентация по pitch. Один PID. Управление: v,linear,turn
 */

#include "config.h"
#include "motors.h"
#include "imu_sensor.h"
#include "orientation.h"
#include "pid.h"
#include "stabilizer.h"
#include "twist_control.h"
#include "calibration.h"
#include "fall_handler.h"
#include "pid_eeprom.h"
#include <Wire.h>

#define SERIAL_BAUD 115200

ImuSensor imu;
Orientation orientation;
Motors motors;
Stabilizer stabilizer;
TwistControl twist;

CalibData calib;
PidParams pidParams;
bool isFallen = false;
bool hasCalibration = false;
bool stabilizationEnabled = true;
bool debugEnabled = DEBUG_ENABLED;

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
  Serial.setTimeout(5);
  Wire.begin();
  Wire.setClock(400000);

  if (!imu.begin()) {
    Serial.println(F("MPU6050 not found!"));
    while (1) yield();
  }
  motors.begin();

  if (loadPidFromEEPROM(pidParams)) {
    stabilizer.setPid(pidParams.kp, pidParams.ki, pidParams.kd, pidParams.limit);
    Serial.print(F("PID from EEPROM: Kp=")); Serial.print(pidParams.kp);
    Serial.print(F(" Ki=")); Serial.print(pidParams.ki);
    Serial.print(F(" Kd=")); Serial.println(pidParams.kd);
  } else {
    pidParams.kp = PID_KP;
    pidParams.ki = PID_KI;
    pidParams.kd = PID_KD;
    pidParams.limit = PID_LIMIT;
    stabilizer.setPid(pidParams.kp, pidParams.ki, pidParams.kd, pidParams.limit);
  }

  if (loadCalibrationFromEEPROM(calib)) {
    hasCalibration = true;
    imu.setGyroBias(calib.gyroBiasX, calib.gyroBiasY, calib.gyroBiasZ);
    imu.setAccelCalibration(calib.accelOffX, calib.accelOffY, calib.accelOffZ,
                           calib.accelScaleX, calib.accelScaleY, calib.accelScaleZ);
    stabilizer.setTargetOffset(calib.targetAngleOffset);
    orientation.setAngle(calib.targetAngleOffset);
    Serial.println(F("Calibration loaded."));
  } else {
    hasCalibration = false;
    Serial.println(F("EEPROM empty. Send 'c' for calibration."));
  }

  stabilizer.reset();
  Serial.println(F("READY. v,linear,turn | c=calib | z / z,val | p,i,d,l | P=print | w=save | D=debug | s=stop | e=erase"));
}

void processSerial() {
  if (Serial.available() < 1) return;
  char cmd = Serial.peek();

  if (cmd == 'v' || cmd == 'V') {
    if (Serial.available() < 6) return;
    Serial.read();
    if (Serial.peek() == ',') Serial.read();
    float linear = Serial.parseFloat();
    float turn = Serial.parseFloat();
    twist.setTwist(linear, turn);
    stabilizationEnabled = true;
  } else if (cmd == 's' || cmd == 'S') {
    Serial.read();
    twist.stop();
    stabilizationEnabled = false;
  } else if (cmd == 'D') {
    Serial.read();
    debugEnabled = !debugEnabled;
    Serial.print(F("Debug ")); Serial.println(debugEnabled ? F("ON") : F("OFF"));
  } else if (cmd == 'c' || cmd == 'C') {
    Serial.read();
    bool wasEmpty = !hasCalibration;
    if (runFullCalibration(imu.getMpu(), calib)) {
      hasCalibration = true;
      imu.setGyroBias(calib.gyroBiasX, calib.gyroBiasY, calib.gyroBiasZ);
      imu.setAccelCalibration(calib.accelOffX, calib.accelOffY, calib.accelOffZ,
                              calib.accelScaleX, calib.accelScaleY, calib.accelScaleZ);
      stabilizer.setTargetOffset(calib.targetAngleOffset);
      orientation.setAngle(calib.targetAngleOffset);
      stabilizer.reset();
      if (wasEmpty) {
        pidParams.kp = PID_KP;
        pidParams.ki = PID_KI;
        pidParams.kd = PID_KD;
        pidParams.limit = PID_LIMIT;
        stabilizer.setPid(pidParams.kp, pidParams.ki, pidParams.kd, pidParams.limit);
      }
    }
  } else if ((cmd == 'z' || cmd == 'Z') && hasCalibration) {
    Serial.read();
    if (Serial.peek() == ',' && Serial.available() >= 3) {
      Serial.read();
      float v = Serial.parseFloat();
      stabilizer.setTargetOffset(v);
      orientation.setAngle(v);
      calib.targetAngleOffset = v;
      calib.magic = EEPROM_MAGIC;
      saveCalibrationToEEPROM(calib);
      Serial.print(F("Zero set: ")); Serial.println(v);
    } else {
      Serial.println(F("Memorize zero: hold upright 2 sec..."));
      uint32_t t0 = millis();
      float sum = 0;
      int cnt = 0;
      while (millis() - t0 < 2000) {
        float ax, ay, az, gx, gy, gz;
        if (imu.read(ax, ay, az, gx, gy, gz)) {
          float pitch = orientation.update(ax, ay, az, gx, 0.01f);
          sum += pitch;
          cnt++;
        }
        delay(2);
      }
      if (cnt > 0) {
        float off = sum / cnt;
        stabilizer.setTargetOffset(off);
        orientation.setAngle(off);
        calib.targetAngleOffset = off;
        calib.magic = EEPROM_MAGIC;
        saveCalibrationToEEPROM(calib);
        Serial.print(F("Zero: ")); Serial.println(off);
      }
    }
  } else if (cmd == 'z' || cmd == 'Z') {
    Serial.read();
    if (!hasCalibration) Serial.println(F("Calibrate first (c)."));
  } else if (cmd == 'e' || cmd == 'E') {
    Serial.read();
    clearCalibrationEEPROM();
    clearPidEEPROM();
    hasCalibration = false;
    Serial.println(F("EEPROM cleared."));
  } else if (cmd == 'P') {
    Serial.read();
    Serial.print(F("PID: Kp=")); Serial.print(pidParams.kp);
    Serial.print(F(" Ki=")); Serial.print(pidParams.ki);
    Serial.print(F(" Kd=")); Serial.print(pidParams.kd);
    Serial.print(F(" limit=")); Serial.println(pidParams.limit);
  } else if (cmd == 'p') {
    Serial.read();
    if (Serial.available() >= 1) {
      float v = Serial.parseFloat();
      if (v >= 0) {
        pidParams.kp = v;
        stabilizer.setPid(pidParams.kp, pidParams.ki, pidParams.kd, pidParams.limit);
        Serial.print(F("Kp=")); Serial.println(v);
      }
    }
  } else if (cmd == 'i') {
    Serial.read();
    if (Serial.available() >= 1) {
      float v = Serial.parseFloat();
      if (v >= 0) {
        pidParams.ki = v;
        stabilizer.setPid(pidParams.kp, pidParams.ki, pidParams.kd, pidParams.limit);
        Serial.print(F("Ki=")); Serial.println(v);
      }
    }
  } else if (cmd == 'd') {
    Serial.read();
    if (Serial.available() >= 1) {
      float v = Serial.parseFloat();
      if (v >= 0) {
        pidParams.kd = v;
        stabilizer.setPid(pidParams.kp, pidParams.ki, pidParams.kd, pidParams.limit);
        Serial.print(F("Kd=")); Serial.println(v);
      }
    }
  } else if (cmd == 'l') {
    Serial.read();
    if (Serial.available() >= 1) {
      float v = Serial.parseFloat();
      if (v >= 100.0f) {
        pidParams.limit = v;
        stabilizer.setPid(pidParams.kp, pidParams.ki, pidParams.kd, pidParams.limit);
        Serial.print(F("limit=")); Serial.println(v);
      }
    }
  } else if (cmd == 'w' || cmd == 'W') {
    Serial.read();
    savePidToEEPROM(pidParams.kp, pidParams.ki, pidParams.kd, pidParams.limit);
    Serial.print(F("PID saved: Kp=")); Serial.print(pidParams.kp);
    Serial.print(F(" Ki=")); Serial.print(pidParams.ki);
    Serial.print(F(" Kd=")); Serial.print(pidParams.kd);
    Serial.print(F(" limit=")); Serial.println(pidParams.limit);
  } else if (cmd == 'f' || cmd == 'F') {
    if (Serial.available() < 6) return;
    Serial.read();
    if (Serial.peek() == ',') Serial.read();
    float kp = Serial.parseFloat();
    float ki = Serial.parseFloat();
    float kd = Serial.parseFloat();
    float lim = Serial.parseFloat();
    if (kp >= 0 && ki >= 0 && kd >= 0 && lim >= 100.0f) {
      pidParams.kp = kp;
      pidParams.ki = ki;
      pidParams.kd = kd;
      pidParams.limit = lim;
      stabilizer.setPid(pidParams.kp, pidParams.ki, pidParams.kd, pidParams.limit);
      Serial.print(F("PID: ")); Serial.print(kp); Serial.print(F(",")); Serial.print(ki);
      Serial.print(F(",")); Serial.print(kd); Serial.print(F(",")); Serial.println(lim);
    }
  } else {
    Serial.read();
  }
}

void loop() {
  processSerial();

  if (!hasCalibration) {
    motors.stop();
    static uint32_t lastMsg = 0;
    if (millis() - lastMsg > 2000) {
      lastMsg = millis();
      Serial.println(F("Waiting for 'c'..."));
    }
    delay(5);
    return;
  }

  float ax, ay, az, gx, gy, gz;
  if (!imu.read(ax, ay, az, gx, gy, gz)) return;

  float angle = orientation.update(ax, ay, az, gx, CONTROL_DT);
  float targetOffset = stabilizer.getTargetOffset();

  if (isFallenCheck(angle, targetOffset)) {
    if (!isFallen) {
      isFallen = true;
      stabilizer.reset();
      motors.stop();
      motors.disable();
    }
  } else if (isFallen && canRecover(angle, targetOffset)) {
    isFallen = false;
    stabilizer.reset();
  }

  if (isFallen) {
    motors.stop();
  } else if (!stabilizationEnabled) {
    motors.stop();
    motors.disable();
  } else {
    float linear = twist.getLinear();
    float turn = twist.getTurn();
    float motorSpeed = stabilizer.update(linear, angle, CONTROL_DT);

    if (turn == 0.0f) {
      motors.setBalanceSpeed((int16_t)motorSpeed);
    } else {
      float lim = fmaxf(pidParams.limit, 1.0f);
      float baseNorm = motorSpeed / lim;
      float leftNorm = baseNorm - turn;
      float rightNorm = baseNorm + turn;
      float m = fmaxf(fmaxf(fabsf(leftNorm), fabsf(rightNorm)), 0.001f);
      if (m > 1.0f) { leftNorm /= m; rightNorm /= m; }
      motors.setLeftRight((int16_t)(leftNorm * lim), (int16_t)(rightNorm * lim));
    }
    motors.enable();
  }

  if (debugEnabled) {
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > DEBUG_PRINT_MS) {
      lastPrint = millis();
      Serial.print(F("a:")); Serial.print(angle);
      Serial.print(F(" t:")); Serial.print(targetOffset);
      Serial.print(F(" m:")); Serial.print(stabilizer.getMotorSpeed());
      Serial.print(F(" L:")); Serial.print(twist.getLinear());
      Serial.print(F(" T:")); Serial.print(twist.getTurn());
      if (isFallen) Serial.print(F(" FALL"));
      if (!stabilizationEnabled) Serial.print(F(" STOP"));
      Serial.println();
    }
  }

  delay((int)(1000.0f / CONTROL_LOOP_HZ));
}
