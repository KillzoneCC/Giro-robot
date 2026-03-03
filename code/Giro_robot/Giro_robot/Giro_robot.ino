/**
 * Giro-Robot — балансирующий робот
 * ================================
 * Каскад: Speed PID (внешний) → Angle PID (внутренний).
 * Speed: target_speed, current_speed → target_angle
 * Angle: target_angle, current_angle → motor_speed
 */

#include "config.h"
#include "motors.h"
#include "speed_motor.h"
#include "velocity_fusion.h"
#include "speed_controller.h"
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
VelocityFusion velocityFusion;
SpeedController speedController;
Motors motors;
Stabilizer stabilizer;
TwistControl twist;

CalibData calib;
PidParams pidParams;
PidParams speedPidParams;  // Speed PID (внешний контур)
bool isFallen = false;
bool hasCalibration = false;
bool stabilizationEnabled = true;
bool debugEnabled = DEBUG_ENABLED;
bool graphEnabled = false;
bool monitorEnabled = false;

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

  motors.begin();
  motors.stop();

  if (!imu.begin()) {
    Serial.println(F("MPU6050 not found!"));
    while (1) yield();
  }
  delay(100);

  if (loadPidFromEEPROM(pidParams)) {
    stabilizer.setPid(pidParams.kp, pidParams.ki, pidParams.kd, pidParams.limit);
  } else {
    pidParams.kp = PID_KP;
    pidParams.ki = PID_KI;
    pidParams.kd = PID_KD;
    pidParams.limit = PID_LIMIT;
    stabilizer.setPid(pidParams.kp, pidParams.ki, pidParams.kd, pidParams.limit);
  }
  if (loadSpeedPidFromEEPROM(speedPidParams)) {
    speedController.setPid(speedPidParams.kp, speedPidParams.ki, speedPidParams.kd, speedPidParams.limit);
  } else {
    speedPidParams.kp = SPEED_PID_KP;
    speedPidParams.ki = SPEED_PID_KI;
    speedPidParams.kd = SPEED_PID_KD;
    speedPidParams.limit = SPEED_PID_ANGLE_LIMIT;
    speedController.setPid(speedPidParams.kp, speedPidParams.ki, speedPidParams.kd, speedPidParams.limit);
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
  speedController.reset();
  Serial.println(F("READY"));
}

void processSerial() {
  if (Serial.available() < 1) return;
  char cmd = Serial.peek();

  if (cmd == 'v') {
    // v,linear,turn — linear -1..1 (совместимость)
    if (Serial.available() < 6) return;
    Serial.read();
    if (Serial.peek() == ',') Serial.read();
    float linear = Serial.parseFloat();
    float turn = Serial.parseFloat();
    twist.setTwist(linear, turn);
    stabilizationEnabled = true;
  } else if (cmd == 'V') {
    // V,speed_mps,turn — скорость в м/с (0 = остановка, баланс на месте)
    if (Serial.available() < 4) return;
    Serial.read();
    if (Serial.peek() == ',') Serial.read();
    float speedMps = Serial.parseFloat();
    float turn = 0.0f;
    while (Serial.available() && (Serial.peek() == ',' || Serial.peek() == ' ')) Serial.read();
    if (Serial.available() > 0 && Serial.peek() != '\n' && Serial.peek() != '\r') turn = Serial.parseFloat();
    twist.setTargetSpeedMps(speedMps);
    twist.setTurn(turn);
    stabilizationEnabled = true;
  } else if (cmd == 's' || cmd == 'S') {
    Serial.read();
    twist.stop();
    stabilizationEnabled = false;
  } else if (cmd == 'D') {
    Serial.read();
    debugEnabled = !debugEnabled;
    Serial.print(F("Debug ")); Serial.println(debugEnabled ? F("ON") : F("OFF"));
  } else if (cmd == 'M') {
    Serial.read();
    monitorEnabled = !monitorEnabled;
    Serial.print(F("Monitor ")); Serial.println(monitorEnabled ? F("ON (v м/с)") : F("OFF"));
  } else if (cmd == 'G') {
    Serial.read();
    graphEnabled = !graphEnabled;
    if (graphEnabled) debugEnabled = false;
    Serial.print(F("Graph "));
    if (graphEnabled) {
      Serial.println(F("ON. Plotter: target,angle,error,P,I,D,output,speed_mps"));
    } else {
      Serial.println(F("OFF"));
    }
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
      speedController.reset();
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
          float gp = (GYRO_PITCH_AXIS == 1) ? gy : (GYRO_PITCH_AXIS == 2) ? gz : gx;
          float pitch = orientation.update(ax, ay, az, gp, 0.01f);
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
  } else if (cmd == 'P') {
    Serial.read();
    if (Serial.available() >= 1 && Serial.peek() == '2') {
      Serial.read();
      Serial.print(F("Speed PID: Kp=")); Serial.print(speedPidParams.kp);
      Serial.print(F(" Ki=")); Serial.print(speedPidParams.ki);
      Serial.print(F(" Kd=")); Serial.print(speedPidParams.kd);
      Serial.print(F(" limit=")); Serial.println(speedPidParams.limit);
    } else {
      Serial.print(F("PID: Kp=")); Serial.print(pidParams.kp);
      Serial.print(F(" Ki=")); Serial.print(pidParams.ki);
      Serial.print(F(" Kd=")); Serial.print(pidParams.kd);
      Serial.print(F(" limit=")); Serial.println(pidParams.limit);
    }
  } else if (cmd == 'p') {
    Serial.read();
    if (Serial.available() >= 1 && Serial.peek() != '2') {
      float v = Serial.parseFloat();
      if (v >= 0) { pidParams.kp = v; stabilizer.setPid(pidParams.kp, pidParams.ki, pidParams.kd, pidParams.limit); }
    }
  } else if (cmd == 'i') {
    Serial.read();
    if (Serial.available() >= 1 && Serial.peek() != '2') {
      float v = Serial.parseFloat();
      if (v >= 0) { pidParams.ki = v; stabilizer.setPid(pidParams.kp, pidParams.ki, pidParams.kd, pidParams.limit); }
    }
  } else if (cmd == 'd') {
    Serial.read();
    if (Serial.available() >= 1 && Serial.peek() != '2') {
      float v = Serial.parseFloat();
      if (v >= 0) { pidParams.kd = v; stabilizer.setPid(pidParams.kp, pidParams.ki, pidParams.kd, pidParams.limit); }
    }
  } else if (cmd == 'l') {
    Serial.read();
    if (Serial.available() >= 1 && Serial.peek() != '2') {
      float v = Serial.parseFloat();
      if (v >= 100.0f) { pidParams.limit = v; stabilizer.setPid(pidParams.kp, pidParams.ki, pidParams.kd, pidParams.limit); }
    }
  } else if (cmd == 'w' || cmd == 'W') {
    Serial.read();
    if (Serial.available() >= 1 && Serial.peek() == '2') {
      Serial.read();
      saveSpeedPidToEEPROM(speedPidParams.kp, speedPidParams.ki, speedPidParams.kd, speedPidParams.limit);
    } else {
      savePidToEEPROM(pidParams.kp, pidParams.ki, pidParams.kd, pidParams.limit);
    }
  } else if (cmd == 'f' || cmd == 'F') {
    if (Serial.available() < 6) return;
    Serial.read();
    if (Serial.available() >= 1 && Serial.peek() == '2') {
      Serial.read();
      if (Serial.peek() == ',') Serial.read();
      float kp = Serial.parseFloat();
      float ki = Serial.parseFloat();
      float kd = Serial.parseFloat();
      float lim = Serial.parseFloat();
      if (kp >= 0 && ki >= 0 && kd >= 0 && lim >= 1.0f) {
        speedPidParams.kp = kp;
        speedPidParams.ki = ki;
        speedPidParams.kd = kd;
        speedPidParams.limit = lim;
        speedController.setPid(speedPidParams.kp, speedPidParams.ki, speedPidParams.kd, speedPidParams.limit);
      }
    } else {
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
      }
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

  float gyroPitch = (GYRO_PITCH_AXIS == 1) ? gy : (GYRO_PITCH_AXIS == 2) ? gz : gx;
  float angle = orientation.update(ax, ay, az, gyroPitch, CONTROL_DT);
  float targetOffset = stabilizer.getTargetOffset();

  static uint32_t fallStartMs = 0;
  static uint32_t recoverStartMs = 0;

  if (isFallenCheck(angle, targetOffset)) {
    if (fallStartMs == 0) fallStartMs = millis();
    if (!isFallen && (millis() - fallStartMs) >= FALL_DEBOUNCE_MS) {
      isFallen = true;
      fallStartMs = 0;
      stabilizer.reset();
      speedController.reset();
      velocityFusion.setVelocity(0);
      motors.stop();
      motors.disable();
    }
  } else {
    fallStartMs = 0;
    if (isFallen && canRecover(angle, targetOffset)) {
      if (recoverStartMs == 0) recoverStartMs = millis();
      if ((millis() - recoverStartMs) >= RECOVERY_DEBOUNCE_MS) {
        isFallen = false;
        recoverStartMs = 0;
        stabilizer.reset();
        speedController.reset();
        velocityFusion.setVelocity(0);
      }
    } else {
      recoverStartMs = 0;
    }
  }

  // Спидометр: всегда по моторам. С учётом MOTOR2_INVERT.
  twist.updateRamp(CONTROL_DT);
  float targetSpeed = twist.getTargetSpeedMps();

  int16_t left = motors.getLeftSpeed();
  int16_t right = motors.getRightSpeed();
  float effSteps = (left + (MOTOR2_INVERT ? -right : right)) * 0.5f;
  float vWheel = stepsPerSecToMps(effSteps);

  float vFused = velocityFusion.update(ax, ay, az, angle, vWheel, CONTROL_DT);

  // Сброс velocity только при целевой 0, долгой стоянке и реальной остановке
  // (чтобы не сбрасывать сразу после толчка — иначе Speed PID не увидит движение)
  static uint8_t zeroCount = 0;
  if (fabsf(targetSpeed) < 0.02f) {
    if (zeroCount < 200) zeroCount++;
    if (zeroCount > 80 && fabsf(vFused) < 0.05f) velocityFusion.setVelocity(0);
  } else {
    zeroCount = 0;
  }

  if (isFallen) {
    motors.stop();
  } else if (!stabilizationEnabled) {
    motors.stop();
    motors.disable();
  } else {
    // Каскад: Speed PID → angleOffset (град), Angle PID → motorSpeed (шаг/с).
    // Speed PID всегда активен: при targetSpeed=0 тормозит до остановки, при движении — задаёт угол.
    static float prevTargetSign = 0.0f;
    static float smoothOffset = 0.0f;

    float s = (targetSpeed > 0.02f) ? 1.0f : (targetSpeed < -0.02f) ? -1.0f : 0.0f;
    bool signChanged = (prevTargetSign != 0.0f && s != 0.0f && prevTargetSign != s);
    bool fromBalance = (prevTargetSign == 0.0f && s != 0.0f);
    if (signChanged) speedController.reset();
    prevTargetSign = s;

    // Всегда: targetSpeed и vFused — при 0 скорости PID тормозит, если робот ещё движется
    float rawOffset = speedController.update(targetSpeed, vFused, CONTROL_DT);
    bool needInstant = signChanged || fromBalance;
    if (needInstant) {
      smoothOffset = rawOffset;
    } else {
      smoothOffset = ANGLE_OFFSET_SMOOTH * rawOffset + (1.0f - ANGLE_OFFSET_SMOOTH) * smoothOffset;
    }
    float angleOffset = smoothOffset;
    float targetAngle = targetOffset + angleOffset;

    float turn = twist.getTurn() * TURN_SCALE;
    float motorSpeed = stabilizer.update(targetAngle, angle, CONTROL_DT);

    if (fabsf(turn) < 0.001f) {
      motors.setBalanceSpeed((int16_t)motorSpeed);
    } else {
      float lim = fmaxf(pidParams.limit, 1.0f);
      // При повороте уменьшаем баланс, чтобы оба колеса крутились — вращение вокруг своей оси, а не опора на одно
      float turnAbs = fabsf(turn);
      float balanceBlend = 1.0f - turnAbs * (1.0f - TURN_BALANCE_BLEND);
      if (balanceBlend < 0.0f) balanceBlend = 0.0f;
      float baseNorm = (motorSpeed / lim) * balanceBlend;
      float leftNorm = baseNorm - turn;
      float rightNorm = baseNorm + turn;
      float m = fmaxf(fmaxf(fabsf(leftNorm), fabsf(rightNorm)), 0.001f);
      if (m > 1.0f) { leftNorm /= m; rightNorm /= m; }
      motors.setLeftRight((int16_t)(leftNorm * lim), (int16_t)(rightNorm * lim));
    }
    motors.enable();
  }

  if (monitorEnabled && hasCalibration) {
    static uint32_t lastMonitor = 0;
    if (millis() - lastMonitor >= DEBUG_PRINT_MS) {
      lastMonitor = millis();
      Serial.print(F("v: ")); Serial.print(vFused, 3);
      Serial.print(F(" (колёса: ")); Serial.print(vWheel, 3);
      Serial.print(F(", IMU: ")); Serial.print(velocityFusion.getVelocityAccel(), 3);
      Serial.println(F(") m/s"));
    }
  }

  if (debugEnabled) {
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > DEBUG_PRINT_MS) {
      lastPrint = millis();
      Serial.print(F("a:")); Serial.print(angle);
      Serial.print(F(" t:")); Serial.print(targetOffset);
      Serial.print(F(" m:")); Serial.print(stabilizer.getMotorSpeed());
      Serial.print(F(" target:")); Serial.print(twist.getTargetSpeedMps(), 2);
      Serial.print(F(" T:")); Serial.print(twist.getTurn());
      if (isFallen) Serial.print(F(" FALL"));
      if (!stabilizationEnabled) Serial.print(F(" STOP"));
      Serial.println();
    }
  }

  if (graphEnabled) {
    static uint32_t lastGraph = 0;
    if (millis() - lastGraph >= GRAPH_INTERVAL_MS) {
      lastGraph = millis();
      float targetAngle = targetOffset + speedController.getAngleOutput();
      Serial.print(targetAngle);
      Serial.print(',');
      Serial.print(angle);
      Serial.print(',');
      Serial.print(stabilizer.getPidError());
      Serial.print(',');
      Serial.print(stabilizer.getPidP());
      Serial.print(',');
      Serial.print(stabilizer.getPidI());
      Serial.print(',');
      Serial.print(stabilizer.getPidD());
      Serial.print(',');
      Serial.print(stabilizer.getMotorSpeed());
      Serial.print(',');
      Serial.println(vFused);
    }
  }

  delay((int)(1000.0f / CONTROL_LOOP_HZ));
}
