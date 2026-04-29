/**
 * Giro-Robot — балансирующий робот
 * ================================
 * Каскад: Speed PID (внешний) → Angle PID (внутренний).
 * Speed: target_speed, current_speed → target_angle
 * Angle: target_angle, current_angle → motor_speed
 *
 * Безопасность: единый модуль safety.h. Условие аварии — реальное падение
 * (|angle - target| > FALL_ANGLE_DEG устойчиво). Восстановление — автоматическое,
 * когда робот снова вертикален и стоит спокойно. На обоих переходах выполняется
 * resetAllControlState() — гарантированный сброс накоплений Speed PID и Angle PID.
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
#include "safety.h"
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
FallHandler safety;

CalibData calib;
PidParams pidParams;
PidParams speedPidParams;  // Speed PID (внешний контур)
bool hasCalibration = false;
bool stabilizationEnabled = true;
bool debugEnabled = DEBUG_ENABLED;
bool monitorEnabled = false;
/** 0 — выкл; 1 — автотюн: цель 0/баланс, каскад активен; 2 — только Angle PID (angleOffset=0). Только RAM. */
uint8_t autotuneMode = 0;
bool csvTelemetryEnabled = false;
static uint32_t recoverySettleEndMs = 0;  // 0 = не в окне успокоения после recovery
static bool prevFallenPitch = false;      // на прошлом шаге был fallen → pitch только из акселя

uint8_t controlLoopHz = CONTROL_LOOP_HZ;  // Частота цикла (Гц), 25–100 для экономии ресурсов
float controlDt = 1.0f / CONTROL_LOOP_HZ;

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

/**
 * Полный сброс накопленного состояния контуров управления.
 * Вызывается на обоих переходах safety: JUST_FELL (чтобы не стартовать потом
 * с устаревшими интегралами) и JUST_RECOVERED (чистый старт после подъёма).
 */
static void resetAllControlState() {
  stabilizer.reset();         // Angle PID: интеграл, производная, телеметрия, motorSpeed
  speedController.reset();    // Speed PID + smoothOffset + prevTargetSign
  velocityFusion.setVelocity(0);
  twist.stop();               // чтобы после recovery не стартовать со старой целью
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(5);
  Wire.begin();
  Wire.setClock(400000);

  motors.begin();
  motors.stop();

  if (!imu.begin()) {
    Serial.println(F("!IMU"));
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
    Serial.println(F("+"));
  } else {
    hasCalibration = false;
    Serial.println(F("c?"));
  }

  resetAllControlState();
  safety.forceClear();

  uint8_t savedHz;
  if (loadLoopHzFromEEPROM(savedHz)) {
    controlLoopHz = savedHz;
    controlDt = 1.0f / controlLoopHz;
    Serial.print(controlLoopHz); Serial.println(F("Hz"));
  }

  Serial.println(F("OK"));
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
    if (autotuneMode == 0) {
      twist.setTwist(linear, turn);
      stabilizationEnabled = true;
    }
  } else if (cmd == 'V') {
    // V,speed_mps,turn — скорость в м/с (0 = остановка, баланс на месте)
    if (Serial.available() < 4) return;
    Serial.read();
    if (Serial.peek() == ',') Serial.read();
    float speedMps = Serial.parseFloat();
    float turn = 0.0f;
    while (Serial.available() && (Serial.peek() == ',' || Serial.peek() == ' ')) Serial.read();
    if (Serial.available() > 0 && Serial.peek() != '\n' && Serial.peek() != '\r') turn = Serial.parseFloat();
    if (autotuneMode == 0) {
      twist.setTargetSpeedMps(speedMps);
      twist.setTurn(turn);
      stabilizationEnabled = true;
    }
  } else if (cmd == 's' || cmd == 'S') {
    Serial.read();
    twist.stop();
    stabilizationEnabled = false;
  } else if (cmd == 'D') {
    Serial.read();
    debugEnabled = !debugEnabled;
    if (debugEnabled) csvTelemetryEnabled = false;
    Serial.print(F("D")); Serial.println(debugEnabled ? F("1") : F("0"));
  } else if (cmd == 'M') {
    Serial.read();
    monitorEnabled = !monitorEnabled;
    Serial.print(F("M")); Serial.println(monitorEnabled ? F("1") : F("0"));
  } else if (cmd == 'G') {
    Serial.read();
    Serial.println(F("G0"));
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
      resetAllControlState();
      safety.forceClear();
      if (wasEmpty) {
        pidParams.kp = PID_KP;
        pidParams.ki = PID_KI;
        pidParams.kd = PID_KD;
        pidParams.limit = PID_LIMIT;
        stabilizer.setPid(pidParams.kp, pidParams.ki, pidParams.kd, pidParams.limit);
      }
    }
  } else if (cmd == 'a') {
    // a,<mode> — режим автотюна (RAM): 0 выкл; 1 баланс на месте, каскад; 2 только Angle PID
    Serial.read();
    if (Serial.peek() == ',') Serial.read();
    int am = Serial.parseInt();
    if (am >= 0 && am <= 2) {
      autotuneMode = (uint8_t)am;
      twist.stop();
      Serial.write('a');
      Serial.println(autotuneMode);
    }
  } else if (cmd == 'x') {
    // x,<0|1> — машиночитаемая телеметрия CSV (отключает D/G при вкл.)
    Serial.read();
    if (Serial.peek() == ',') Serial.read();
    int xon = Serial.parseInt();
    csvTelemetryEnabled = (xon != 0);
    if (csvTelemetryEnabled) {
      debugEnabled = false;
      Serial.println(F("TH"));
    }
    Serial.print(F("CSV "));
    Serial.println(csvTelemetryEnabled ? F("ON") : F("OFF"));
  } else if (cmd == 'r') {
    // Полный сброс контуров (как после fall/recovery), без изменения FallHandler
    Serial.read();
    resetAllControlState();
    Serial.println(F("rOK"));
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
      Serial.print(F("Z=")); Serial.println(v);
    } else {
      Serial.println(F("2s"));
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
        Serial.print(F("Z=")); Serial.println(off);
      }
    }
  } else if (cmd == 'z' || cmd == 'Z') {
    Serial.read();
    if (!hasCalibration) Serial.println(F("!cal"));
  } else if (cmd == 'e' || cmd == 'E') {
    Serial.read();
    clearCalibrationEEPROM();
    clearPidEEPROM();
    hasCalibration = false;
  } else if (cmd == '?') {
    Serial.read();
    Serial.print(F("STATUS fall="));
    Serial.print(safety.isFallen() ? 1 : 0);
    Serial.print(F(" hz="));
    Serial.print(controlLoopHz);
    Serial.print(F(" atz="));
    Serial.println(autotuneMode);
  } else if (cmd == 'H') {
    Serial.read();
    if (Serial.peek() == ',' || Serial.available() >= 2) {
      if (Serial.peek() == ',') Serial.read();
      int hz = Serial.parseInt();
      if (hz >= 25 && hz <= 200) {
        controlLoopHz = (uint8_t)hz;
        controlDt = 1.0f / controlLoopHz;
        saveLoopHzToEEPROM(controlLoopHz);
        Serial.print(controlLoopHz); Serial.println(F("Hz"));
      }
    } else {
      Serial.print(controlLoopHz); Serial.println(F("Hz"));
    }
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
      Serial.println(F("c!"));
    }
    delay(5);
    return;
  }

  float ax, ay, az, gx, gy, gz;
  if (!imu.read(ax, ay, az, gx, gy, gz)) return;

  float gyroPitch = (GYRO_PITCH_AXIS == 1) ? gy : (GYRO_PITCH_AXIS == 2) ? gz : gx;
  float angle;
  if (prevFallenPitch) {
    float ap = atan2f(-ay, sqrtf(ax * ax + az * az + 0.001f)) * 57.29577951308232f;
    orientation.setAngle(ap);
    angle = ap;
  } else {
    angle = orientation.update(ax, ay, az, gyroPitch, controlDt);
  }
  float targetOffset = stabilizer.getTargetOffset();

  // ===== Safety: единый диспетчер падения/восстановления =====
  FallEvent ev = safety.update(angle, targetOffset, gyroPitch, millis());
  if (ev == FALL_JUST_FELL) {
    motors.stop();
    motors.disable();
    recoverySettleEndMs = 0;
    resetAllControlState();
  } else if (ev == FALL_JUST_RECOVERED) {
    resetAllControlState();
    orientation.setAngle(targetOffset);
    angle = targetOffset;
    motors.enable();
    recoverySettleEndMs = millis() + (uint32_t)RECOVERY_SETTLE_MS;
  }
  prevFallenPitch = safety.isFallen();
  if (safety.isFallen()) {
    motors.stop();
    if (csvTelemetryEnabled && hasCalibration) {
      Serial.println(F("TLM_FALL"));
    }
    delay((int)(1000.0f / controlLoopHz));
    return;
  }

  if (autotuneMode != 0) {
    twist.stop();
    stabilizationEnabled = true;
  }

  // ===== Оценка скорости =====
  twist.updateRamp(controlDt);
  float targetSpeed = twist.getTargetSpeedMps();

  int16_t left = motors.getLeftSpeed();
  int16_t right = motors.getRightSpeed();
  float effSteps = (left + (MOTOR2_INVERT ? -right : right)) * 0.5f;
  float vWheel = stepsPerSecToMps(effSteps);

  // Окно после подъёма: не интегрировать аксель в фузию скорости — иначе ложная v и Speed PID тянет корпус вперёд.
  bool recoveryFusionFrozen = (recoverySettleEndMs != 0);
  float vFused;
  if (recoveryFusionFrozen) {
    velocityFusion.setVelocity(0);
    vFused = 0.0f;
  } else {
    vFused = velocityFusion.update(ax, ay, az, angle, vWheel, controlDt);
  }

  bool recoverySettling = false;
  if (recoverySettleEndMs != 0) {
    uint32_t nowMs = millis();
    if (nowMs >= recoverySettleEndMs) {
      recoverySettleEndMs = 0;
    } else if (fabsf(vWheel) <= RECOVERY_SETTLE_MAX_VFUSED
               && fabsf(gyroPitch) <= RECOVERY_SETTLE_MAX_GYRO_DPS) {
      recoverySettleEndMs = 0;
    } else {
      recoverySettling = true;
    }
  }

  // Сброс velocity только при целевой 0, долгой стоянке и реальной остановке.
  // (чтобы не сбрасывать сразу после толчка — иначе Speed PID не увидит движение)
  static uint8_t zeroCount = 0;
  if (fabsf(targetSpeed) < 0.02f) {
    if (zeroCount < 200) zeroCount++;
    if (zeroCount > 80 && fabsf(vFused) < 0.05f) velocityFusion.setVelocity(0);
  } else {
    zeroCount = 0;
  }

  // ===== Управление =====
  if (!stabilizationEnabled) {
    motors.stop();
    motors.disable();
  } else {
    // Каскад: Speed PID → angleOffset (град, со сглаживанием) → Angle PID → motorSpeed (шаг/с).
    float angleOffset = 0;
    if (recoverySettling || autotuneMode == 2) {
      angleOffset = 0;
    } else {
      angleOffset = speedController.updateSmoothed(targetSpeed, vFused, controlDt);
    }
    float targetAngle = targetOffset + angleOffset;

    float turn = twist.getTurn() * TURN_SCALE;
    float motorSpeed = stabilizer.update(targetAngle, angle, controlDt);

    // Смягчение при большой ошибке — моторы не дёргаются резко
    float angleErr = fabsf(angle - targetAngle);
    if (angleErr > SOFT_ERR_THRESHOLD) {
      float t = (angleErr - SOFT_ERR_THRESHOLD) / (SOFT_ERR_MAX - SOFT_ERR_THRESHOLD);
      t = constrain(t, 0.0f, 1.0f);
      float scale = 1.0f - (1.0f - SOFT_ERR_MIN_SCALE) * t;
      motorSpeed *= scale;
    }

    if (fabsf(turn) < 0.001f) {
      motors.setBalanceSpeed((int16_t)motorSpeed);
    } else {
      float lim = fmaxf(pidParams.limit, 1.0f);
      // При повороте уменьшаем баланс, чтобы оба колеса крутились — вращение вокруг оси
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
      Serial.print(F("v:")); Serial.print(vFused, 3);
      Serial.print(F(" w:")); Serial.print(vWheel, 3);
      Serial.print(F(" a:")); Serial.print(velocityFusion.getVelocityAccel(), 3);
      Serial.println();
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
      if (safety.isFallen()) Serial.print(F(" FALL"));
      if (!stabilizationEnabled) Serial.print(F(" STOP"));
      Serial.println();
    }
  }

  if (csvTelemetryEnabled && hasCalibration) {
    float aoTel = (recoverySettling || autotuneMode == 2) ? 0.0f : speedController.getAngleOutput();
    float taTel = targetOffset + aoTel;
    Serial.print(F("TLM,0,"));
    Serial.print(angle, 4);
    Serial.print(',');
    Serial.print(gyroPitch, 4);
    Serial.print(',');
    Serial.print(taTel, 4);
    Serial.print(',');
    Serial.print(stabilizationEnabled ? stabilizer.getMotorSpeed() : 0.0f, 2);
    Serial.print(',');
    Serial.print(speedController.getPidError(), 5);
    Serial.print(',');
    Serial.print(speedController.getPidP(), 5);
    Serial.print(',');
    Serial.print(speedController.getPidI(), 5);
    Serial.print(',');
    Serial.print(speedController.getPidD(), 5);
    Serial.print(',');
    Serial.print(stabilizer.getPidError(), 5);
    Serial.print(',');
    Serial.print(stabilizer.getPidP(), 5);
    Serial.print(',');
    Serial.print(stabilizer.getPidI(), 5);
    Serial.print(',');
    Serial.print(stabilizer.getPidD(), 5);
    Serial.print(',');
    Serial.print(aoTel, 4);
    Serial.print(',');
    Serial.print(vFused, 5);
    Serial.print(',');
    Serial.print(vWheel, 5);
    Serial.print(',');
    Serial.print(autotuneMode);
    Serial.print(',');
    Serial.print(recoverySettling ? 1 : 0);
    Serial.print(',');
    Serial.print(targetSpeed, 4);
    Serial.print(',');
    Serial.println(stabilizationEnabled ? 1 : 0);
  }

  delay((int)(1000.0f / controlLoopHz));
}
