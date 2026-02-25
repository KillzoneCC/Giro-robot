/*
  Тест скорости шаговых двигателей (те же пины и тип, что в shag.h).
  Подключение: мотор 1 — 3,7,8,5; мотор 2 — 9,7,8,10 (пины 7,8 общие).
  Serial 115200: команды или автоматический прогон скоростей.
*/

#include <Stepper.h>

#define STEPS_PER_REV  200

#define MOTOR1_PIN1  3
#define MOTOR1_PIN2  7
#define MOTOR1_PIN3  8
#define MOTOR1_PIN4  5
#define MOTOR2_PIN1  9
#define MOTOR2_PIN2  7
#define MOTOR2_PIN3  8
#define MOTOR2_PIN4  10

Stepper motor1(STEPS_PER_REV, MOTOR1_PIN1, MOTOR1_PIN2, MOTOR1_PIN3, MOTOR1_PIN4);
Stepper motor2(STEPS_PER_REV, MOTOR2_PIN1, MOTOR2_PIN2, MOTOR2_PIN3, MOTOR2_PIN4);

#ifndef SERIAL_BAUD
#define SERIAL_BAUD 115200
#endif

// Сколько шагов крутить в одном тесте (чем больше — тем дольше и точнее)
#define TEST_STEPS  8000

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.println("STEPPER SPEED TEST");
  Serial.println("Commands: 1=run test once, 2=loop tests, s=stop");
  Serial.println("Same pins as shag.h (balance robot).");
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '1') runSpeedTestOnce();
    if (c == '2') {
      while (Serial.available() == 0 || Serial.peek() != 's') {
        runSpeedTestOnce();
        delay(500);
      }
      if (Serial.available() > 0) Serial.read();
    }
    if (c == 's') { /* stop already handled by single run */ }
  }
  delay(50);
}

void runSpeedTestOnce() {
  const unsigned long speeds[] = { 500, 2000, 4000, 6000, 8000, 9800 };
  const int nSpeeds = sizeof(speeds) / sizeof(speeds[0]);

  Serial.println("--- Set RPM | Steps | Time ms | Steps/s | Eff.RPM | Note ---");

  float prevStepsPerSec = 0;
  for (int i = 0; i < nSpeeds; i++) {
    unsigned long rpm = speeds[i];
    motor1.setSpeed(rpm);
    motor2.setSpeed(rpm);

    unsigned long t0 = millis();
    for (int s = 0; s < TEST_STEPS; s++) {
      motor1.step(1);
      motor2.step(-1);
    }
    unsigned long dt = millis() - t0;

    float stepsPerSec = (dt > 0) ? (1000.0f * TEST_STEPS / (float)dt) : 0;
    float effRPM = stepsPerSec * 60.0f / (float)STEPS_PER_REV;  // steps/s -> rev/min

    Serial.print("  ");
    Serial.print(rpm);
    Serial.print("    | ");
    Serial.print(TEST_STEPS);
    Serial.print(" | ");
    Serial.print(dt);
    Serial.print("   | ");
    Serial.print(stepsPerSec, 1);
    Serial.print("  | ");
    Serial.print((int)effRPM);
    Serial.print("   | ");
    if (i > 0 && stepsPerSec - prevStepsPerSec < 100) {
      Serial.println("ceiling");
    } else {
      Serial.println("-");
    }
    prevStepsPerSec = stepsPerSec;
  }

  Serial.println("Done. Send 1 or 2 to repeat.");
}
