/*
  IMU Calibration — в стиле DJI Tello
  ==================================
  Экспериментальная калибровка MPU6050 по всем осям (6 позиций).
  Аналогично Tello: укладываете гиро-робота на 6 плоскостей,
  скетч собирает данные и вычисляет offsets для акселерометра и гироскопа.

  После калибровки скопируйте выведенные значения в основной код (fgggggggggggg5.ino).

  Подключение MPU6050: VCC→3.3V, GND→GND, SDA→A4, SCL→A5
  Serial: 115200 бод
*/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#ifndef SERIAL_BAUD
#define SERIAL_BAUD 115200
#endif

#define CALIB_SAMPLES_PER_POS  800   // сэмплов на каждую позицию (больше = точнее)
#define GRAVITY_MS2            9.80665f
#define RAD_TO_DEG_F           57.29577951308232f

Adafruit_MPU6050 mpu;

// Позиции калибровки (как у DJI Tello): 1=Z↑ 2=Z↓ 3=X↑ 4=X↓ 5=Y↑ 6=Y↓
const char* POS_NAMES[] = {
  "1: Стоит вертикально (колёса вниз)",
  "2: На спине (колёса вверх)",
  "3: На ЛЕВОМ боку",
  "4: На ПРАВОМ боку",
  "5: Нос в потолок",
  "6: Нос в пол"
};

// Результаты калибровки
float accel_offset_x = 0, accel_offset_y = 0, accel_offset_z = 0;
float gyro_bias_x_dps = 0, gyro_bias_y_dps = 0, gyro_bias_z_dps = 0;
float accel_scale_x = 1.0f, accel_scale_y = 1.0f, accel_scale_z = 1.0f;

// Буферы для сбора данных по 6 позициям
static float acc_means[6][3];
static float gyro_means[6][3];

void printHeader() {
  Serial.println();
  Serial.println("========================================");
  Serial.println("  IMU CALIBRATION (DJI Tello style)");
  Serial.println("  Гиро-робот: 6 позиций по осям");
  Serial.println("========================================");
  Serial.println();
}

void collectPosition(int posIndex) {
  Serial.println(POS_NAMES[posIndex]);
  Serial.print("  Уложите робота и подождите ");
  Serial.print((CALIB_SAMPLES_PER_POS * 3) / 1000);
  Serial.println(" сек...");

  float sum_ax = 0, sum_ay = 0, sum_az = 0;
  float sum_gx = 0, sum_gy = 0, sum_gz = 0;
  uint32_t count = 0;

  uint32_t start = millis();
  while (count < CALIB_SAMPLES_PER_POS) {
    sensors_event_t a, g, temp;
    if (mpu.getEvent(&a, &g, &temp)) {
      sum_ax += a.acceleration.x;
      sum_ay += a.acceleration.y;
      sum_az += a.acceleration.z;
      sum_gx += g.gyro.x * RAD_TO_DEG_F;
      sum_gy += g.gyro.y * RAD_TO_DEG_F;
      sum_gz += g.gyro.z * RAD_TO_DEG_F;
      count++;
    }
    delay(3);
  }
  uint32_t elapsed = millis() - start;

  float mean_ax = sum_ax / count;
  float mean_ay = sum_ay / count;
  float mean_az = sum_az / count;
  float mean_gx = sum_gx / count;
  float mean_gy = sum_gy / count;
  float mean_gz = sum_gz / count;

  // Сохраняем для последующего расчёта
  acc_means[posIndex][0] = mean_ax;
  acc_means[posIndex][1] = mean_ay;
  acc_means[posIndex][2] = mean_az;
  gyro_means[posIndex][0] = mean_gx;
  gyro_means[posIndex][1] = mean_gy;
  gyro_means[posIndex][2] = mean_gz;

  Serial.print("  [OK] ax="); Serial.print(mean_ax, 3);
  Serial.print(" ay="); Serial.print(mean_ay, 3);
  Serial.print(" az="); Serial.print(mean_az, 3);
  Serial.print(" | gx="); Serial.print(mean_gx, 4);
  Serial.print(" gy="); Serial.print(mean_gy, 4);
  Serial.print(" gz="); Serial.print(mean_gz, 4);
  Serial.print(" ("); Serial.print(elapsed); Serial.println(" ms)");
  Serial.println();

  // После 6-й позиции — вычисляем калибровку
  if (posIndex == 5) {
    computeCalibration();
  }
}

void computeCalibration() {
  // Гироскоп: среднее по всем 6 позициям (в покое везде должен быть 0)
  gyro_bias_x_dps = 0;
  gyro_bias_y_dps = 0;
  gyro_bias_z_dps = 0;
  for (int i = 0; i < 6; i++) {
    gyro_bias_x_dps += gyro_means[i][0];
    gyro_bias_y_dps += gyro_means[i][1];
    gyro_bias_z_dps += gyro_means[i][2];
  }
  gyro_bias_x_dps /= 6.0f;
  gyro_bias_y_dps /= 6.0f;
  gyro_bias_z_dps /= 6.0f;

  // Акселерометр: 6-point calibration
  // offset_X = (reading при +1g + reading при -1g) / 2
  // scale_X = (reading при +1g - reading при -1g) / (2 * 9.81)
  accel_offset_x = (acc_means[2][0] + acc_means[3][0]) / 2.0f;  // X: pos 3 (+1g) и 4 (-1g)
  accel_offset_y = (acc_means[4][1] + acc_means[5][1]) / 2.0f;  // Y: pos 5 (+1g) и 6 (-1g)
  accel_offset_z = (acc_means[0][2] + acc_means[1][2]) / 2.0f;  // Z: pos 1 (+1g) и 2 (-1g)

  float range_x = acc_means[2][0] - acc_means[3][0];
  float range_y = acc_means[4][1] - acc_means[5][1];
  float range_z = acc_means[0][2] - acc_means[1][2];

  accel_scale_x = (fabsf(range_x) > 0.1f) ? (2.0f * GRAVITY_MS2 / range_x) : 1.0f;
  accel_scale_y = (fabsf(range_y) > 0.1f) ? (2.0f * GRAVITY_MS2 / range_y) : 1.0f;
  accel_scale_z = (fabsf(range_z) > 0.1f) ? (2.0f * GRAVITY_MS2 / range_z) : 1.0f;

  printCalibrationResult();
}

void printCalibrationResult() {
  Serial.println();
  Serial.println("========================================");
  Serial.println("  КАЛИБРОВКА ЗАВЕРШЕНА!");
  Serial.println("========================================");
  Serial.println();
  Serial.println("Скопируйте эти значения в основной код (fgggggggggggg5.ino):");
  Serial.println();
  Serial.println("// --- Калибровка IMU (6-point, Tello-style) ---");
  Serial.print("static constexpr float ACCEL_OFFSET_X = ");
  Serial.print(accel_offset_x, 6);
  Serial.println("f;");
  Serial.print("static constexpr float ACCEL_OFFSET_Y = ");
  Serial.print(accel_offset_y, 6);
  Serial.println("f;");
  Serial.print("static constexpr float ACCEL_OFFSET_Z = ");
  Serial.print(accel_offset_z, 6);
  Serial.println("f;");
  Serial.print("static constexpr float GYRO_BIAS_X_DPS = ");
  Serial.print(gyro_bias_x_dps, 6);
  Serial.println("f;");
  Serial.print("static constexpr float GYRO_BIAS_Y_DPS = ");
  Serial.print(gyro_bias_y_dps, 6);
  Serial.println("f;");
  Serial.print("static constexpr float GYRO_BIAS_Z_DPS = ");
  Serial.print(gyro_bias_z_dps, 6);
  Serial.println("f;");
  Serial.print("static constexpr float ACCEL_SCALE_X = ");
  Serial.print(accel_scale_x, 6);
  Serial.println("f;");
  Serial.print("static constexpr float ACCEL_SCALE_Y = ");
  Serial.print(accel_scale_y, 6);
  Serial.println("f;");
  Serial.print("static constexpr float ACCEL_SCALE_Z = ");
  Serial.print(accel_scale_z, 6);
  Serial.println("f;");
  Serial.println();
  Serial.println("В основном коде при чтении IMU применяйте:");
  Serial.println("  ax_cal = (ax - ACCEL_OFFSET_X) * ACCEL_SCALE_X;");
  Serial.println("  ay_cal = (ay - ACCEL_OFFSET_Y) * ACCEL_SCALE_Y;");
  Serial.println("  az_cal = (az - ACCEL_OFFSET_Z) * ACCEL_SCALE_Z;");
  Serial.println("  gx_cal = gx - GYRO_BIAS_X_DPS;  // и т.д.");
  Serial.println();
  Serial.println("Отправьте 'r' для повторной калибровки.");
  Serial.println("========================================");
}

void runFullCalibration() {
  printHeader();
  Serial.println("Подготовка: разместите робота на ровной поверхности.");
  Serial.println("Избегайте вибраций, магнитных полей, металла рядом.");
  Serial.println();
  delay(2000);

  for (int i = 0; i < 6; i++) {
    collectPosition(i);
    if (i < 5) {
      Serial.println(">>> Переходите к следующей позиции <<<");
      delay(1500);
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  if (!mpu.begin()) {
    Serial.println("ОШИБКА: MPU6050 не найден! Проверьте подключение.");
    while (1) yield();
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050 OK. Отправьте 's' для старта калибровки.");
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 's' || c == 'S' || c == 'r' || c == 'R') {
      runFullCalibration();
    }
  }
  delay(100);
}
