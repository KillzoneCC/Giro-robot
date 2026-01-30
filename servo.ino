#include <Wire.h>
#include <SoftwareSerial.h>
#include <TMC2208Stepper.h>

// === НАСТРОЙКИ ПОЛЬЗОВАТЕЛЯ ===
#define TOTAL_STEPS    4240  // Максимальное количество шагов
// STEP_DELAY_US убрали из define, теперь это переменная ниже

// === ПИНЫ ===
#define DIR1_PIN   5
#define STEP1_PIN  3
#define DIR2_PIN   10
#define STEP2_PIN  9

// === UART ===
#define UART1_RX   7
#define UART1_TX   6
#define UART2_RX   4
#define UART2_TX   2

// === ТЕХНИЧЕСКИЕ КОНСТАНТЫ ===
#define PULSE_US       10    // Длительность импульса шага
#define DIR_SETUP_US   5     // Пауза смены направления
#define STEP_GAP_US    10    // Микро-пауза между мотором 1 и 2
#define R_SENSE        0.11f // Для TMC22xx

// === ПЕРЕМЕННЫЕ ДЛЯ СКОРОСТИ ===
int stepDelay = 800;         // Начальная скорость (задержка в мкс)
const int minDelay = 100;    // Максимальная скорость (минимальная задержка)
const int maxDelay = 4000;   // Минимальная скорость (максимальная задержка)
const int speedStep = 50;    // Шаг изменения скорости при нажатии

SoftwareSerial TMCuart1(UART1_RX, UART1_TX);
SoftwareSerial TMCuart2(UART2_RX, UART2_TX);
TMC2208Stepper driver1(&TMCuart1, false);
TMC2208Stepper driver2(&TMCuart2, false);

void setup() {
  // Инициализация USB Serial для общения с компьютером
  Serial.begin(115200);
  Serial.println("System Start. Use '+' to speed up, '-' to slow down.");

  // Настройка пинов
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(STEP1_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(STEP2_PIN, OUTPUT);
  digitalWrite(STEP1_PIN, LOW);
  digitalWrite(STEP2_PIN, LOW);

  // --- Инициализация драйверов (ток, микрошаг) ---
  // Драйвер 1
  TMCuart1.begin(115200);
  delay(100);
  driver1.pdn_disable(true);
  driver1.mstep_reg_select(true);
  driver1.microsteps(16);             
  driver1.rms_current(1200, 0.5, R_SENSE); 
  driver1.push();
  delay(50);

  // Драйвер 2
  TMCuart2.begin(115200);
  delay(100);
  driver2.pdn_disable(true);
  driver2.mstep_reg_select(true);
  driver2.microsteps(16);             
  driver2.rms_current(1200, 0.5, R_SENSE); 
  driver2.push();
  delay(50);

  // Предварительная установка направления
  digitalWrite(DIR1_PIN, HIGH);
  digitalWrite(DIR2_PIN, HIGH);
  delay(100);
}

// Вспомогательная функция для Мотора 1
inline void stepMotor1(bool forward) {
  digitalWrite(DIR1_PIN, forward ? HIGH : LOW);
  delayMicroseconds(DIR_SETUP_US);
  digitalWrite(STEP1_PIN, HIGH);
  delayMicroseconds(PULSE_US);
  digitalWrite(STEP1_PIN, LOW);
}

// Вспомогательная функция для Мотора 2
inline void stepMotor2(bool forward) {
  digitalWrite(DIR2_PIN, forward ? HIGH : LOW);
  delayMicroseconds(DIR_SETUP_US);
  digitalWrite(STEP2_PIN, HIGH);
  delayMicroseconds(PULSE_US);
  digitalWrite(STEP2_PIN, LOW);
}

// === ГЛАВНАЯ ФУНКЦИЯ ДВИЖЕНИЯ ===
void stepBothOpposite(bool mainDir) {
  stepMotor1(mainDir);        
  delayMicroseconds(STEP_GAP_US); 
  stepMotor2(!mainDir);           
}

// === ФУНКЦИЯ ЧТЕНИЯ КЛАВИАТУРЫ ===
void checkInput() {
  if (Serial.available() > 0) {
    char key = Serial.read();
    
    // Игнорируем символы перевода строки, если они прилетают
    if (key == '\n' || key == '\r') return;

    if (key == '+' || key == '=') { // '+' (иногда '=' без Shift)
      stepDelay -= speedStep;
      if (stepDelay < minDelay) stepDelay = minDelay;
      Serial.print("FASTER! Delay: "); Serial.println(stepDelay);
    } 
    else if (key == '-' || key == '_') { // '-' (иногда '_' с Shift)
      stepDelay += speedStep;
      if (stepDelay > maxDelay) stepDelay = maxDelay;
      Serial.print("SLOWER. Delay: "); Serial.println(stepDelay);
    }
  }
}

void loop() {
  // --- Движение В ОДНУ сторону ---
  for (unsigned long i = 0; i < TOTAL_STEPS; i++) {
    checkInput(); // Проверяем кнопки перед каждым шагом
    stepBothOpposite(true); 
    delayMicroseconds(stepDelay); // Используем переменную
  }
  
  Serial.println("Direction Change -> Back");
  delay(1000);
}