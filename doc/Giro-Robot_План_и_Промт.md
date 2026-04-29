# 🤖 Giro-Robot — Подробный план доработок + Промт для AI-ассистента

**Проект:** Балансирующий двухколёсный робот (Arduino Nano + MPU6050 + TMC2208)  
**Документ:** План исправлений, доработок и улучшений  
**Дата:** 2026-04

---

## ЧАСТЬ 1 — ПОДРОБНЫЙ ПЛАН ДОРАБОТОК

---

## 🔴 ПРОБЛЕМА 1: Робот уходит в ошибку (fall=1) при поднятии рукой и не выходит из неё

### Диагноз (на основе кода)

Текущая логика в `safety.h` / `config.h`:
- Робот считается «упавшим» если `|angle - target| > FALL_ANGLE_DEG (55°)` в течение `FALL_DEBOUNCE_MS (200 мс)`
- Восстановление: `|angle - target| < FALL_RECOVERY_DEG (15°)` И `|gyroRate| < FALL_RECOVERY_RATE_MAX (60 °/с)` в течение `RECOVERY_DEBOUNCE_MS (400 мс)`

**Корневая причина проблемы:** Когда робота поднимают рукой — угол выходит за 55°, срабатывает `fall=1`. Но пока ты держишь его в воздухе и пытаешься выровнять, `gyroRate` может быть высоким (рука трясётся, ты поворачиваешь робота) — поэтому условие восстановления НЕ выполняется. `RECOVERY_DEBOUNCE_MS` (400 мс) сбрасывается каждый раз, когда условие нарушается — и таймер никогда не добегает до конца.

**Почему «само проходит после падения на пол»:** Когда робот лежит на полу и ты его ПЛАВНО ставишь — скорость движения руки низкая, `gyroRate` маленький, угол быстро входит в зону `<15°`, таймер 400 мс добегает — `fall=0`.

---

### ✅ Решение 1A — Увеличить допуск `FALL_RECOVERY_RATE_MAX`

**Файл:** `config.h`

```cpp
// БЫЛО:
#define FALL_RECOVERY_RATE_MAX   60.0f   // °/с

// СТАЛО (попробовать 90–120):
#define FALL_RECOVERY_RATE_MAX   100.0f  // °/с
```

**Эффект:** При удержании робота рукой руки дрожат ~60–90 °/с. Увеличение порога до 100 позволит выйти из fall даже при небольшом движении рукой.

**Риск:** Небольшой — может чуть увеличить вероятность ложного восстановления при падении «насквозь» через вертикаль. Компенсируется `RECOVERY_DEBOUNCE_MS`.

---

### ✅ Решение 1B — Уменьшить `RECOVERY_DEBOUNCE_MS`

**Файл:** `config.h`

```cpp
// БЫЛО:
#define RECOVERY_DEBOUNCE_MS     400

// СТАЛО:
#define RECOVERY_DEBOUNCE_MS     250
```

**Эффект:** Роботу нужно держаться вертикально не 400 мс, а 250 мс. Рука успевает выровнять быстрее.

---

### ✅ Решение 1C (Рекомендуемое) — Добавить режим «ручного сброса» через Serial

Добавить команду `R` (uppercase) в `Giro_robot.ino` для принудительного сброса `fall=1`, если пользователь уверен что робот стоит:

**Файл:** `Giro_robot.ino` — в блок обработки Serial-команд:

```cpp
else if (cmd == "R") {
    // Принудительный сброс аварии (только если угол уже близок к вертикали)
    if (abs(currentAngle - targetAngleOffset) < 25.0f) {
        fallHandler.forceRecover();   // нужно добавить метод в FallHandler
        resetAllControlState();
        motors.enable();
        Serial.println(F("forceOK"));
    } else {
        Serial.println(F("notVertical"));
    }
}
```

**В `safety.h` добавить метод:**

```cpp
void forceRecover() {
    _fallen = false;
    _fallStartMs = 0;
    _recoverStartMs = 0;
}
```

---

### ✅ Решение 1D — Улучшить окно recovery settle

В `config.h` и `architecture.md` описано окно `RECOVERY_SETTLE_MS (350 мс)` — в течение него Speed PID не обновляет `angleOffset`. Убедиться что параметры правильные:

```cpp
#define RECOVERY_SETTLE_MS              350
#define RECOVERY_SETTLE_MAX_VFUSED      0.08f   // м/с
#define RECOVERY_SETTLE_MAX_GYRO_DPS    45.0f   // °/с
```

Если после постановки робот «уезжает» — уменьшить `RECOVERY_SETTLE_MAX_VFUSED` до `0.05f`.

---

### 📋 Чек-лист исправления проблемы 1

- [ ] В `config.h`: изменить `FALL_RECOVERY_RATE_MAX` с 60 на 100
- [ ] В `config.h`: изменить `RECOVERY_DEBOUNCE_MS` с 400 на 250
- [ ] В `safety.h`: добавить метод `forceRecover()`
- [ ] В `Giro_robot.ino`: добавить обработку команды `R`
- [ ] Обновить `doc/safety.md` с новыми порогами
- [ ] Обновить `doc/commands.md` — добавить команду `R`
- [ ] Тест: поднять рукой → `?` → должен показать `fall=0` в течение ~1–2 сек
- [ ] Тест: намеренно упасть → `?` → `fall=1` → поставить плавно → `fall=0`

---

---

## 🟡 ПРОБЛЕМА 2: Нет поворотов влево/вправо, только вперёд/назад

### Диагноз

Команда `V,speed,turn` уже поддерживается в прошивке (см. `commands.md`).  
Модуль `twist_control.h` принимает `turn` в диапазоне `-1..1`.  
**Проблема в том, что `TURN_SCALE` и связанные константы в `config.h` могут быть занулены или слишком малы**, либо `MOTOR_DRIFT_CORRECTION` компенсирует поворот.

Также в `troubleshooting.md` прямо указано:
- `TURN_SCALE` — если дёргается при повороте: уменьшить до 0.3–0.5
- `TURN_BALANCE_BLEND` — для вращения вокруг оси: 0.1–0.3

---

### ✅ Решение 2A — Проверить и выставить `TURN_SCALE` в `config.h`

```cpp
// Проверить что НЕ равно нулю:
#define TURN_SCALE              0.4f    // Масштаб поворота (0 = поворот выключен!)

// Если робот крутится слишком резко:
#define TURN_SCALE              0.3f

// Если поворот едва заметен:
#define TURN_SCALE              0.6f
```

---

### ✅ Решение 2B — Проверить `twist_control.h` — реализацию поворота

В `twist_control.h` должна быть логика дифференциального управления:

```cpp
// Поворот реализуется разностью скоростей левого и правого мотора:
float leftSpeed  = linearSpeed + turnRate * TURN_SCALE;
float rightSpeed = linearSpeed - turnRate * TURN_SCALE;
```

Убедиться, что `turn` из команды `V,speed,turn` реально передаётся в моторы.

---

### ✅ Решение 2C — Доработка системы траектории

Добавить в `twist_control.h` / `Giro_robot.ino` поддержку **траекторных команд**:

#### 2C.1 — Команда «повернуть на N градусов»

```
T,<degrees>       # Повернуть на N градусов (+ вправо, - влево)
T,90              # Повернуть вправо на 90°
T,-45             # Повернуть влево на 45°
```

**Реализация в `Giro_robot.ino`:**

```cpp
else if (cmd.startsWith("T,")) {
    float deg = cmd.substring(2).toFloat();
    trajectoryTurn(deg);   // новая функция
}
```

**Новая функция `trajectoryTurn(float degrees)`:**

```cpp
void trajectoryTurn(float degrees) {
    // Оценка: при TURN_SCALE=0.4 и скорости turn=0.5
    // угловая скорость ≈ вычисляется экспериментально
    float turnRate = (degrees > 0) ? 0.5f : -0.5f;
    float duration = abs(degrees) / TURN_RATE_DEG_PER_SEC;  // калибровать!
    twist.setTarget(0, turnRate);
    turnEndTime = millis() + (unsigned long)(duration * 1000);
    turningActive = true;
}
```

#### 2C.2 — Команда «ехать по квадрату / по кругу»

```
TQ,<side_m>       # Проехать квадрат со стороной side_m метров
TC,<radius_m>     # Ехать по кругу с радиусом radius_m
```

#### 2C.3 — Waypoint-режим (продвинутый)

```
WP,<x1>,<y1>,<x2>,<y2>,...   # Задать маршрут через точки (в метрах)
```

**Требуется добавить модуль `odometry.h`:**

```cpp
// Интегрирует шаги моторов → X, Y, heading
class Odometry {
    float _x, _y, _heading;
public:
    void update(float leftSteps, float rightSteps, float dt);
    float getX() { return _x; }
    float getY() { return _y; }
    float getHeading() { return _heading; }
    void reset();
};
```

---

### 📋 Чек-лист исправления проблемы 2

- [ ] В `config.h`: проверить `TURN_SCALE` — должен быть 0.3–0.6 (НЕ 0)
- [ ] В `config.h`: проверить `MOTOR_DRIFT_CORRECTION` — не компенсирует ли поворот
- [ ] Тест базовый: `V,0,0.5` → робот должен вращаться вправо на месте
- [ ] Тест базовый: `V,0,-0.5` → робот должен вращаться влево на месте
- [ ] Тест комбо: `V,0.3,0.3` → движение вперёд с поворотом вправо
- [ ] Добавить команду `T,<degrees>` для точного поворота
- [ ] Создать `odometry.h` для отслеживания положения
- [ ] Добавить `doc/trajectory.md` с описанием новых команд
- [ ] Обновить `doc/commands.md`

---

---

## 💡 ЧАСТЬ 2 — ИДЕИ ПО УЛУЧШЕНИЮ

---

### 🚀 Улучшение 1 — Беспроводное управление (Bluetooth / WiFi)

**Проблема:** Сейчас управление только по USB Serial — кабель мешает движению.

**Решение:** Добавить модуль **HC-05 (Bluetooth)** или **ESP8266/ESP32** (WiFi).

- HC-05 подключается на UART (TX/RX), работает прозрачно — команды те же
- На телефоне: приложение Serial Bluetooth Terminal (Android) или BlueSee (iOS)
- Все Serial-команды (`V,0.3,0`, `s`, `?`) работают без изменений прошивки

```
Пины: HC-05 TX → Arduino RX (D0), HC-05 RX → Arduino TX (D1)
Скорость: настроить HC-05 на 115200 бод командой AT+UART=115200,0,0
```

---

### 🚀 Улучшение 2 — Одометрия и самоориентация

Добавить модуль `odometry.h` для отслеживания позиции (X, Y, угол):

```cpp
// В каждом цикле:
odometry.update(motor1Steps, motor2Steps, dt);

// Новая команда для чтения позиции:
// "O" → выводит "ODO x=0.34 y=-0.12 h=45.2"
// "O,reset" → обнуляет одометрию
```

**Применение:** Можно задать «вернуться на старт», квадратный маршрут, избегание препятствий.

---

### 🚀 Улучшение 3 — Ультразвуковой датчик (HC-SR04) — избегание препятствий

```cpp
// config.h:
#define OBSTACLE_DISTANCE_CM    25   // тормозить если препятствие ближе 25 см
#define SONAR_TRIG_PIN          D6
#define SONAR_ECHO_PIN          D7
```

**Логика в главном цикле:**

```cpp
if (sonar.getDistanceCm() < OBSTACLE_DISTANCE_CM) {
    twist.stop();   // автостоп
    Serial.println(F("OBSTACLE"));
}
```

---

### 🚀 Улучшение 4 — Аварийная кнопка (физический стоп)

Добавить кнопку на корпус, подключённую к прерыванию:

```cpp
// config.h:
#define EMERGENCY_STOP_PIN   D2   // INT0 — аппаратное прерывание

// setup():
attachInterrupt(digitalPinToInterrupt(EMERGENCY_STOP_PIN), emergencyStop, FALLING);

void emergencyStop() {
    motors.stop();
    motors.disable();
    emergencyActive = true;
}
```

---

### 🚀 Улучшение 5 — Телеметрия на дисплей (OLED 0.96")

Подключить OLED I2C дисплей (SSD1306) на те же шины A4/A5 (адрес 0x3C):

```
Отображать:
- Текущий pitch (угол)
- Скорость (м/с)
- Статус fall (OK / FALL)
- Режим (BALANCE / DRIVE / TURN)
- Заряд батареи (если есть делитель напряжения)
```

**Библиотека:** `Adafruit SSD1306` + `Adafruit GFX`

---

### 🚀 Улучшение 6 — Мониторинг батареи

Добавить делитель напряжения на аналоговый пин:

```cpp
#define BATTERY_PIN   A3
#define BATTERY_LOW_V 7.0f   // порог предупреждения

float readBattery() {
    int raw = analogRead(BATTERY_PIN);
    return raw * (5.0f / 1023.0f) * VOLTAGE_DIVIDER_RATIO;
}
```

При низком заряде — замедлить скорость и вывести в Serial `BATT_LOW`.

---

### 🚀 Улучшение 7 — PID с адаптивным Kp

При больших ошибках угла автоматически усиливать реакцию:

```cpp
// В stabilizer.h:
float adaptiveKp = _kp;
if (abs(angleError) > 10.0f) {
    adaptiveKp = _kp * 1.3f;   // +30% при ошибке > 10°
}
```

---

### 🚀 Улучшение 8 — Логирование в EEPROM (чёрный ящик)

Записывать последние N падений с timestamp и углом:

```cpp
struct FallEvent {
    uint32_t timestamp_ms;
    float    angle_at_fall;
    uint8_t  cause;   // 0=angle, 1=lifted, 2=obstacle
};
// Хранить кольцевой буфер на 10 событий в EEPROM
```

Команда `L` — вывести лог падений в Serial.

---

---

## ЧАСТЬ 3 — ПРОМТ ДЛЯ AI-АССИСТЕНТА

---

```
===========================================================================
СИСТЕМНЫЙ ПРОМТ ДЛЯ AI-АССИСТЕНТА: GIRO-ROBOT
===========================================================================

Ты — AI-ассистент по разработке прошивки для балансирующего двухколёсного
робота Giro-Robot. Ниже — полное описание проекта. Используй его при
ответах на вопросы о коде, архитектуре, ошибках и улучшениях.

---

## СТЕК И ЖЕЛЕЗО

- Контроллер: Arduino Nano (ATmega328P, 16 МГц, 32KB Flash, 2KB RAM)
- IMU: MPU6050 (I2C, адрес 0x68, шины A4/A5)
- Драйверы: TMC2208 ×2 (16 микрошагов, 200 шаг/об)
- Пины мотор1: STEP=D3, DIR=D5 | мотор2: STEP=D9, DIR=D10
- Enable оба мотора: D8 (активный LOW)
- Прошивка: C++ Arduino framework
- Serial: 115200 бод

---

## АЛГОРИТМ УПРАВЛЕНИЯ

Каскадный PID (два контура):

1. Speed PID (внешний): target_speed (м/с) → ошибка скорости → angleOffset (°)
2. Angle PID (внутренний): (targetOffset + angleOffset) → ошибка угла → motorSpeed (шаг/с)

Оценка угла: комплементарный фильтр (orientation.h) — акселерометр + гироскоп.
Оценка скорости: velocity_fusion.h — фузия шагов моторов и интеграла IMU.

---

## КЛЮЧЕВЫЕ МОДУЛИ

| Файл               | Роль |
|--------------------|------|
| Giro_robot.ino     | Главный цикл, Serial, safety dispatcher, каскад |
| config.h           | ВСЕ константы, пины, пороги, коэффициенты |
| pid.h              | PID с антивиндапом + полный reset() |
| stabilizer.h       | Angle PID (внутренний контур) |
| speed_controller.h | Speed PID + EMA-сглаживание angleOffset |
| safety.h           | FallHandler — детекция fall/recovery |
| orientation.h      | Комплементарный фильтр pitch |
| velocity_fusion.h  | Фузия скорости |
| motors.h           | ISR Timer1/Timer2 для шаговиков |
| twist_control.h    | Целевая скорость + поворот + ramp |
| calibration.h      | 6-позиционная калибровка IMU |
| pid_eeprom.h       | EEPROM: PID / калибровка / Hz |
| speed_motor.h      | STEPS_PER_REV=3200, WHEEL_DIAMETER=0.078м |

---

## SERIAL-КОМАНДЫ (основные)

V,speed_mps,turn   — движение (turn: -1..1)
V,0,0              — баланс на месте
s / S              — полный стоп
c                  — калибровка IMU (6 позиций)
z                  — запомнить нуль (2 сек вертикально)
f,kp,ki,kd,lim     — Angle PID
f2,kp,ki,kd,lim    — Speed PID
w / w2             — сохранить PID в EEPROM
P / P2             — показать PID
D                  — вкл/выкл отладку
?                  — статус: fall=0/1 hz=NN atz=M
x,1 / x,0          — телеметрия TLM,... (CSV)
a,0/1/2            — автотюн (0=выкл, 1=каскад, 2=только Angle)
r                  — сброс контуров (rOK)
H,<hz>             — частота цикла 25–200 Гц
e                  — очистить EEPROM

---

## SAFETY (ПАДЕНИЕ И ВОССТАНОВЛЕНИЕ)

Единственное аварийное состояние: fallen (bool).

Переход NORMAL → FALLEN:
  |angle - target| > FALL_ANGLE_DEG (55°) в течение FALL_DEBOUNCE_MS (200 мс)
  → motors.stop(), motors.disable(), resetAllControlState()

Переход FALLEN → NORMAL:
  |angle - target| < FALL_RECOVERY_DEG (15°)
  И |gyroRate| < FALL_RECOVERY_RATE_MAX (60 °/с)
  В течение RECOVERY_DEBOUNCE_MS (400 мс)
  → resetAllControlState(), motors.enable()

После восстановления: окно RECOVERY_SETTLE_MS (350 мс) — Speed PID заморожен.

ВАЖНАЯ ИЗВЕСТНАЯ ПРОБЛЕМА:
При поднятии робота рукой он уходит в fall=1 и не выходит, пока рука
держит его в воздухе (gyroRate слишком высокий). Выходит только когда
его ставят на пол плавно. Решение: увеличить FALL_RECOVERY_RATE_MAX до
100 °/с и/или добавить команду R для принудительного сброса fall.

---

## ПОВОРОТ (ИЗВЕСТНАЯ ПРОБЛЕМА)

Команда V,speed,turn принимается, turn обрабатывается в twist_control.h.
Если поворот не работает — проверить TURN_SCALE в config.h (должно быть
0.3–0.6, НЕ 0). MOTOR_DRIFT_CORRECTION не должен компенсировать поворот.
Тест: V,0,0.5 → вращение вправо на месте.

---

## ПРАВИЛА ДЛЯ AI-АССИСТЕНТА

1. Константы и пороги — ТОЛЬКО в config.h с комментариями.
2. Весь сброс контуров — ТОЛЬКО через resetAllControlState() в Giro_robot.ino.
3. Аварийное состояние — ЕДИНСТВЕННОЕ: fallen в safety.h. Не добавлять отдельную lift-логику.
4. Не менять имена Serial-команд без миграции EEPROM.
5. PID::reset() должен очищать телеметрию (_lastP/I/D, _lastErrorOut).
6. При любом изменении порогов/поведения — обновлять doc/*.md.
7. Используй Conventional Commits: feat:, fix:, refactor:, docs:.
8. Новые модули (odometry.h, sonar.h и т.д.) — отдельные файлы .h.
9. Учитывай ограничения Nano: 32KB Flash, 2KB RAM — избегай String, malloc, heavy libs.
10. При добавлении команд — обновлять doc/commands.md и README.md.

---

## ПРИОРИТЕТНЫЕ ЗАДАЧИ ДЛЯ РАЗРАБОТКИ

### ЗАДАЧА 1 (критическая): Исправить fall при поднятии рукой
  - Файл: config.h — FALL_RECOVERY_RATE_MAX: 60 → 100
  - Файл: config.h — RECOVERY_DEBOUNCE_MS: 400 → 250
  - Файл: safety.h — добавить forceRecover()
  - Файл: Giro_robot.ino — добавить команду "R" (force recover)

### ЗАДАЧА 2 (высокая): Включить и протестировать повороты
  - Проверить TURN_SCALE в config.h (не 0!)
  - Протестировать V,0,0.5 и V,0,-0.5
  - Если не работает — отладить twist_control.h

### ЗАДАЧА 3 (средняя): Добавить команду T,<degrees> для точного поворота
  - Добавить обработку в Giro_robot.ino
  - Калибровать TURN_RATE_DEG_PER_SEC экспериментально

### ЗАДАЧА 4 (средняя): Одометрия (odometry.h)
  - Интеграция шагов → X, Y, heading
  - Команда "O" для чтения позиции

### ЗАДАЧА 5 (желательная): Bluetooth управление (HC-05)
  - Подключить на UART, скорость 115200
  - Прошивка не меняется

===========================================================================
```

---

## ЧАСТЬ 4 — КАК ИСПОЛЬЗОВАТЬ ЭТОТ ДОКУМЕНТ

### Для работы с AI-ассистентом (ChatGPT / Claude / Gemini):

1. Скопируй блок из **ЧАСТИ 3** (между строками `=====`)
2. Вставь его в начало разговора с AI как **системный контекст**
3. После этого задавай конкретные вопросы, например:
   - *«Покажи полный код изменений в config.h для исправления проблемы с fall»*
   - *«Напиши функцию trajectoryTurn() для поворота на N градусов»*
   - *«Как добавить HC-05 Bluetooth не меняя прошивку?»*

### Порядок работы по плану:

```
Шаг 1 → Исправить fall при поднятии (Проблема 1)
        Изменить config.h + safety.h + Giro_robot.ino

Шаг 2 → Проверить поворот V,0,0.5 (Проблема 2A)
        Если работает → переходить к Шагу 3
        Если нет → отладить twist_control.h

Шаг 3 → Добавить команду T,<degrees>
        Откалибровать скорость поворота

Шаг 4 → Добавить odometry.h

Шаг 5 → Подключить HC-05 Bluetooth

Шаг 6 → Остальные улучшения по желанию
```

---

*Документ создан на основе: AGENTS.md, README.md, architecture.md, safety.md, commands.md, hardware.md, troubleshooting.md, pid_tuning.md, calibration.md, autotune.md*
