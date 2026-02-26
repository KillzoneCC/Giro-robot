# Giro-Robot

Балансирующий гиро-робот на Arduino. Ориентация по pitch, один PID.

## Структура проекта

```
Giro_robot/
├── Giro_robot.ino   # Главный файл
├── config.h         # Параметры
├── orientation.h    # Угол pitch (комплементарный фильтр)
├── imu_sensor.h     # MPU6050
├── pid.h            # PID-регулятор
├── stabilizer.h     # Стабилизация (угол → моторы)
├── motors.h         # Шаговики
├── twist_control.h  # linear, turn
├── calibration.h    # Калибровка IMU (6 поз)
├── pid_eeprom.h     # PID в EEPROM
└── fall_handler.h   # Падение
```

## Зависимости

- **Adafruit MPU6050** (`Adafruit_MPU6050`)
- **Adafruit Unified Sensor** (`Adafruit_Sensor`)
- **Wire** (встроено)

Установка через Arduino Library Manager: Adafruit MPU6050, Adafruit Unified Sensor.

## Команды Serial (115200 бод)

| Команда | Описание |
|---------|----------|
| `v,linear,turn` | Twist: linear [-1..1] вперёд/назад, turn [-1..1] влево/вправо |
| `s` | Стоп |
| `c` | Полная калибровка IMU (6 позиций) |
| `z` | Запомнить нуль (держать 2 сек) |
| `z,val` | Установить нуль вручную (град) |
| `p,val` `i,val` `d,val` `l,val` | Fine-tune: установить Kp, Ki, Kd, limit в реальном времени |
| `f,kp,ki,kd,lim` | Установить все коэффициенты PID сразу |
| `P` | Вывести текущие коэффициенты PID |
| `w` | Сохранить PID в EEPROM |
| `D` | Вкл/выкл отладку |
| `e` | Очистить EEPROM (IMU + PID) |

Пример: `v,0.5,0` — вперёд. `p,300` — Kp=300. `f,280,0.005,0.0045,7500` — все PID. `w` — сохранить в EEPROM.

## Калибровка

**При пустом EEPROM:** робот ждёт команды `c`. Отправьте `c` — полная калибровка IMU (6 позиций). Базовый PID из config. Подстройка: `p,val` `i,val` `d,val` `l,val`, `w` — сохранить.

**При наличии данных в EEPROM:** отдельные калибровки только вручную:
- `c` — калибровка IMU
- `z` — запомнить нуль

## Характеристики робота

- Диаметр колеса: 72 мм
- Масса: 1.1 кг
- Ширина: 134 мм, высота: 146 мм

## Плата

Arduino Nano. Пины: D3,D5 — мотор 1; D9,D10 — мотор 2. I2C — A4(SDA), A5(SCL).
