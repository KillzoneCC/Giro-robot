# Архитектура Giro-Robot

Балансирующий гиро-робот (перевёрнутый маятник) на двух шаговых моторах с
ориентацией по IMU. Управление — каскад из двух PID-контуров.

## Каскад Speed PID → Angle PID

```
target_speed (м/с)
      │
      ▼
┌──────────────┐   angleOffset (°)   ┌──────────────┐
│  Speed PID   │ ──────────────────▶ │  Angle PID   │ ──▶ motorSpeed (шаг/с)
│ (внешний)    │                     │ (внутренний) │
└──────────────┘                     └──────────────┘
      ▲                                    ▲
      │ current_speed (м/с, фузия)         │ current_angle (°, IMU)
      │
  velocity_fusion.h                   orientation.h
```

- **Speed PID** ([speed_controller.h](../code/Giro_robot/Giro_robot/speed_controller.h)):
  получает целевую скорость (м/с) и текущую оценку (м/с), выдаёт дополнительный угол
  наклона. Положительный выход → робот наклоняется вперёд, ускоряется. 0 → держит
  баланс на месте.
- **Angle PID** ([stabilizer.h](../code/Giro_robot/Giro_robot/stabilizer.h)):
  получает целевой угол `targetOffset + angleOffset` и текущий угол pitch,
  выдаёт команду на моторы (шаг/с).

## Модули прошивки

Все файлы — в `code/Giro_robot/Giro_robot/`.

| Модуль | Ответственность |
|---|---|
| `Giro_robot.ino` | Главный цикл, Serial-команды, диспетчер safety, каскад |
| `config.h` | Константы: пины, пороги, коэффициенты по умолчанию |
| `pid.h` | Универсальный PID с антивиндапом и полным `reset()` |
| `stabilizer.h` | Angle PID (внутренний контур) |
| `speed_controller.h` | Speed PID + EMA-сглаживание `angleOffset` |
| `orientation.h` | Комплементарный фильтр для pitch (accel + gyro) |
| `velocity_fusion.h` | Комплементарная фузия скорости (колёса + интеграл IMU) |
| `imu_sensor.h` | Чтение MPU6050, применение калибровки |
| `motors.h` | Шаговые моторы на Timer1/Timer2 (ISR) |
| `twist_control.h` | Целевые скорость+поворот, rate-limit ramp |
| `calibration.h` | Полная калибровка IMU (6 позиций) |
| `pid_eeprom.h` | Сохранение/загрузка PID, калибровки и Hz в EEPROM |
| `safety.h` | Детекция падения и восстановления (см. [safety.md](safety.md)) |
| `speed_motor.h` | Пересчёт шаг/с ↔ м/с |

## Частота цикла

`CONTROL_LOOP_HZ = 100` по умолчанию (задаётся командой `H,<hz>`, сохраняется в EEPROM).
Диапазон 25–200 Гц.

## Сброс накоплений

Весь сброс состояния контуров идёт через единую функцию
`resetAllControlState()` в [Giro_robot.ino](../code/Giro_robot/Giro_robot/Giro_robot.ino).
Она вызывается на обоих событиях safety (`FALL_JUST_FELL` и `FALL_JUST_RECOVERED`) и
после калибровки `c`. Выполняет:

- `stabilizer.reset()` — Angle PID: интеграл, производная, телеметрия, motorSpeed.
- `speedController.reset()` — Speed PID + `smoothOffset` + `prevTargetSign`.
- `velocityFusion.setVelocity(0)` — обнуление интеграла IMU-скорости.
- `twist.stop()` — сброс целевой скорости и rate-limit ramp.

Накопленного состояния за пределами этих классов (static-локали в `loop`) нет —
кроме `zeroCount` в счётчике холостого хода, который авто-восстанавливается.
