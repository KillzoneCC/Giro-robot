// /*
//   Заготовка маршрута для балансирующего робота.
//   Маршрут — последовательность сегментов: целевой угол (roll/pitch) и время удержания в мс.
//   Робот наклоняется к заданному углу → колёса едут в нужную сторону.
// */
// #ifndef SERIAL_BAUD
// #define SERIAL_BAUD 115200
// #endif
// /*
//   Использование в основном скетче:
//   1. #include "route.h"
//   2. В loop() вызывать routeUpdate() и использовать routeGetTargetRoll() / routeGetTargetPitch()
//      вместо запоминания позиции, либо подменять target_roll_deg/target_pitch_deg из маршрута.
//   3. Либо вручную: раз в цикл проверять, прошло ли duration_ms текущего сегмента,
//      затем переходить к следующему и выставлять target по route[i].
// */

// #ifndef ROUTE_H
// #define ROUTE_H

// #include "Arduino.h"

// // Один сегмент маршрута: целевые углы (град) и длительность (мс)
// struct RouteSegment {
//   float target_roll_deg;
//   float target_pitch_deg;
//   uint32_t duration_ms;
// };

// // Длина маршрута (число сегментов)
// #define ROUTE_LEN 6

// // Маршрут: можно менять углы и длительности под свою механику и USE_PITCH_FOR_BALANCE
// // Пример: стой 3 с → наклон вперёд (pitch) 2 с → стой 2 с → наклон назад 2 с → стой 3 с → конец
// static const RouteSegment ROUTE[ROUTE_LEN] = {
//   {  0.0f,  0.0f, 3000 },   // 0: стой ровно 3 сек
//   {  0.0f,  4.0f, 2000 },   // 1: наклон вперёд (pitch +4°) — едет вперёд
//   {  0.0f,  0.0f, 2000 },   // 2: стой 2 сек
//   {  0.0f, -4.0f, 2000 },   // 3: наклон назад (pitch -4°) — едет назад
//   {  0.0f,  0.0f, 3000 },   // 4: стой 3 сек
//   {  0.0f,  0.0f, 0    }    // 5: конец (duration 0 = не переключаться дальше)
// };

// // Состояние проигрывателя маршрута (объяви в .ino или здесь static в функциях)
// // route_segment_index, route_segment_start_ms — задаётся снаружи или через routeStart()

// static inline void routeGetTarget(uint8_t segmentIndex, float* out_roll, float* out_pitch) {
//   if (segmentIndex >= ROUTE_LEN) {
//     *out_roll  = 0.0f;
//     *out_pitch = 0.0f;
//     return;
//   }
//   *out_roll  = ROUTE[segmentIndex].target_roll_deg;
//   *out_pitch = ROUTE[segmentIndex].target_pitch_deg;
// }

// static inline uint32_t routeGetDurationMs(uint8_t segmentIndex) {
//   if (segmentIndex >= ROUTE_LEN) return 0;
//   return ROUTE[segmentIndex].duration_ms;
// }

// /*
//   ПРИМЕР ВСТАВКИ В loop() (после balance_target_memorized):
//   ---
//   static uint8_t route_idx = 0;
//   static uint32_t route_start_ms = 0;
//   if (route_idx == 0) route_start_ms = millis();
//   if (route_idx < ROUTE_LEN && routeGetDurationMs(route_idx) > 0 &&
//       (uint32_t)(millis() - route_start_ms) >= routeGetDurationMs(route_idx)) {
//     route_idx++;
//     route_start_ms = millis();
//   }
//   if (route_idx < ROUTE_LEN) {
//     target_roll_deg  = ROUTE[route_idx].target_roll_deg;
//     target_pitch_deg = ROUTE[route_idx].target_pitch_deg;
//   }
//   ---
// */

// #endif
