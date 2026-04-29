"""
Host-driven автотюн PID для Giro-Robot: оркестрация на ПК.
Прошивка: a,<n>, x,<n>, r; Angle — f,...; Speed — f2,...

Пошаговый sweep Kp с паузой при fall и откатом по пересечениям ошибки (Angle: angE, Speed: spdE).
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
import time
from typing import Optional


class AutotuneState(Enum):
    IDLE = auto()
    RUNNING = auto()
    PAUSED_FALL = auto()


class PidRole(Enum):
    ANGLE = auto()
    SPEED = auto()


class SweepPlan(Enum):
    ANGLE_ONLY = auto()
    SPEED_ONLY = auto()
    BOTH = auto()


@dataclass
class SweepParams:
    kp_start: float = 280.0
    kp_end: float = 400.0
    kp_step: float = 10.0
    ki: float = 0.005
    kd: float = 0.0045
    limit: float = 7500.0
    step_duration_s: float = 3.0
    max_crossings: int = 12


class AutotuneController:
    """Вызывать методы только из потока UI Tkinter."""

    def __init__(self, app):
        self.app = app
        self.state = AutotuneState.IDLE
        self.plan = SweepPlan.ANGLE_ONLY
        self.pid_role = PidRole.ANGLE
        self.angle_only_for_angle_phase = True
        self.params = SweepParams()
        self.speed_params_stored: Optional[SweepParams] = None
        self.current_kp = 0.0
        self.last_good_kp: Optional[float] = None
        self.prev_err_sign: Optional[float] = None
        self.crossings = 0
        self._timer_id = None
        # Итог Kp после фазы Angle (режим BOTH), для подсказки на панели Speed
        self.angle_phase_saved_kp: Optional[float] = None
        self._last_live_panel_ts = 0.0

    def log(self, msg: str, channel: Optional[str] = None) -> None:
        """channel: None → по текущей фазе (Angle/Speed); 'angle' | 'speed' | 'both'."""
        if hasattr(self.app, "_log_autotune"):
            self.app._log_autotune(msg, channel)
        elif hasattr(self.app, "_log"):
            self.app._log(msg, "recv")

    def _refresh_live_panel_throttled(self) -> None:
        now = time.monotonic()
        if now - self._last_live_panel_ts < 0.2:
            return
        self._last_live_panel_ts = now
        try:
            self.app.root.after(0, self.app._update_autotune_live_panel)
        except Exception:
            pass

    def _send(self, line: str) -> bool:
        sp = getattr(self.app, "serial_port", None)
        if not sp or not sp.is_open:
            return False
        try:
            sp.write(line.encode("utf-8"))
            sp.flush()
            self.app._log(f">>> {line.strip()}\n", "sent")
            return True
        except Exception:
            return False

    def _cancel_timer(self) -> None:
        if self._timer_id is not None:
            try:
                self.app.root.after_cancel(self._timer_id)
            except Exception:
                pass
            self._timer_id = None

    def reset_counters(self) -> None:
        self.prev_err_sign = None
        self.crossings = 0

    def feed_fall_marker(self) -> None:
        if self.state != AutotuneState.RUNNING:
            return
        self.state = AutotuneState.PAUSED_FALL
        self._cancel_timer()
        self.log(
            "AUTOTUNE: fall — фаза на паузе (поднимите робота, затем «Продолжить»)\n",
            "both",
        )
        try:
            self.app.root.after(0, lambda: self.app.at_status_var.set("Автотюн: пауза (fall)"))
            self.app.root.after(0, self.app._update_autotune_live_panel)
        except Exception:
            pass

    def feed_tlm_csv(self, line: str) -> None:
        if self.state != AutotuneState.RUNNING:
            return
        parts = [p.strip() for p in line.strip().split(",")]
        if len(parts) < 21 or parts[0] != "TLM":
            return
        err_idx = 10 if self.pid_role == PidRole.ANGLE else 6
        try:
            err = float(parts[err_idx])
        except (ValueError, IndexError):
            return
        s = 1.0 if err > 1e-6 else (-1.0 if err < -1e-6 else 0.0)
        if self.prev_err_sign is not None and s != 0 and self.prev_err_sign != 0:
            if s != self.prev_err_sign:
                self.crossings += 1
        if s != 0:
            self.prev_err_sign = s
        self._refresh_live_panel_throttled()

    def _serial_mode_a(self) -> int:
        """Режим прошивки a,N: Speed всегда каскад (1); Angle — 2 только внутренний контур при sweep угла."""
        if self.pid_role == PidRole.SPEED:
            return 1
        return 2 if self.angle_only_for_angle_phase else 1

    def _apply_current_kp(self) -> None:
        p = self.params
        kp = self.current_kp
        if self.pid_role == PidRole.ANGLE:
            self._send(f"f,{kp},{p.ki},{p.kd},{p.limit}\n")
        else:
            self._send(f"f2,{kp},{p.ki},{p.kd},{p.limit}\n")
        role = "Angle PID" if self.pid_role == PidRole.ANGLE else "Speed PID"
        ch = "angle" if self.pid_role == PidRole.ANGLE else "speed"
        self.log(
            f"[{role}] на роботе: Kp={kp:g}  Ki={p.ki:g}  Kd={p.kd:g}  L={p.limit:g}  "
            f"| пересечений ошибки: {self.crossings}/{p.max_crossings}\n",
            ch,
        )
        try:
            self.app.root.after(0, self.app._update_autotune_live_panel)
        except Exception:
            pass

    def start(
        self,
        plan: SweepPlan,
        angle_only_inner: bool,
        angle_sp: SweepParams,
        speed_sp: Optional[SweepParams] = None,
    ) -> bool:
        if self.state == AutotuneState.RUNNING:
            return False

        self.plan = plan
        self.angle_only_for_angle_phase = angle_only_inner
        self.speed_params_stored = speed_sp

        if plan == SweepPlan.SPEED_ONLY:
            vals = self.app._get_speed_pid_values()
            if vals is None:
                return False
            self.pid_role = PidRole.SPEED
            self.params = SweepParams(
                kp_start=speed_sp.kp_start,
                kp_end=speed_sp.kp_end,
                kp_step=speed_sp.kp_step,
                ki=speed_sp.ki,
                kd=speed_sp.kd,
                limit=speed_sp.limit,
                step_duration_s=speed_sp.step_duration_s,
                max_crossings=speed_sp.max_crossings,
            )
            self.last_good_kp = vals[0]
            self.current_kp = self.params.kp_start
            self.reset_counters()
            self.state = AutotuneState.RUNNING
            self._send("x,1\n")
            self._send("a,1\n")
            self.log(
                f"AUTOTUNE Speed: Kp {self.params.kp_start}→{self.params.kp_end} шаг {self.params.kp_step} (f2, каскад a,1)\n",
                "speed",
            )
            self._apply_current_kp()
            self._schedule_step()
            return True

        # Угол (ANGLE_ONLY или первая фаза BOTH)
        vals = self.app._get_pid_values()
        if vals is None:
            return False
        if plan == SweepPlan.BOTH and speed_sp is None:
            return False

        self.pid_role = PidRole.ANGLE
        self.params = angle_sp
        self.last_good_kp = vals[0]
        self.current_kp = angle_sp.kp_start
        self.reset_counters()
        self.state = AutotuneState.RUNNING

        self._send("x,1\n")
        self._send(f"a,{self._serial_mode_a()}\n")
        tag = "Angle a,2" if angle_only_inner else "Angle a,1"
        self.angle_phase_saved_kp = None
        self.log(
            f"AUTOTUNE Angle: Kp {angle_sp.kp_start}→{angle_sp.kp_end} шаг {angle_sp.kp_step} ({tag})\n",
            "angle",
        )
        self._apply_current_kp()
        self._schedule_step()
        return True

    def _begin_speed_phase(self) -> None:
        sp = self.speed_params_stored
        if sp is None:
            self.stop()
            return
        vals = self.app._get_speed_pid_values()
        if vals is None:
            self.stop()
            return

        self.pid_role = PidRole.SPEED
        self.params = SweepParams(
            kp_start=sp.kp_start,
            kp_end=sp.kp_end,
            kp_step=sp.kp_step,
            ki=sp.ki,
            kd=sp.kd,
            limit=sp.limit,
            step_duration_s=sp.step_duration_s,
            max_crossings=sp.max_crossings,
        )
        self.last_good_kp = vals[0]
        self.current_kp = self.params.kp_start
        self.reset_counters()
        self.state = AutotuneState.RUNNING

        self._send("x,1\n")
        self._send("a,1\n")
        self.log(
            f"AUTOTUNE: фаза Speed PID — Kp {self.params.kp_start}→{self.params.kp_end} шаг {self.params.kp_step} (f2)\n",
            "speed",
        )
        self._apply_current_kp()
        self._schedule_step()

    def stop(self) -> None:
        self._cancel_timer()
        self._send("a,0\n")
        self._send("x,0\n")
        self.state = AutotuneState.IDLE
        self.plan = SweepPlan.ANGLE_ONLY
        self.pid_role = PidRole.ANGLE
        self.speed_params_stored = None
        self.angle_phase_saved_kp = None
        self.log("AUTOTUNE: стоп (коэффициенты в RAM; EEPROM не изменён — сохраните w / w2 вручную)\n", "both")
        try:
            self.app.root.after(0, lambda: self.app.at_status_var.set("Автотюн: выкл"))
            self.app.root.after(0, self.app._update_autotune_live_panel)
        except Exception:
            pass

    def resume_after_fall(self) -> None:
        if self.state != AutotuneState.PAUSED_FALL:
            return
        self._send("r\n")
        self._send(f"a,{self._serial_mode_a()}\n")
        self._send("x,1\n")
        self.reset_counters()
        self.state = AutotuneState.RUNNING
        self.log("AUTOTUNE: продолжение после r\n", "both")
        self.app.root.after(400, self._schedule_step)

    def _schedule_step(self) -> None:
        self._cancel_timer()
        if self.state != AutotuneState.RUNNING:
            return
        p = self.params
        self._timer_id = self.app.root.after(int(p.step_duration_s * 1000), self._step_tick)

    def _step_tick(self) -> None:
        self._timer_id = None
        if self.state != AutotuneState.RUNNING:
            return
        p = self.params

        if self.crossings >= p.max_crossings:
            if self.last_good_kp is not None:
                role_name = "угла" if self.pid_role == PidRole.ANGLE else "скорости"
                ch = "angle" if self.pid_role == PidRole.ANGLE else "speed"
                self.log(
                    f"AUTOTUNE: откат к Kp={self.last_good_kp} (осцилляции по ошибке {role_name})\n",
                    ch,
                )
                self.current_kp = self.last_good_kp
                self._apply_current_kp()
        else:
            self.last_good_kp = self.current_kp

        next_kp = self.current_kp + p.kp_step
        if next_kp > p.kp_end + 1e-6:
            if self.plan == SweepPlan.BOTH and self.pid_role == PidRole.ANGLE:
                self.angle_phase_saved_kp = self.last_good_kp
                self.log(
                    f"AUTOTUNE: фаза Angle завершена → Speed PID (итог Angle Kp≈{self.angle_phase_saved_kp})\n",
                    "angle",
                )
                self.log(
                    f"AUTOTUNE: старт фазы Speed после Angle (рекомендуемый Kp угла в RAM: {self.angle_phase_saved_kp})\n",
                    "speed",
                )
                self._begin_speed_phase()
                return
            self.log(
                "AUTOTUNE: sweep завершён\n",
                "speed" if self.pid_role == PidRole.SPEED else "angle",
            )
            self.stop()
            return

        self.current_kp = next_kp
        self._apply_current_kp()
        self.reset_counters()
        self._schedule_step()
