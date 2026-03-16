#!/usr/bin/env python3
"""
PID Tuner — приложение для управления PID-регулятором Giro-Robot
Подключение к Arduino через USB (Serial)
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext, filedialog
import serial
import json
import os
import serial.tools.list_ports
import threading
import queue
import re
from collections import deque

try:
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

BAUD_RATE = 115200
DEFAULT_KP = 280.0
DEFAULT_KI = 0.005
DEFAULT_KD = 0.0045
DEFAULT_LIMIT = 7500.0

# Для расчёта скорости м/с (если Arduino не передаёт — совпадать с speed_motor.h)
STEPS_PER_REV = 3200  # 200×16 микрошагов (TMC2208 microsteps(16))
WHEEL_DIAMETER_M = 0.078  # 78 мм
PI = 3.14159265358979

# Диапазоны для ползунков (Angle PID)
KP_RANGE = (0, 10000)
KI_RANGE = (0, 10000)
KD_RANGE = (0, 5)
LIMIT_RANGE = (100, 20000)

# Speed PID (внешний контур: скорость → угол)
DEFAULT_SPD_KP = 3.5
DEFAULT_SPD_KI = 0.05
DEFAULT_SPD_KD = 0.02
DEFAULT_SPD_LIMIT = 10.0
SPD_KP_RANGE = (0, 10000)
SPD_KI_RANGE = (0, 10000)
SPD_KD_RANGE = (0, 5)
SPD_LIMIT_RANGE = (1, 10000)

# Целевая скорость (м/с) — V,speed_mps,turn
MAX_TARGET_SPEED_MPS = 1.5  # как в config.h
SPEED_STEP_MPS = 0.05  # шаг при нажатии ←/→

# Шаги для клавиатурного управления ползунками PID
PID_STEP_OPTIONS = ("0.001", "0.01", "0.1", "1", "10", "100")

# Расширение и фильтр для файлов траекторий
TRAJECTORY_EXT = ".traj"
TRAJECTORY_FILTER = [("Траектории", f"*{TRAJECTORY_EXT}"), ("Все файлы", "*.*")]


class TrajectoryWindow:
    """Окно управления демо-траекториями: последовательность команд скорость+время."""

    def __init__(self, parent, app):
        self.parent = parent
        self.app = app
        self.commands = []  # [{"speed": float, "duration": float}, ...]
        self._run_after_id = None
        self._run_timer_id = None
        self._run_stop = False
        self._run_index = 0
        self._run_remaining = 0.0

        self.win = tk.Toplevel(parent)
        self.win.title("Управление траекториями (демки)")
        self.win.geometry("520x480")
        self.win.resizable(True, True)

        main = ttk.Frame(self.win, padding=10)
        main.pack(fill=tk.BOTH, expand=True)

        # Таблица команд
        tbl_frame = ttk.LabelFrame(main, text="Таблица команд", padding=5)
        tbl_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 5))

        cols = ("#", "speed", "duration")
        self.tree = ttk.Treeview(tbl_frame, columns=cols, show="headings", height=8, selectmode="browse")
        self.tree.heading("#", text="№")
        self.tree.heading("speed", text="Скорость (м/с)")
        self.tree.heading("duration", text="Время (сек)")
        self.tree.column("#", width=40)
        self.tree.column("speed", width=120)
        self.tree.column("duration", width=100)
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll = ttk.Scrollbar(tbl_frame, orient=tk.VERTICAL, command=self.tree.yview)
        scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.tree.configure(yscrollcommand=scroll.set)
        self.tree.bind("<Double-1>", self._on_row_double_click)

        # Кнопки управления таблицей
        btn_row = ttk.Frame(main)
        btn_row.pack(fill=tk.X, pady=(0, 5))
        ttk.Button(btn_row, text="Добавить", command=self._add_cmd).pack(side=tk.LEFT, padx=(0, 5))
        ttk.Button(btn_row, text="Изменить", command=self._edit_cmd).pack(side=tk.LEFT, padx=(0, 5))
        ttk.Button(btn_row, text="Удалить", command=self._delete_cmd).pack(side=tk.LEFT, padx=(0, 5))
        ttk.Button(btn_row, text="Очистить", command=self._clear_cmds).pack(side=tk.LEFT, padx=(0, 5))
        ttk.Button(btn_row, text="Загрузить", command=self._load_file).pack(side=tk.LEFT, padx=(0, 5))
        ttk.Button(btn_row, text="Сохранить", command=self._save_file).pack(side=tk.LEFT, padx=(0, 5))

        # Параметры для добавления
        param_frame = ttk.LabelFrame(main, text="Параметры новой команды", padding=5)
        param_frame.pack(fill=tk.X, pady=(0, 5))
        pr = ttk.Frame(param_frame)
        pr.pack(fill=tk.X)
        ttk.Label(pr, text="Скорость (м/с):").pack(side=tk.LEFT, padx=(0, 5))
        self.speed_entry = ttk.Entry(pr, width=10)
        self.speed_entry.pack(side=tk.LEFT, padx=(0, 10))
        self.speed_entry.insert(0, "0.5")
        ttk.Label(pr, text="Время (сек):").pack(side=tk.LEFT, padx=(10, 5))
        self.duration_entry = ttk.Entry(pr, width=10)
        self.duration_entry.pack(side=tk.LEFT, padx=(0, 5))
        self.duration_entry.insert(0, "5")

        # Кнопки выполнения
        run_row = ttk.Frame(main)
        run_row.pack(fill=tk.X, pady=(0, 5))
        self.run_btn = ttk.Button(run_row, text="Запустить", command=self._run_trajectory)
        self.run_btn.pack(side=tk.LEFT, padx=(0, 5))
        self.stop_btn = ttk.Button(run_row, text="Стоп", command=self._stop_trajectory, state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=(0, 5))

        # Статус
        self.status_var = tk.StringVar(value="Готов")
        ttk.Label(main, textvariable=self.status_var, font=("", 10)).pack(anchor=tk.W)

        self.win.protocol("WM_DELETE_WINDOW", self._on_close)

    def _refresh_table(self):
        for i in self.tree.get_children():
            self.tree.delete(i)
        for i, c in enumerate(self.commands, 1):
            self.tree.insert("", tk.END, values=(i, f"{c['speed']:.2f}", f"{c['duration']:.1f}"))

    def _add_cmd(self):
        try:
            speed = float(self.speed_entry.get())
            dur = float(self.duration_entry.get())
            speed = max(-MAX_TARGET_SPEED_MPS, min(MAX_TARGET_SPEED_MPS, speed))
            dur = max(0.0, dur)
            self.commands.append({"speed": speed, "duration": dur})
            self._refresh_table()
        except ValueError:
            messagebox.showwarning("Ошибка", "Введите число для скорости и времени")

    def _on_row_double_click(self, event):
        self._edit_cmd()

    def _edit_cmd(self):
        sel = self.tree.selection()
        if not sel:
            messagebox.showinfo("Подсказка", "Выберите строку для редактирования")
            return
        idx = self.tree.index(sel[0])
        if idx < 0 or idx >= len(self.commands):
            return
        c = self.commands[idx]
        dlg = tk.Toplevel(self.win)
        dlg.title("Изменить команду")
        dlg.geometry("280x120")
        dlg.transient(self.win)
        dlg.grab_set()
        f = ttk.Frame(dlg, padding=10)
        f.pack(fill=tk.BOTH, expand=True)
        ttk.Label(f, text="Скорость (м/с):").grid(row=0, column=0, sticky=tk.W, pady=2)
        e1 = ttk.Entry(f, width=12)
        e1.grid(row=0, column=1, padx=5, pady=2)
        e1.insert(0, str(c["speed"]))
        ttk.Label(f, text="Время (сек):").grid(row=1, column=0, sticky=tk.W, pady=2)
        e2 = ttk.Entry(f, width=12)
        e2.grid(row=1, column=1, padx=5, pady=2)
        e2.insert(0, str(c["duration"]))

        def ok():
            try:
                speed = float(e1.get())
                dur = float(e2.get())
                speed = max(-MAX_TARGET_SPEED_MPS, min(MAX_TARGET_SPEED_MPS, speed))
                dur = max(0.0, dur)
                self.commands[idx] = {"speed": speed, "duration": dur}
                self._refresh_table()
            except ValueError:
                messagebox.showwarning("Ошибка", "Введите числа")
                return
            dlg.destroy()

        def cancel():
            dlg.destroy()

        btn_f = ttk.Frame(f)
        btn_f.grid(row=2, column=0, columnspan=2, pady=10)
        ttk.Button(btn_f, text="OK", command=ok).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_f, text="Отмена", command=cancel).pack(side=tk.LEFT)
        dlg.wait_window()

    def _delete_cmd(self):
        sel = self.tree.selection()
        if not sel:
            return
        idx = self.tree.index(sel[0])
        if 0 <= idx < len(self.commands):
            self.commands.pop(idx)
            self._refresh_table()

    def _clear_cmds(self):
        self.commands.clear()
        self._refresh_table()

    def _save_file(self):
        path = filedialog.asksaveasfilename(defaultextension=TRAJECTORY_EXT, filetypes=TRAJECTORY_FILTER)
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(self.commands, f, indent=2)
            self.status_var.set(f"Сохранено: {os.path.basename(path)}")
        except Exception as e:
            messagebox.showerror("Ошибка", str(e))

    def _load_file(self):
        path = filedialog.askopenfilename(filetypes=TRAJECTORY_FILTER)
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.commands = [{"speed": float(c.get("speed", 0)), "duration": float(c.get("duration", 0))}
                            for c in data if isinstance(c, dict)]
            self._refresh_table()
            self.status_var.set(f"Загружено: {os.path.basename(path)}")
        except Exception as e:
            messagebox.showerror("Ошибка", str(e))

    def _send_speed(self, speed, turn=0.0):
        """Отправить V,speed,turn на робота."""
        if not self.app.serial_port or not self.app.serial_port.is_open:
            return False
        try:
            speed = max(-MAX_TARGET_SPEED_MPS, min(MAX_TARGET_SPEED_MPS, speed))
            turn = max(-1.0, min(1.0, turn))
            cmd = f"V,{speed},{turn}\n"
            self.app.serial_port.write(cmd.encode("utf-8"))
            self.app.serial_port.flush()
            self.app._log(f">>> {cmd.strip()} (демо)\n", "sent")
            return True
        except Exception:
            return False

    def _run_trajectory(self):
        if not self.app.serial_port or not self.app.serial_port.is_open:
            messagebox.showwarning("Ошибка", "Сначала подключитесь к Arduino")
            return
        if not self.commands:
            messagebox.showwarning("Ошибка", "Добавьте хотя бы одну команду")
            return
        self._run_stop = False
        self._run_index = 0
        self.run_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)
        self._run_next()

    def _run_next(self):
        if self._run_stop:
            self._run_after_id = None
            return
        if self._run_index >= len(self.commands):
            self._send_speed(0)
            if hasattr(self.app, "speed_target_var"):
                self.app.speed_target_var.set("0")
            self.status_var.set("Готов (завершено)")
            self.run_btn.config(state=tk.NORMAL)
            self.stop_btn.config(state=tk.DISABLED)
            return
        cmd = self.commands[self._run_index]
        speed, dur = cmd["speed"], cmd["duration"]
        self._run_remaining = dur
        self.status_var.set(f"Выполняется: {self._run_index + 1}/{len(self.commands)} — {speed:.2f} м/с, осталось {dur:.1f} сек")
        self._send_speed(speed)
        if hasattr(self.app, "speed_target_var"):
            self.app.speed_target_var.set(f"{speed:.2f}")
        if dur <= 0:
            self._run_index += 1
            self.parent.after(10, self._run_next)
            return
        self._run_after_id = self.parent.after(int(dur * 1000), self._run_step_done)
        self._schedule_timer_update()

    def _schedule_timer_update(self):
        """Обновлять оставшееся время каждые 500 мс."""
        if self._run_timer_id:
            self.parent.after_cancel(self._run_timer_id)
        if self._run_stop or self._run_index >= len(self.commands):
            return

        def _tick():
            self._run_timer_id = None
            if self._run_stop:
                return
            self._run_remaining = max(0, self._run_remaining - 0.5)
            cmd = self.commands[self._run_index]
            self.status_var.set(
                f"Выполняется: {self._run_index + 1}/{len(self.commands)} — {cmd['speed']:.2f} м/с, осталось {self._run_remaining:.1f} сек"
            )
            if self._run_remaining > 0:
                self._run_timer_id = self.parent.after(500, _tick)

        self._run_timer_id = self.parent.after(500, _tick)

    def _run_step_done(self):
        self._run_after_id = None
        if self._run_timer_id:
            self.parent.after_cancel(self._run_timer_id)
            self._run_timer_id = None
        if self._run_stop:
            self._send_speed(0)
            self.status_var.set("Остановлено")
            self.run_btn.config(state=tk.NORMAL)
            self.stop_btn.config(state=tk.DISABLED)
            return
        self._run_index += 1
        self._run_next()

    def _stop_trajectory(self):
        self._run_stop = True
        if self._run_timer_id:
            self.parent.after_cancel(self._run_timer_id)
            self._run_timer_id = None
        if self._run_after_id:
            self.parent.after_cancel(self._run_after_id)
            self._run_after_id = None
        self._send_speed(0)
        self.status_var.set("Остановлено")
        self.run_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        if hasattr(self.app, "speed_target_var"):
            self.app.speed_target_var.set("0")

    def _on_close(self):
        if self._run_timer_id:
            self.parent.after_cancel(self._run_timer_id)
        if self._run_after_id:
            self.parent.after_cancel(self._run_after_id)
        self._run_stop = True
        self.win.destroy()


# Стиль как в React: серый фон, белые карточки
BG_MAIN = "#e5e7eb"      # gray-200
BG_CARD = "#ffffff"      # white
BORDER_CARD = "#d1d5db"  # gray-300


class PidTunerApp:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Giro-Robot PID Tuner")
        self.root.geometry("900x820")
        self.root.resizable(True, True)
        self.root.configure(bg=BG_MAIN)

        self.serial_port = None
        self._updating = False  # Блокировка рекурсии при синхронизации
        self._send_after_id = None  # Для debounce авто-отправки
        self.read_queue = queue.Queue()
        self.read_thread = None
        self.read_running = False
        self.graph_enabled = False
        self.graph_window = None
        self.graph_data = deque(maxlen=400)  # Буфер данных
        self.graph_lines = None  # Линии для set_data (без перерисовки)
        self.speed_window = None
        self.speed_enabled = False
        self.speed_mps = 0.0
        self.speed_steps = 0.0
        self.trajectory_window = None

        self._build_ui()
        self._start_read_thread()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _card_frame(self, parent, **kw):
        """Белая карточка как в React (rounded-lg shadow-sm)."""
        f = tk.Frame(parent, bg=BG_CARD, highlightbackground=BORDER_CARD, highlightthickness=1, **kw)
        return f

    def _build_ui(self):
        main = tk.Frame(self.root, bg=BG_MAIN, padx=16, pady=24)
        main.pack(fill=tk.BOTH, expand=True)

        # === Header ===
        header = self._card_frame(main)
        header.pack(fill=tk.X, pady=(0, 24))
        header_inner = tk.Frame(header, bg=BG_CARD, padx=16, pady=12)
        header_inner.pack(fill=tk.X)
        tk.Label(header_inner, text="Giro-Robot PID Tuner", font=("", 18, "bold"),
                 bg=BG_CARD, fg="#111827").pack(anchor=tk.W)

        # === 2-колоночный layout ===
        columns = tk.Frame(main, bg=BG_MAIN)
        columns.pack(fill=tk.BOTH, expand=True)

        # --- Левая колонка: Подключение + ANGLE PID ---
        left_col = tk.Frame(columns, bg=BG_MAIN)
        left_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 12))

        # ConnectionPanel
        conn_card = self._card_frame(left_col)
        conn_card.pack(fill=tk.X, pady=(0, 12))
        conn_inner = tk.Frame(conn_card, bg=BG_CARD, padx=16, pady=12)
        conn_inner.pack(fill=tk.X)
        ttk.Label(conn_inner, text="Подключение").pack(anchor=tk.W)
        row1 = ttk.Frame(conn_inner)
        row1.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(row1, text="Порт:").pack(side=tk.LEFT, padx=(0, 8))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(row1, textvariable=self.port_var, width=14, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=(0, 8))
        self.refresh_btn = ttk.Button(row1, text="Обновить", command=self._refresh_ports)
        self.refresh_btn.pack(side=tk.LEFT, padx=(0, 8))
        self.connect_btn = ttk.Button(row1, text="Подключить", command=self._toggle_connect)
        self.connect_btn.pack(side=tk.LEFT)
        self.status_label = ttk.Label(row1, text="  Отключено", foreground="gray")
        self.status_label.pack(side=tk.LEFT, padx=(12, 0))

        # Строка: активный ползунок
        active_row = ttk.Frame(left_col)
        active_row.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(active_row, text="Активный ползунок:", font=("", 10, "bold")).pack(side=tk.LEFT, padx=(0, 6))
        self._active_slider_label_var = tk.StringVar(value="— (кликните на ползунок)")
        ttk.Label(active_row, textvariable=self._active_slider_label_var, foreground="blue").pack(side=tk.LEFT)
        ttk.Label(active_row, text="  ↑↓ ←→ для изменения", font=("", 9), foreground="gray").pack(side=tk.LEFT, padx=(8, 0))

        # PidCard: ANGLE PID (угол → моторы)
        pid_card = self._card_frame(left_col)
        pid_card.pack(fill=tk.BOTH, expand=True, pady=(0, 12))
        pid_frame = tk.Frame(pid_card, bg=BG_CARD, padx=16, pady=12)
        pid_frame.pack(fill=tk.BOTH, expand=True)
        ttk.Label(pid_frame, text="ANGLE PID", font=("", 12, "bold")).pack(anchor=tk.W)
        ttk.Label(pid_frame, text="Angle → Motors", font=("", 9), foreground="gray").pack(anchor=tk.W)

        self.kp_var = tk.StringVar(value=str(DEFAULT_KP))
        self.ki_var = tk.StringVar(value=str(DEFAULT_KI))
        self.kd_var = tk.StringVar(value=str(DEFAULT_KD))
        self.limit_var = tk.StringVar(value=str(DEFAULT_LIMIT))
        self.angle_step_var = tk.StringVar(value="0.01")

        step_row = ttk.Frame(pid_frame)
        step_row.pack(fill=tk.X, pady=(12, 6))
        ttk.Label(step_row, text="Шаг:").pack(side=tk.LEFT, padx=(0, 6))
        angle_step_combo = ttk.Combobox(step_row, textvariable=self.angle_step_var, values=PID_STEP_OPTIONS,
                                        width=6, state="readonly")
        angle_step_combo.pack(side=tk.LEFT)

        def add_param_row(parent, label, str_var, slider_range, resolution=0.1):
            row = tk.Frame(parent, bg="#f0f0f0", padx=4, pady=2)
            row.pack(fill=tk.X, pady=2)
            lbl = ttk.Label(row, text=label, width=6)
            lbl.pack(side=tk.LEFT, padx=(0, 5))
            slider = tk.Scale(row, from_=slider_range[0], to=slider_range[1], resolution=resolution,
                             orient=tk.HORIZONTAL, length=220, showvalue=1, takefocus=1,
                             command=lambda v, sv=str_var: self._slider_changed(sv, v),
                             highlightthickness=0)
            slider.pack(side=tk.LEFT, padx=(0, 8))
            entry = ttk.Entry(row, textvariable=str_var, width=10)
            entry.pack(side=tk.LEFT)
            self._bind_slider_row(row, slider, str_var, slider_range, self.angle_step_var, lbl, "Angle")
            return slider

        self.kp_slider = add_param_row(pid_frame, "Kp:", self.kp_var, KP_RANGE, 1.0)
        self.ki_slider = add_param_row(pid_frame, "Ki:", self.ki_var, KI_RANGE, 0.01)
        self.kd_slider = add_param_row(pid_frame, "Kd:", self.kd_var, KD_RANGE, 0.01)
        self.limit_slider = add_param_row(pid_frame, "Limit:", self.limit_var, LIMIT_RANGE, 100.0)

        # Z-коррекция в ANGLE PID
        z_row = ttk.Frame(pid_frame)
        z_row.pack(fill=tk.X, pady=(12, 0))
        ttk.Label(z_row, text="Z:", width=6).pack(side=tk.LEFT, padx=(0, 6))
        self.z_var = tk.StringVar(value="0")
        ttk.Entry(z_row, textvariable=self.z_var, width=10).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(z_row, text="Сохранить Z в EEPROM", command=self._save_z_to_eeprom).pack(side=tk.LEFT)
        ttk.Label(z_row, text="(коррекция нуля, градусы)", font=("", 9), foreground="gray").pack(side=tk.LEFT, padx=(12, 0))

        # --- Правая колонка: SPEED PID + SpeedControls ---
        right_col = tk.Frame(columns, bg=BG_MAIN)
        right_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(12, 0))

        # PidCard: SPEED PID (скорость → угол наклона)
        spd_card = self._card_frame(right_col)
        spd_card.pack(fill=tk.X, pady=(0, 12))
        spd_frame = tk.Frame(spd_card, bg=BG_CARD, padx=16, pady=12)
        spd_frame.pack(fill=tk.X)
        ttk.Label(spd_frame, text="SPEED PID", font=("", 12, "bold")).pack(anchor=tk.W)
        ttk.Label(spd_frame, text="Speed → Tilt Angle", font=("", 9), foreground="gray").pack(anchor=tk.W)

        self.spd_kp_var = tk.StringVar(value=str(DEFAULT_SPD_KP))
        self.spd_ki_var = tk.StringVar(value=str(DEFAULT_SPD_KI))
        self.spd_kd_var = tk.StringVar(value=str(DEFAULT_SPD_KD))
        self.spd_limit_var = tk.StringVar(value=str(DEFAULT_SPD_LIMIT))
        self.spd_step_var = tk.StringVar(value="0.01")

        spd_step_row = ttk.Frame(spd_frame)
        spd_step_row.pack(fill=tk.X, pady=(12, 6))
        ttk.Label(spd_step_row, text="Шаг:").pack(side=tk.LEFT, padx=(0, 6))
        spd_step_combo = ttk.Combobox(spd_step_row, textvariable=self.spd_step_var, values=PID_STEP_OPTIONS,
                                       width=6, state="readonly")
        spd_step_combo.pack(side=tk.LEFT)

        def add_spd_param_row(parent, label, str_var, slider_range, resolution=0.01):
            row = tk.Frame(parent, bg="#f0f0f0", padx=4, pady=2)
            row.pack(fill=tk.X, pady=2)
            lbl = ttk.Label(row, text=label, width=6)
            lbl.pack(side=tk.LEFT, padx=(0, 5))
            slider = tk.Scale(row, from_=slider_range[0], to=slider_range[1], resolution=resolution,
                             orient=tk.HORIZONTAL, length=220, showvalue=1, takefocus=1,
                             command=lambda v, sv=str_var: self._slider_changed_spd(sv, v),
                             highlightthickness=0)
            slider.pack(side=tk.LEFT, padx=(0, 8))
            entry = ttk.Entry(row, textvariable=str_var, width=10)
            entry.pack(side=tk.LEFT)
            self._bind_slider_row(row, slider, str_var, slider_range, self.spd_step_var, lbl, "Speed")
            return slider

        self.spd_kp_slider = add_spd_param_row(spd_frame, "Kp:", self.spd_kp_var, SPD_KP_RANGE, 1.0)
        self.spd_ki_slider = add_spd_param_row(spd_frame, "Ki:", self.spd_ki_var, SPD_KI_RANGE, 0.1)
        self.spd_kd_slider = add_spd_param_row(spd_frame, "Kd:", self.spd_kd_var, SPD_KD_RANGE, 0.01)
        self.spd_limit_slider = add_spd_param_row(spd_frame, "Limit:", self.spd_limit_var, SPD_LIMIT_RANGE, 1.0)
        # Вывод значений Speed PID с Arduino
        spd_status_row = ttk.Frame(spd_frame)
        spd_status_row.pack(fill=tk.X, pady=(12, 0))
        self.spd_status_var = tk.StringVar(value="Speed PID на Arduino: —")
        ttk.Label(spd_status_row, textvariable=self.spd_status_var, font=("", 9), foreground="blue").pack(side=tk.LEFT)
        ttk.Button(spd_status_row, text="Прочитать Speed PID", command=self._read_speed_pid_only).pack(side=tk.LEFT, padx=(12, 0))
        ttk.Label(spd_frame, text="Каскад: Speed → угол, Angle → моторы. При 0 м/с только Angle.", font=("", 9), foreground="gray").pack(anchor=tk.W, pady=(4, 0))

        # SpeedControls: целевая скорость
        speed_card = self._card_frame(right_col)
        speed_card.pack(fill=tk.X, pady=(0, 12))
        speed_frame = tk.Frame(speed_card, bg=BG_CARD, padx=16, pady=12)
        speed_frame.pack(fill=tk.X)
        ttk.Label(speed_frame, text="Целевая скорость (м/с)", font=("", 12, "bold")).pack(anchor=tk.W)
        speed_row = ttk.Frame(speed_frame)
        speed_row.pack(fill=tk.X, pady=4)
        ttk.Label(speed_row, text="Скорость:").pack(side=tk.LEFT, padx=(0, 5))
        self.speed_target_var = tk.StringVar(value="0")
        self.speed_target_entry = ttk.Entry(speed_row, textvariable=self.speed_target_var, width=8)
        self.speed_target_entry.pack(side=tk.LEFT, padx=(0, 5))
        ttk.Label(speed_row, text="м/с").pack(side=tk.LEFT, padx=(0, 10))
        ttk.Label(speed_row, text="Поворот:").pack(side=tk.LEFT, padx=(0, 5))
        self.turn_var = tk.StringVar(value="0")
        ttk.Entry(speed_row, textvariable=self.turn_var, width=6).pack(side=tk.LEFT, padx=(0, 5))
        ttk.Button(speed_row, text="Установить", command=self._send_target_speed).pack(side=tk.LEFT, padx=(0, 10))
        # Ползунок соотношения шаговиков: -1 влево, 0 прямо, 1 вправо
        turn_row = ttk.Frame(speed_frame)
        turn_row.pack(fill=tk.X, pady=4)
        ttk.Label(turn_row, text="Соотношение колёс:").pack(side=tk.LEFT, padx=(0, 5))
        self.turn_slider = tk.Scale(turn_row, from_=-1.0, to=1.0, resolution=0.05, orient=tk.HORIZONTAL,
                                    length=280, showvalue=1, command=self._turn_slider_changed)
        self.turn_slider.pack(side=tk.LEFT, padx=(0, 8))
        ttk.Label(turn_row, text="← влево | 0 прямо | вправо →", font=("", 9), foreground="gray").pack(side=tk.LEFT, padx=(5, 0))
        quick_row = ttk.Frame(speed_frame)
        quick_row.pack(fill=tk.X, pady=2)
        ttk.Label(quick_row, text="Быстро:", foreground="gray").pack(side=tk.LEFT, padx=(0, 5))
        for label, val in [("Стоп (0)", "0"), ("0.3", "0.3"), ("0.5", "0.5"), ("1.0", "1.0")]:
            btn = ttk.Button(quick_row, text=label, width=6, command=lambda v=val: self._set_and_send_speed(v))
            btn.pack(side=tk.LEFT, padx=2)
        ttk.Label(speed_frame, text="0 м/с = остановка и баланс на месте", font=("", 9), foreground="gray").pack(anchor=tk.W)
        ttk.Label(speed_frame, text="← → меняют скорость (когда фокус не на ползунке)", font=("", 9), foreground="gray").pack(anchor=tk.W)

        self._bind_speed_arrows()

        self._sync_sliders_from_vars()
        self._sync_sliders_spd_from_vars()
        self._sync_turn_slider_from_var()

        # Привязка переменных
        for var in (self.kp_var, self.ki_var, self.kd_var, self.limit_var):
            var.trace_add("write", lambda *a: self._on_param_changed())
        for var in (self.spd_kp_var, self.spd_ki_var, self.spd_kd_var, self.spd_limit_var):
            var.trace_add("write", lambda *a: self._on_param_changed_spd())
        for var in (self.kp_var, self.ki_var, self.kd_var, self.limit_var,
                    self.spd_kp_var, self.spd_ki_var, self.spd_kd_var, self.spd_limit_var,
                    self.speed_target_var, self.turn_var):
            var.trace_add("write", lambda *a: self._update_params_display())
        self.turn_var.trace_add("write", lambda *a: self._sync_turn_slider_from_var())

        # === ActionPanel (кнопки) ===
        action_card = self._card_frame(main)
        action_card.pack(fill=tk.X, pady=(0, 12))
        btn_frame = tk.Frame(action_card, bg=BG_CARD, padx=16, pady=12)
        btn_frame.pack(fill=tk.X)

        self.read_btn = ttk.Button(btn_frame, text="Прочитать оба PID", command=self._read_from_arduino)
        self.read_btn.pack(side=tk.LEFT, padx=(0, 8))
        self.save_btn = ttk.Button(btn_frame, text="Сохранить Angle PID", command=self._save_to_eeprom)
        self.save_btn.pack(side=tk.LEFT, padx=(0, 8))
        self.save_spd_btn = ttk.Button(btn_frame, text="Сохранить Speed PID", command=self._save_speed_pid_to_eeprom)
        self.save_spd_btn.pack(side=tk.LEFT, padx=(0, 8))
        self.graph_btn = ttk.Button(btn_frame, text="График PID", command=self._toggle_graph)
        self.graph_btn.pack(side=tk.LEFT, padx=(0, 8))
        self.speed_btn = ttk.Button(btn_frame, text="Скорость моторов", command=self._toggle_speed_window)
        self.speed_btn.pack(side=tk.LEFT, padx=(0, 8))
        self.traj_btn = ttk.Button(btn_frame, text="Траектории", command=self._open_trajectory_window)
        self.traj_btn.pack(side=tk.LEFT)
        ttk.Label(btn_frame, text="  (изменения отправляются автоматически)", font=("", 9), foreground="gray").pack(side=tk.LEFT, padx=(16, 0))

        # === Текущие параметры ===
        params_card = self._card_frame(main)
        params_card.pack(fill=tk.X, pady=(0, 12))
        params_inner = tk.Frame(params_card, bg=BG_CARD, padx=16, pady=10)
        params_inner.pack(fill=tk.X)
        self._params_display_var = tk.StringVar()
        ttk.Label(params_inner, textvariable=self._params_display_var, font=("", 9)).pack(anchor=tk.W)
        self._update_params_display()

        # === LogPanel ===
        log_card = self._card_frame(main)
        log_card.pack(fill=tk.BOTH, expand=True)
        log_inner = tk.Frame(log_card, bg=BG_CARD, padx=16, pady=12)
        log_inner.pack(fill=tk.BOTH, expand=True)
        ttk.Label(log_inner, text="Лог Serial (команды и ответы)").pack(anchor=tk.W)
        self.log_text = scrolledtext.ScrolledText(log_inner, height=6, state=tk.DISABLED, wrap=tk.WORD,
                                                  font=("Courier", 9))
        self.log_text.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        self.log_text.tag_configure("sent", foreground="#006600")
        self.log_text.tag_configure("recv", foreground="#0000aa")

        self._refresh_ports()

    def _slider_changed(self, var, value):
        """Ползунок изменён — обновить поле и отправить (с debounce)."""
        if self._updating:
            return
        self._updating = True
        var.set(str(float(value)))
        self._updating = False
        if self._send_after_id:
            self.root.after_cancel(self._send_after_id)
        self._send_after_id = self.root.after(150, self._send_pid_auto)

    def _slider_changed_spd(self, var, value):
        """Speed PID: ползунок изменён."""
        if self._updating:
            return
        self._updating = True
        var.set(str(float(value)))
        self._updating = False
        if self._send_after_id:
            self.root.after_cancel(self._send_after_id)
        self._send_after_id = self.root.after(150, self._send_speed_pid_auto)

    def _bind_slider_row(self, row, slider, str_var, slider_range, step_var, label_widget, block_name):
        """Фокус по клику, подсветка активной строки, стрелки ↑↓←→ для изменения."""
        ROW_BG_NORMAL = "#f0f0f0"
        ROW_BG_ACTIVE = "#c8e6ff"  # голубой — активный ползунок

        def _focus_slider(event=None):
            slider.focus_set()

        def _on_focus_in(e):
            row.config(bg=ROW_BG_ACTIVE)
            lbl_text = label_widget.cget("text").strip()
            self._active_slider_label_var.set(f"{block_name} {lbl_text}")

        def _on_focus_out(e):
            row.config(bg=ROW_BG_NORMAL)

        def _step(delta):
            try:
                step = float(step_var.get())
            except ValueError:
                step = 0.01
            try:
                val = float(str_var.get())
            except ValueError:
                val = slider_range[0]
            val = val + delta * step
            val = max(slider_range[0], min(slider_range[1], val))
            str_var.set(f"{val:.6g}")

        def on_up(event):
            _step(1)
            return "break"

        def on_down(event):
            _step(-1)
            return "break"

        # Клик по строке/ползунку/подписи — фокус на ползунок
        for w in (row, slider, label_widget):
            w.bind("<Button-1>", _focus_slider)
        slider.bind("<FocusIn>", _on_focus_in)
        slider.bind("<FocusOut>", _on_focus_out)
        for key in ("<Up>", "<Right>"):
            slider.bind(key, on_up)
        for key in ("<Down>", "<Left>"):
            slider.bind(key, on_down)

    def _update_params_display(self):
        """Обновить панель текущих параметров."""
        try:
            a = (self.kp_var.get(), self.ki_var.get(), self.kd_var.get(), self.limit_var.get())
            s = (self.spd_kp_var.get(), self.spd_ki_var.get(), self.spd_kd_var.get(), self.spd_limit_var.get())
            text = (
                f"Angle: Kp={a[0]}  Ki={a[1]}  Kd={a[2]}  Limit={a[3]}  |  "
                f"Speed: Kp={s[0]}  Ki={s[1]}  Kd={s[2]}  L={s[3]}  |  "
                f"Скорость={self.speed_target_var.get()} м/с  Поворот={self.turn_var.get()}"
            )
            self._params_display_var.set(text)
        except Exception:
            self._params_display_var.set("—")

    def _sync_sliders_from_vars(self):
        """Синхронизировать ползунки Angle PID с полями."""
        if self._updating:
            return
        self._updating = True
        try:
            for slider, var, rng in [
                (self.kp_slider, self.kp_var, KP_RANGE),
                (self.ki_slider, self.ki_var, KI_RANGE),
                (self.kd_slider, self.kd_var, KD_RANGE),
                (self.limit_slider, self.limit_var, LIMIT_RANGE),
            ]:
                try:
                    v = float(var.get())
                    v = max(rng[0], min(rng[1], v))
                    slider.set(v)
                except ValueError:
                    pass
        finally:
            self._updating = False

    def _sync_sliders_spd_from_vars(self):
        """Синхронизировать ползунки Speed PID с полями."""
        if self._updating:
            return
        self._updating = True
        try:
            for slider, var, rng in [
                (self.spd_kp_slider, self.spd_kp_var, SPD_KP_RANGE),
                (self.spd_ki_slider, self.spd_ki_var, SPD_KI_RANGE),
                (self.spd_kd_slider, self.spd_kd_var, SPD_KD_RANGE),
                (self.spd_limit_slider, self.spd_limit_var, SPD_LIMIT_RANGE),
            ]:
                try:
                    v = float(var.get())
                    v = max(rng[0], min(rng[1], v))
                    slider.set(v)
                except ValueError:
                    pass
        finally:
            self._updating = False

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _toggle_connect(self):
        if self.serial_port and self.serial_port.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("Ошибка", "Выберите COM-порт")
            return
        try:
            self.serial_port = serial.Serial(port, BAUD_RATE, timeout=0.1, write_timeout=1.0)
            self.status_label.config(text="Подключено", foreground="green")
            self.connect_btn.config(text="Отключить")
            self.port_combo.config(state="disabled")
            self._log(f"Подключено к {port}\n")
        except Exception as e:
            messagebox.showerror("Ошибка", f"Не удалось подключиться:\n{e}")

    def _disconnect(self):
        self.read_running = False
        self.graph_enabled = False
        self.speed_enabled = False
        self._close_graph_window()
        self._close_speed_window()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.serial_port = None
        self.status_label.config(text="Отключено", foreground="gray")
        self.connect_btn.config(text="Подключить")
        self.port_combo.config(state="readonly")
        self._log("Отключено\n")

    def _on_param_changed(self):
        if self._updating:
            return
        self._sync_sliders_from_vars()
        if self._send_after_id:
            self.root.after_cancel(self._send_after_id)
        self._send_after_id = self.root.after(200, self._send_pid_auto)

    def _on_param_changed_spd(self):
        if self._updating:
            return
        self._sync_sliders_spd_from_vars()
        if self._send_after_id:
            self.root.after_cancel(self._send_after_id)
        self._send_after_id = self.root.after(200, self._send_speed_pid_auto)

    def _get_pid_values(self):
        try:
            kp = float(self.kp_var.get())
            ki = float(self.ki_var.get())
            kd = float(self.kd_var.get())
            limit = float(self.limit_var.get())
            if kp >= 0 and ki >= 0 and kd >= 0 and limit >= 100:
                return kp, ki, kd, limit
        except ValueError:
            pass
        return None

    def _send_pid_auto(self):
        """Автоматическая отправка Angle PID при изменении."""
        self._send_after_id = None
        vals = self._get_pid_values()
        if vals is None:
            return
        if not self.serial_port or not self.serial_port.is_open:
            return
        kp, ki, kd, limit = vals
        cmd = f"f,{kp},{ki},{kd},{limit}\n"
        try:
            self.serial_port.write(cmd.encode("utf-8"))
            self.serial_port.flush()
            self._log(f">>> {cmd.strip()}\n", "sent")
        except Exception as e:
            self._log(f"Ошибка отправки: {e}\n")

    def _get_speed_pid_values(self):
        try:
            kp = float(self.spd_kp_var.get())
            ki = float(self.spd_ki_var.get())
            kd = float(self.spd_kd_var.get())
            limit = float(self.spd_limit_var.get())
            if kp >= 0 and ki >= 0 and kd >= 0 and limit >= 1:
                return kp, ki, kd, limit
        except ValueError:
            pass
        return None

    def _send_speed_pid_auto(self):
        """Автоматическая отправка Speed PID при изменении."""
        self._send_after_id = None
        vals = self._get_speed_pid_values()
        if vals is None:
            return
        if not self.serial_port or not self.serial_port.is_open:
            return
        kp, ki, kd, limit = vals
        cmd = f"f2,{kp},{ki},{kd},{limit}\n"
        try:
            self.serial_port.write(cmd.encode("utf-8"))
            self.serial_port.flush()
            self.spd_status_var.set(f"Speed PID: Kp={kp} Ki={ki} Kd={kd} L={limit}")
            self._log(f">>> {cmd.strip()}\n", "sent")
        except Exception as e:
            self._log(f"Ошибка отправки Speed PID: {e}\n")

    def _send_pid(self):
        """Явная отправка (с проверками и диалогами)."""
        vals = self._get_pid_values()
        if vals is None:
            messagebox.showwarning("Ошибка", "Проверьте значения: Kp,Ki,Kd ≥ 0, Limit ≥ 100")
            return
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Ошибка", "Сначала подключитесь к Arduino")
            return
        self._send_pid_auto()

    def _read_from_arduino(self):
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Ошибка", "Сначала подключитесь к Arduino")
            return
        try:
            self.serial_port.write(b"P\n")
            self.serial_port.flush()
            self._log(">>> P (запрос Angle PID)\n")
            self.root.after(150, self._read_speed_pid_only)  # P2 с задержкой для раздельных ответов
        except Exception as e:
            self._log(f"Ошибка: {e}\n")

    def _read_speed_pid_only(self):
        """Запросить только Speed PID (для отображения значений)."""
        if not self.serial_port or not self.serial_port.is_open:
            return
        try:
            self.serial_port.write(b"P2\n")
            self.serial_port.flush()
            self._log(">>> P2 (запрос Speed PID)\n")
        except Exception as e:
            self._log(f"Ошибка: {e}\n")

    def _save_to_eeprom(self):
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Ошибка", "Сначала подключитесь к Arduino")
            return
        try:
            self.serial_port.write(b"w\n")
            self.serial_port.flush()
            self._log(">>> w (сохранить Angle PID в EEPROM)\n")
        except Exception as e:
            self._log(f"Ошибка: {e}\n")

    def _save_speed_pid_to_eeprom(self):
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Ошибка", "Сначала подключитесь к Arduino")
            return
        try:
            self.serial_port.write(b"w2\n")
            self.serial_port.flush()
            self._log(">>> w2 (сохранить Speed PID в EEPROM)\n")
        except Exception as e:
            self._log(f"Ошибка: {e}\n")

    def _toggle_graph(self):
        """Включить/выключить график PID."""
        if not HAS_MATPLOTLIB:
            messagebox.showerror("Ошибка", "Установите matplotlib: pip install matplotlib")
            return
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Ошибка", "Сначала подключитесь к Arduino")
            return
        self.graph_enabled = not self.graph_enabled
        try:
            self.serial_port.write(b"G\n")
            self.serial_port.flush()
            self._log(f">>> G (график {'вкл' if self.graph_enabled else 'выкл'})\n")
        except Exception as e:
            self._log(f"Ошибка: {e}\n")
            self.graph_enabled = False
            return
        if self.graph_enabled:
            self._open_graph_window()
        else:
            self._close_graph_window()

    def _open_graph_window(self):
        """Открыть окно графика."""
        if self.graph_window and self.graph_window.winfo_exists():
            self.graph_window.lift()
            return
        self.graph_data.clear()
        self.graph_lines = None
        self.graph_window = tk.Toplevel(self.root)
        self.graph_window.title("График PID")
        self.graph_window.geometry("950x700")
        self.graph_window.protocol("WM_DELETE_WINDOW", self._close_graph_window)
        if HAS_MATPLOTLIB:
            fig = Figure(figsize=(9.5, 6.5), dpi=100)
            # 1) Углы: target, angle, error
            self.ax1 = fig.add_subplot(311)
            # 2) Компоненты PID: P, I, D
            self.ax2 = fig.add_subplot(312)
            # 3) Выход и лимит
            self.ax3 = fig.add_subplot(313)
            fig.tight_layout(pad=2.0)
            self.canvas = FigureCanvasTkAgg(fig, master=self.graph_window)
            self.canvas.draw()
            self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
            # Подсказка
            hint = (
                "P — пропорциональная: Kp×ошибка\n"
                "I — интегральная: Ki×∫ошибки\n"
                "D — дифференциальная: Kd×d(ошибка)/dt\n"
                "L — лимит: макс. выход на моторы"
            )
            ttk.Label(self.graph_window, text=hint, font=("", 9), foreground="gray").pack(anchor=tk.W, padx=8, pady=2)
        self._schedule_graph_update()

    def _close_graph_window(self):
        """Закрыть окно графика."""
        was_enabled = self.graph_enabled
        self.graph_enabled = False
        # Отправить G только при закрытии окна (X) — по кнопке уже отправили
        if was_enabled and self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(b"G\n")
                self.serial_port.flush()
            except Exception:
                pass
        if self.graph_window and self.graph_window.winfo_exists():
            self.graph_window.destroy()
        self.graph_window = None
        self.graph_lines = None

    def _schedule_graph_update(self):
        """Запланировать обновление графика (реже = плавнее)."""
        try:
            if self.graph_enabled and self.graph_window and self.graph_window.winfo_exists() and HAS_MATPLOTLIB:
                self._update_graph()
                self.root.after(120, self._schedule_graph_update)  # 120 мс — меньше нагрузка
        except tk.TclError:
            pass

    def _update_graph(self):
        """Обновить график из накопленных данных."""
        if not self.graph_data:
            return
        try:
            limit_val = float(self.limit_var.get()) if self.limit_var.get() else 7500
            data = list(self.graph_data)
            n = len(data)
            # Ограничиваем точки для скорости (каждая 2-я при >300)
            step = max(1, n // 250)
            data = data[::step]
            t = list(range(0, n, step))
            target = [d[0] for d in data]
            angle = [d[1] for d in data]
            err = [d[2] for d in data]
            p_vals = [d[3] for d in data]
            i_vals = [d[4] for d in data]
            d_vals = [d[5] for d in data]
            out = [d[6] for d in data]

            if self.graph_lines is None:
                # Первый раз — создаём линии
                self.ax1.set_ylabel("град")
                self.ax1.set_title("Углы: target (целевой), angle (текущий), error (ошибка)")
                self.ax1.grid(True, alpha=0.3)
                self.ax2.set_ylabel("P (левая ось)")
                self.ax2_twin = self.ax2.twinx()
                self.ax2_twin.set_ylabel("I, D (правая ось)")
                self.ax2.set_title("P — пропорц. (Kp×ошибка) | I — интеграл | D — дифференциал")
                self.ax2.grid(True, alpha=0.3)
                self.ax3.set_ylabel("выход")
                self.ax3.set_xlabel("время (отсчёты)")
                self.ax3.set_title("output (сумма P+I+D), L — лимит ±limit")
                self.ax3.grid(True, alpha=0.3)
                l1a, = self.ax1.plot(t, target, "b-", label="target", alpha=0.9)
                l1b, = self.ax1.plot(t, angle, "g-", label="angle", alpha=0.9)
                l1c, = self.ax1.plot(t, err, "r-", label="error", alpha=0.7)
                l2a, = self.ax2.plot(t, p_vals, "C0-", label="P", alpha=0.9)
                l2b, = self.ax2_twin.plot(t, i_vals, "C1-", label="I", alpha=0.9)
                l2c, = self.ax2_twin.plot(t, d_vals, "C2-", label="D", alpha=0.9)
                l3a, = self.ax3.plot(t, out, "k-", label="output", linewidth=1.5)
                self.ax3.axhline(limit_val, color="red", linestyle="--", alpha=0.7, label="L +limit")
                self.ax3.axhline(-limit_val, color="red", linestyle="--", alpha=0.7, label="L -limit")
                self.ax1.legend(loc="upper right", fontsize=8)
                self.ax2.legend(loc="upper left", fontsize=8)
                self.ax2_twin.legend(loc="upper right", fontsize=8)
                self.ax3.legend(loc="upper right", fontsize=8)
                self.graph_lines = (l1a, l1b, l1c, l2a, l2b, l2c, l3a)
            else:
                # Обновляем данные линий (быстрее чем clear+plot)
                l1a, l1b, l1c, l2a, l2b, l2c, l3a = self.graph_lines
                l1a.set_data(t, target)
                l1b.set_data(t, angle)
                l1c.set_data(t, err)
                l2a.set_data(t, p_vals)
                l2b.set_data(t, i_vals)
                l2c.set_data(t, d_vals)
                l3a.set_data(t, out)
                # Обновить лимиты
                to_remove = [ln for ln in self.ax3.get_lines() if ln.get_label().startswith("L ")]
                for ln in to_remove:
                    ln.remove()
                self.ax3.axhline(limit_val, color="red", linestyle="--", alpha=0.7, label="L +limit")
                self.ax3.axhline(-limit_val, color="red", linestyle="--", alpha=0.7, label="L -limit")

            for ax in (self.ax1, self.ax2, self.ax2_twin, self.ax3):
                ax.relim()
                ax.autoscale_view()
            self.canvas.draw_idle()
        except Exception:
            pass

    def _open_trajectory_window(self):
        """Открыть окно управления демо-траекториями."""
        if self.trajectory_window and self.trajectory_window.win.winfo_exists():
            self.trajectory_window.win.lift()
            return
        self.trajectory_window = TrajectoryWindow(self.root, self)

    def _toggle_speed_window(self):
        """Открыть/закрыть окно скорости моторов."""
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Ошибка", "Сначала подключитесь к Arduino")
            return
        self.speed_enabled = not self.speed_enabled
        if self.speed_enabled:
            if not self.graph_enabled:
                self.graph_enabled = True
                try:
                    self.serial_port.write(b"G\n")
                    self.serial_port.flush()
                    self._log(">>> G (график вкл для скорости)\n")
                except Exception:
                    pass
            self._open_speed_window()
        else:
            self._close_speed_window()

    def _open_speed_window(self):
        """Открыть окно скорости моторов."""
        if self.speed_window and self.speed_window.winfo_exists():
            self.speed_window.lift()
            return
        self.speed_window = tk.Toplevel(self.root)
        self.speed_window.title("Скорость моторов")
        self.speed_window.geometry("380x200")
        self.speed_window.protocol("WM_DELETE_WINDOW", self._close_speed_window)
        main = ttk.Frame(self.speed_window, padding=15)
        main.pack(fill=tk.BOTH, expand=True)
        ttk.Label(main, text="Скорость (оценка)", font=("", 14, "bold")).pack(pady=(0, 15))
        self.speed_mps_label = ttk.Label(main, text="0.00 м/с", font=("", 24))
        self.speed_mps_label.pack(pady=5)
        self.speed_steps_label = ttk.Label(main, text="0 шаг/с экв.", font=("", 12), foreground="gray")
        self.speed_steps_label.pack(pady=2)
        ttk.Label(main, text="vFused (колёса+IMU), не команда моторов. График вкл. автоматически.", font=("", 9), foreground="gray").pack(pady=(10, 0))
        self._schedule_speed_update()

    def _close_speed_window(self):
        """Закрыть окно скорости."""
        self.speed_enabled = False
        if self.speed_window and self.speed_window.winfo_exists():
            self.speed_window.destroy()
        self.speed_window = None

    def _schedule_speed_update(self):
        """Обновить отображение скорости."""
        try:
            if self.speed_enabled and self.speed_window and self.speed_window.winfo_exists():
                self.speed_mps_label.config(text=f"{self.speed_mps:.3f} м/с")
                self.speed_steps_label.config(text=f"{self.speed_steps:.0f} шаг/с")
                self.root.after(80, self._schedule_speed_update)
        except tk.TclError:
            pass

    def _turn_slider_changed(self, value):
        """Ползунок поворота изменён — обновить поле и отправить."""
        if self._updating:
            return
        self._updating = True
        self.turn_var.set(f"{float(value):.2f}")
        self._updating = False
        if self.serial_port and self.serial_port.is_open:
            self._send_target_speed()

    def _sync_turn_slider_from_var(self):
        """Синхронизировать ползунок поворота с полем ввода."""
        if self._updating:
            return
        try:
            v = float(self.turn_var.get())
            v = max(-1.0, min(1.0, v))
            self._updating = True
            self.turn_slider.set(v)
            self._updating = False
        except ValueError:
            pass

    def _send_target_speed(self):
        """Отправить V,speed_mps,turn — целевая скорость в м/с."""
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Ошибка", "Сначала подключитесь к Arduino")
            return
        try:
            speed = float(self.speed_target_var.get())
            turn = float(self.turn_var.get())
            speed = max(-MAX_TARGET_SPEED_MPS, min(MAX_TARGET_SPEED_MPS, speed))
            turn = max(-1.0, min(1.0, turn))
            cmd = f"V,{speed},{turn}\n"
            self.serial_port.write(cmd.encode("utf-8"))
            self.serial_port.flush()
            self._log(f">>> {cmd.strip()} (целевая скорость м/с)\n", "sent")
        except ValueError:
            messagebox.showwarning("Ошибка", "Введите число для скорости и поворота")

    def _set_and_send_speed(self, speed_mps):
        """Установить скорость в поле и отправить."""
        self.speed_target_var.set(speed_mps)
        self._send_target_speed()

    def _bind_speed_arrows(self):
        """Привязка ← и → для изменения скорости. НЕ срабатывает при фокусе на ползунке/поле ввода."""
        def _should_skip_speed():
            w = self.root.focus_get()
            if not w:
                return False
            # Не менять скорость, если фокус на ползунке, поле ввода или логе
            return isinstance(w, (tk.Entry, tk.Text, tk.Scale))

        def on_left(event):
            if _should_skip_speed():
                return
            self._adjust_speed(-SPEED_STEP_MPS)
            return "break"

        def on_right(event):
            if _should_skip_speed():
                return
            self._adjust_speed(SPEED_STEP_MPS)
            return "break"

        self.root.bind("<Left>", on_left)
        self.root.bind("<Right>", on_right)

    def _adjust_speed(self, delta):
        """Изменить скорость на delta и отправить."""
        try:
            speed = float(self.speed_target_var.get())
        except ValueError:
            speed = 0.0
        speed = max(-MAX_TARGET_SPEED_MPS, min(MAX_TARGET_SPEED_MPS, speed + delta))
        self.speed_target_var.set(f"{speed:.2f}")
        if self.serial_port and self.serial_port.is_open:
            self._send_target_speed()

    def _save_z_to_eeprom(self):
        """Отправить Z и сохранить в EEPROM (Arduino сохраняет при получении z,val)."""
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Ошибка", "Сначала подключитесь к Arduino")
            return
        try:
            val = float(self.z_var.get())
            cmd = f"z,{val}\n"
            self.serial_port.write(cmd.encode("utf-8"))
            self.serial_port.flush()
            self._log(f">>> {cmd.strip()} (сохранение Z в EEPROM)\n", "sent")
        except ValueError:
            messagebox.showwarning("Ошибка", "Введите число для Z-коррекции")
        except Exception as e:
            self._log(f"Ошибка: {e}\n")

    def _log(self, msg, tag=None):
        """Добавить в лог. tag: 'sent' (зелёный) или 'recv' (синий)."""
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, msg, tag)
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)

    def _start_read_thread(self):
        def read_loop():
            while True:
                try:
                    if self.serial_port and self.serial_port.is_open and self.read_running:
                        line = self.serial_port.readline().decode("utf-8", errors="ignore").strip()
                        if line:
                            self.read_queue.put(line)
                    else:
                        import time
                        time.sleep(0.05)
                except Exception:
                    pass

        self.read_running = True
        self.read_thread = threading.Thread(target=read_loop, daemon=True)
        self.read_thread.start()
        self._process_read_queue()

    def _process_read_queue(self):
        try:
            while True:
                line = self.read_queue.get_nowait()
                # Данные графика: 7 или 8 чисел (8-й = speed_mps)
                parts = [p.strip() for p in line.split(",")]
                if len(parts) in (7, 8):
                    try:
                        vals = tuple(float(x) for x in parts)
                        if self.graph_enabled:
                            self.graph_data.append(vals[:7])  # График использует 7
                        if self.speed_enabled or (self.speed_window and self.speed_window.winfo_exists()):
                            output = vals[6]  # motor output (шаг/с)
                            if len(vals) >= 8:
                                self.speed_mps = vals[7]  # vFused из Arduino (м/с)
                                self.speed_steps = (vals[7] / (PI * WHEEL_DIAMETER_M)) * STEPS_PER_REV
                            else:
                                self.speed_mps = (output / STEPS_PER_REV) * PI * WHEEL_DIAMETER_M
                                self.speed_steps = output
                    except ValueError:
                        pass
                    continue  # Не логируем поток графика
                self._log(f"<<< {line}\n", "recv")
                # Парсим ответ "Angle PID: Kp=... Ki=... Kd=... limit=..."
                m = re.search(r"Angle PID: Kp=([\d.]+)\s+Ki=([\d.]+)\s+Kd=([\d.]+)\s+limit=([\d.]+)", line)
                if m:
                    self._updating = True
                    self.kp_var.set(m.group(1))
                    self.ki_var.set(m.group(2))
                    self.kd_var.set(m.group(3))
                    self.limit_var.set(m.group(4))
                    self._updating = False
                    self.root.after(0, self._sync_sliders_from_vars)
                    continue
                # Парсим ответ "Speed PID: Kp=... Ki=... Kd=... limit=..."
                m2 = re.search(r"Speed PID: Kp=([\d.]+)\s+Ki=([\d.]+)\s+Kd=([\d.]+)\s+limit=([\d.]+)", line)
                if m2:
                    self._updating = True
                    kp, ki, kd, lim = m2.group(1), m2.group(2), m2.group(3), m2.group(4)
                    self.spd_kp_var.set(kp)
                    self.spd_ki_var.set(ki)
                    self.spd_kd_var.set(kd)
                    self.spd_limit_var.set(lim)
                    self.spd_status_var.set(f"Speed PID: Kp={kp} Ki={ki} Kd={kd} L={lim}")
                    self._updating = False
                    self.root.after(0, self._sync_sliders_spd_from_vars)
                    continue
                # Совместимость: "PID: Kp=..." (без Angle/Speed)
                m3 = re.search(r"PID: Kp=([\d.]+)\s+Ki=([\d.]+)\s+Kd=([\d.]+)\s+limit=([\d.]+)", line)
                if m3:
                    self._updating = True
                    self.kp_var.set(m3.group(1))
                    self.ki_var.set(m3.group(2))
                    self.kd_var.set(m3.group(3))
                    self.limit_var.set(m3.group(4))
                    self._updating = False
                    self.root.after(0, self._sync_sliders_from_vars)
        except queue.Empty:
            pass
        self.root.after(100, self._process_read_queue)

    def _on_close(self):
        self.read_running = False
        self._disconnect()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    try:
        app = PidTunerApp()
        app.run()
    except KeyboardInterrupt:
        pass  # Ctrl+C — нормальное завершение
