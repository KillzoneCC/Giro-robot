#!/usr/bin/env python3
"""
PID Tuner — приложение для управления PID-регулятором Giro-Robot
Подключение к Arduino через USB (Serial).

Интерфейс: вкладки «PID и управление», «Автокалибровка», «Графики PID», «Данные робота»;
консоль Serial под вкладками на всю ширину.
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
import time
from collections import deque

from host_autotune import AutotuneController, SweepParams, SweepPlan, AutotuneState, PidRole

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

# Частота дискретизации цикла управления (Гц) — меньше = экономия ресурсов Arduino
LOOP_HZ_OPTIONS = (25, 50, 75, 100)
DEFAULT_LOOP_HZ = 100

# Расширение и фильтр для файлов траекторий
TRAJECTORY_EXT = ".traj"
TRAJECTORY_FILTER = [("Траектории", f"*{TRAJECTORY_EXT}"), ("Все файлы", "*.*")]

# Параметры детекции подъёма: вход по накоплению ошибки, выход по уменьшению
DEFAULT_LIFT_ANGLE_ERR = 8.0
DEFAULT_LIFT_DEBOUNCE = 150
DEFAULT_LIFT_RECOVERY = 5
LIFT_ANGLE_ERR_RANGE = (1.0, 30.0)
LIFT_DEBOUNCE_RANGE = (50, 500)
LIFT_RECOVERY_RANGE = (2, 20)


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


class PidTunerApp:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("GR Giro-Robot PID Tuner")
        self.root.geometry("920x800")
        self.root.resizable(True, True)
        self.root.configure(bg="#f5f5f5")

        self.serial_port = None
        self._updating = False  # Блокировка рекурсии при синхронизации
        self._send_after_id = None  # Для debounce авто-отправки
        self._send_lift_after_id = None
        self.read_queue = queue.Queue()
        self.read_thread = None
        self.read_running = False
        self.graph_enabled = False
        self.graph_window = None
        self.graph_data = deque(maxlen=400)  # Буфер данных
        self.graph_lines = None  # Линии для set_data (без перерисовки)
        self.imu_graph_enabled = False
        self.imu_graph_window = None
        self.imu_graph_data = deque(maxlen=400)  # ax,ay,az,gx,gy,gz
        self.imu_graph_lines = None
        self.speed_window = None
        self.speed_enabled = False
        self.speed_mps = 0.0
        self.speed_steps = 0.0
        self.trajectory_window = None
        self.is_fallen = False  # Статус падения робота
        self.autotune_atz = 0
        self.autotune = AutotuneController(self)
        self._status_poll_id = None  # Периодический опрос статуса
        # Буфер для вкладки «Графики PID» (из строк TLM)
        self._graph_idx = 0
        self._graph_t = deque(maxlen=900)
        self._ga_e = deque(maxlen=900)
        self._ga_p = deque(maxlen=900)
        self._ga_i = deque(maxlen=900)
        self._ga_d = deque(maxlen=900)
        self._gs_e = deque(maxlen=900)
        self._gs_p = deque(maxlen=900)
        self._gs_i = deque(maxlen=900)
        self._gs_d = deque(maxlen=900)
        self._graph_redraw_scheduled = False
        self._g_fig = None
        self._g_canvas = None
        self._g_ax_a = None
        self._g_ax_s = None

        self._configure_styles()
        self._build_ui()
        self._start_read_thread()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _configure_styles(self):
        """Настройка стилей ttk для современного вида."""
        style = ttk.Style()
        if "clam" in style.theme_names():
            style.theme_use("clam")
        style.configure("TFrame", background="#f5f5f5")
        style.configure("TLabel", background="#f5f5f5", font=("Segoe UI", 10))
        style.configure("TLabelframe", background="#f5f5f5", font=("Segoe UI", 10, "bold"))
        style.configure("TLabelframe.Label", background="#f5f5f5", font=("Segoe UI", 10, "bold"))
        style.configure("TButton", font=("Segoe UI", 9), padding=(8, 4))
        style.map("TButton", background=[("active", "#e0e0e0")])

    def _build_ui(self):
        main = ttk.Frame(self.root, padding=12)
        main.pack(fill=tk.BOTH, expand=True)

        # === Строка 1: Заголовок + Подключение (компактно) ===
        row1 = ttk.Frame(main)
        row1.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(row1, text="GR Giro-Robot PID Tuner", font=("Segoe UI", 13, "bold")).pack(side=tk.LEFT, padx=(0, 20))
        ttk.Label(row1, text="Порт:").pack(side=tk.LEFT, padx=(0, 4))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(row1, textvariable=self.port_var, width=18, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=(0, 5))
        self.refresh_btn = ttk.Button(row1, text="Обновить", command=self._refresh_ports)
        self.refresh_btn.pack(side=tk.LEFT, padx=(0, 5))
        self.connect_btn = ttk.Button(row1, text="Подключить", command=self._toggle_connect)
        self.connect_btn.pack(side=tk.LEFT, padx=(0, 15))
        self.status_label = ttk.Label(row1, text="Отключено", foreground="gray", font=("", 10))
        self.status_label.pack(side=tk.LEFT, padx=(0, 10))
        # Индикатор падения
        self.fall_frame = ttk.Frame(row1)
        self.fall_frame.pack(side=tk.LEFT)
        self.fall_indicator = tk.Label(self.fall_frame, text="●", font=("", 14), fg="#888",
                                       bg="#f5f5f5", padx=4)
        self.fall_indicator.pack(side=tk.LEFT)
        self.fall_label = ttk.Label(self.fall_frame, text="—", foreground="gray", font=("", 9))
        self.fall_label.pack(side=tk.LEFT)
        self._active_slider_label_var = tk.StringVar(value="—")

        # Вкладки: основная работа | автокалибровка | снимки с робота
        self.main_tabs = ttk.Notebook(main)
        self.main_tabs.pack(fill=tk.BOTH, expand=True, pady=(0, 6))

        tab_pid = ttk.Frame(self.main_tabs, padding=4)
        tab_auto = ttk.Frame(self.main_tabs, padding=4)
        tab_graphs = ttk.Frame(self.main_tabs, padding=4)
        tab_data = ttk.Frame(self.main_tabs, padding=4)
        self.main_tabs.add(tab_pid, text="PID и управление")
        self.main_tabs.add(tab_auto, text="Автокалибровка")
        self.main_tabs.add(tab_graphs, text="Графики PID")
        self.main_tabs.add(tab_data, text="Данные робота")
        self._tab_graphs_index = 2  # индекс вкладки «Графики PID» в Notebook
        self.main_tabs.bind("<<NotebookTabChanged>>", self._on_notebook_tab_changed)

        # === Две колонки: PID слева, Скорость справа (вкладка 1) ===
        cols = ttk.Frame(tab_pid)
        cols.pack(fill=tk.BOTH, expand=True)
        left_col = ttk.Frame(cols)
        left_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 12))
        right_col = ttk.Frame(cols)
        right_col.pack(side=tk.LEFT, fill=tk.X)

        ttk.Label(left_col, text="↑↓ ←→ меняют значение (клик на ползунок)", font=("", 8), foreground="gray").pack(anchor=tk.W)

        # === Angle PID (угол → моторы) ===
        pid_frame = ttk.LabelFrame(left_col, text="ANGLE PID (угол → моторы)", padding=5)
        pid_frame.pack(fill=tk.X, pady=(2, 4))

        self.kp_var = tk.StringVar(value=str(DEFAULT_KP))
        self.ki_var = tk.StringVar(value=str(DEFAULT_KI))
        self.kd_var = tk.StringVar(value=str(DEFAULT_KD))
        self.limit_var = tk.StringVar(value=str(DEFAULT_LIMIT))
        self.angle_step_var = tk.StringVar(value="0.01")

        step_row = ttk.Frame(pid_frame)
        step_row.pack(fill=tk.X, pady=(0, 4))
        ttk.Label(step_row, text="Шаг ↑↓←→:").pack(side=tk.LEFT, padx=(0, 5))
        angle_step_combo = ttk.Combobox(step_row, textvariable=self.angle_step_var, values=PID_STEP_OPTIONS,
                                        width=8, state="readonly")
        angle_step_combo.pack(side=tk.LEFT, padx=(0, 10))
        ttk.Label(step_row, text="Клик на ползунок → ↑↓ или ←→ для изменения", font=("", 9), foreground="gray").pack(side=tk.LEFT)

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

        # === Speed PID (скорость → угол) ===
        spd_frame = ttk.LabelFrame(left_col, text="SPEED PID (скорость → угол)", padding=5)
        spd_frame.pack(fill=tk.X, pady=(0, 4))

        self.spd_kp_var = tk.StringVar(value=str(DEFAULT_SPD_KP))
        self.spd_ki_var = tk.StringVar(value=str(DEFAULT_SPD_KI))
        self.spd_kd_var = tk.StringVar(value=str(DEFAULT_SPD_KD))
        self.spd_limit_var = tk.StringVar(value=str(DEFAULT_SPD_LIMIT))
        self.spd_step_var = tk.StringVar(value="0.01")

        spd_step_row = ttk.Frame(spd_frame)
        spd_step_row.pack(fill=tk.X, pady=(0, 4))
        ttk.Label(spd_step_row, text="Шаг ↑↓←→:").pack(side=tk.LEFT, padx=(0, 5))
        spd_step_combo = ttk.Combobox(spd_step_row, textvariable=self.spd_step_var, values=PID_STEP_OPTIONS,
                                       width=8, state="readonly")
        spd_step_combo.pack(side=tk.LEFT, padx=(0, 10))
        ttk.Label(spd_step_row, text="Клик на ползунок → ↑↓ или ←→ для изменения", font=("", 9), foreground="gray").pack(side=tk.LEFT)

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
        spd_status_row.pack(fill=tk.X, pady=4)
        self.spd_status_var = tk.StringVar(value="Speed PID на Arduino: —")
        ttk.Label(spd_status_row, textvariable=self.spd_status_var, font=("", 9), foreground="blue").pack(side=tk.LEFT)
        ttk.Button(spd_status_row, text="Прочитать Speed PID", command=self._read_speed_pid_only).pack(side=tk.LEFT, padx=(15, 0))
        # Z-коррекция
        z_row = ttk.Frame(pid_frame)
        z_row.pack(fill=tk.X, pady=4)
        ttk.Label(z_row, text="Z:", width=6).pack(side=tk.LEFT, padx=(0, 5))
        self.z_var = tk.StringVar(value="0")
        ttk.Entry(z_row, textvariable=self.z_var, width=12).pack(side=tk.LEFT, padx=(0, 5))
        ttk.Button(z_row, text="Сохранить Z в EEPROM", command=self._save_z_to_eeprom).pack(side=tk.LEFT)
        ttk.Label(z_row, text="(коррекция нуля, градусы)", foreground="gray").pack(side=tk.LEFT, padx=(8, 0))

        # === Правая колонка: Частота + Скорость + Кнопки ===
        hz_frame = ttk.LabelFrame(right_col, text="Частота дискретизации (Гц)", padding=6)
        hz_frame.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(hz_frame, text="Меньше Гц = меньше нагрузка на Arduino", font=("", 8), foreground="gray").pack(anchor=tk.W)
        hz_row = ttk.Frame(hz_frame)
        hz_row.pack(fill=tk.X, pady=4)
        self.loop_hz_var = tk.StringVar(value=str(DEFAULT_LOOP_HZ))
        hz_combo = ttk.Combobox(hz_row, textvariable=self.loop_hz_var, values=[str(h) for h in LOOP_HZ_OPTIONS],
                                width=6, state="readonly")
        hz_combo.pack(side=tk.LEFT, padx=(0, 6))
        ttk.Label(hz_row, text="Гц").pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(hz_row, text="Установить", command=self._send_loop_hz).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(hz_row, text="Прочитать", command=self._read_loop_hz).pack(side=tk.LEFT)
        self.loop_hz_var.trace_add("write", lambda *a: self._update_params_display())

        speed_frame = ttk.LabelFrame(right_col, text="Целевая скорость (м/с)", padding=6)
        speed_frame.pack(fill=tk.X, pady=(0, 6))
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
        set_btn = tk.Button(speed_row, text="Установить", command=self._send_target_speed,
                           bg="#4caf50", fg="white", font=("Segoe UI", 9), relief=tk.FLAT, padx=12, pady=4,
                           cursor="hand2", activebackground="#43a047", activeforeground="white")
        set_btn.pack(side=tk.LEFT, padx=(0, 10))
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
        ttk.Label(quick_row, text="Предустановки:", foreground="gray").pack(side=tk.LEFT, padx=(0, 8))
        for label, val in [("Стоп (0)", "0"), ("0.3", "0.3"), ("0.5", "0.5"), ("1.0", "1.0")]:
            btn = ttk.Button(quick_row, text=label, width=7, command=lambda v=val: self._set_and_send_speed(v))
            btn.pack(side=tk.LEFT, padx=3)
        ttk.Label(speed_frame, text="0 м/с = остановка и баланс на месте", font=("", 9), foreground="gray").pack(anchor=tk.W)
        ttk.Label(speed_frame, text="← → меняют скорость (когда фокус не на ползунке/поле)", font=("", 9), foreground="gray").pack(anchor=tk.W)

        # === ПОДЪЁМ (lift detection) ===
        lift_frame = ttk.LabelFrame(right_col, text="ПОДЪЁМ (вход: ошибка накапл., выход: уменьшается)", padding=6)
        lift_frame.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(lift_frame, text="Мин. ошибка °, debounce мс, отсчётов для выхода", font=("", 8), foreground="gray").pack(anchor=tk.W)
        self.lift_angle_err_var = tk.StringVar(value=str(DEFAULT_LIFT_ANGLE_ERR))
        self.lift_debounce_var = tk.StringVar(value=str(DEFAULT_LIFT_DEBOUNCE))
        self.lift_recovery_var = tk.StringVar(value=str(DEFAULT_LIFT_RECOVERY))

        def add_lift_row(parent, label, str_var, slider_range, resolution):
            row = tk.Frame(parent, bg="#f0f0f0", padx=4, pady=2)
            row.pack(fill=tk.X, pady=2)
            lbl = ttk.Label(row, text=label, width=10)
            lbl.pack(side=tk.LEFT, padx=(0, 5))
            slider = tk.Scale(row, from_=slider_range[0], to=slider_range[1], resolution=resolution,
                             orient=tk.HORIZONTAL, length=180, showvalue=1, takefocus=1,
                             command=lambda v, sv=str_var: self._slider_changed_lift(sv, v),
                             highlightthickness=0)
            slider.pack(side=tk.LEFT, padx=(0, 8))
            entry = ttk.Entry(row, textvariable=str_var, width=8)
            entry.pack(side=tk.LEFT)
            return slider

        self.lift_angle_err_slider = add_lift_row(lift_frame, "Ошибка °:", self.lift_angle_err_var, LIFT_ANGLE_ERR_RANGE, 1.0)
        self.lift_debounce_slider = add_lift_row(lift_frame, "Debounce:", self.lift_debounce_var, LIFT_DEBOUNCE_RANGE, 10.0)
        self.lift_recovery_slider = add_lift_row(lift_frame, "Выход (N):", self.lift_recovery_var, LIFT_RECOVERY_RANGE, 1.0)
        lift_btn_row = ttk.Frame(lift_frame)
        lift_btn_row.pack(fill=tk.X, pady=4)
        ttk.Button(lift_btn_row, text="Установить", command=self._send_lift_params).pack(side=tk.LEFT, padx=(0, 5))
        ttk.Button(lift_btn_row, text="Прочитать", command=self._read_lift_from_arduino).pack(side=tk.LEFT)

        self._bind_speed_arrows()

        self._sync_sliders_from_vars()
        self._sync_sliders_spd_from_vars()
        self._sync_lift_sliders_from_vars()
        self._sync_turn_slider_from_var()

        # Привязка: при изменении поля ввода — обновить ползунок и отправить
        for var in (self.kp_var, self.ki_var, self.kd_var, self.limit_var):
            var.trace_add("write", lambda *a: self._on_param_changed())
        for var in (self.spd_kp_var, self.spd_ki_var, self.spd_kd_var, self.spd_limit_var):
            var.trace_add("write", lambda *a: self._on_param_changed_spd())
        for var in (self.kp_var, self.ki_var, self.kd_var, self.limit_var,
                    self.spd_kp_var, self.spd_ki_var, self.spd_kd_var, self.spd_limit_var,
                    self.speed_target_var, self.turn_var,
                    self.lift_angle_err_var, self.lift_debounce_var, self.lift_recovery_var):
            var.trace_add("write", lambda *a: self._update_params_display())
        for var in (self.lift_angle_err_var, self.lift_debounce_var, self.lift_recovery_var):
            var.trace_add("write", lambda *a: self._sync_lift_sliders_from_vars())
        self.turn_var.trace_add("write", lambda *a: self._sync_turn_slider_from_var())

        btn_frame = ttk.LabelFrame(right_col, text="Действия", padding=6)
        btn_frame.pack(fill=tk.X, pady=(0, 6))
        self.read_btn = ttk.Button(btn_frame, text="Прочитать PID", command=self._read_from_arduino)
        self.read_btn.pack(fill=tk.X, pady=2)
        self.save_btn = ttk.Button(btn_frame, text="Сохранить Angle PID", command=self._save_to_eeprom)
        self.save_btn.pack(fill=tk.X, pady=2)
        self.save_spd_btn = ttk.Button(btn_frame, text="Сохранить Speed PID", command=self._save_speed_pid_to_eeprom)
        self.save_spd_btn.pack(fill=tk.X, pady=2)
        self.graph_btn = ttk.Button(btn_frame, text="График PID", command=self._toggle_graph)
        self.graph_btn.pack(fill=tk.X, pady=2)
        self.imu_graph_btn = ttk.Button(btn_frame, text="График IMU", command=self._toggle_imu_graph)
        self.imu_graph_btn.pack(fill=tk.X, pady=2)
        self.speed_btn = ttk.Button(btn_frame, text="Скорость моторов", command=self._toggle_speed_window)
        self.speed_btn.pack(fill=tk.X, pady=2)
        self.traj_btn = ttk.Button(btn_frame, text="Траектории", command=self._open_trajectory_window)
        self.traj_btn.pack(fill=tk.X, pady=2)

        self._params_display_var = tk.StringVar()
        ttk.Label(right_col, textvariable=self._params_display_var, font=("", 8), foreground="gray").pack(anchor=tk.W)
        ttk.Label(
            right_col,
            text="Регулятор PID выполняется на Arduino; ПК отправляет коэффициенты (f / f2), сохранение в EEPROM — w / w2.",
            font=("", 8),
            foreground="#555",
            wraplength=280,
        ).pack(anchor=tk.W, pady=(2, 0))

        # --- Вкладка «Автокалибровка» ---
        at_plan_fr = ttk.LabelFrame(tab_auto, text="Режим sweep Kp", padding=8)
        at_plan_fr.pack(fill=tk.X, expand=False)
        self.at_plan_var = tk.StringVar(value="angle")
        pr = ttk.Frame(at_plan_fr)
        pr.pack(fill=tk.X)
        ttk.Radiobutton(pr, text="Только Angle PID", variable=self.at_plan_var, value="angle").pack(side=tk.LEFT, padx=(0, 12))
        ttk.Radiobutton(pr, text="Только Speed PID", variable=self.at_plan_var, value="speed").pack(side=tk.LEFT, padx=(0, 12))
        ttk.Radiobutton(pr, text="Оба подряд (Angle → Speed)", variable=self.at_plan_var, value="both").pack(side=tk.LEFT)

        atf = ttk.LabelFrame(tab_auto, text="Angle PID — диапазон Kp (f), пауза при падении", padding=8)
        atf.pack(fill=tk.X, expand=False)
        ar = ttk.Frame(atf)
        ar.pack(fill=tk.X)
        self.at_angle_only_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(ar, text="Только внутренний контур (a,2)", variable=self.at_angle_only_var).pack(side=tk.LEFT, padx=(0, 12))
        ttk.Label(ar, text="Kp:").pack(side=tk.LEFT)
        self.at_kp0_var = tk.StringVar(value="260")
        self.at_kp1_var = tk.StringVar(value="360")
        self.at_kps_var = tk.StringVar(value="15")
        ttk.Entry(ar, width=6, textvariable=self.at_kp0_var).pack(side=tk.LEFT, padx=2)
        ttk.Label(ar, text="…").pack(side=tk.LEFT)
        ttk.Entry(ar, width=6, textvariable=self.at_kp1_var).pack(side=tk.LEFT, padx=2)
        ttk.Label(ar, text="шаг").pack(side=tk.LEFT)
        ttk.Entry(ar, width=5, textvariable=self.at_kps_var).pack(side=tk.LEFT, padx=2)

        at_sp = ttk.LabelFrame(tab_auto, text="Speed PID — диапазон Kp (f2); нужен для режимов «Только Speed» и «Оба подряд»", padding=8)
        at_sp.pack(fill=tk.X, expand=False)
        sr = ttk.Frame(at_sp)
        sr.pack(fill=tk.X)
        ttk.Label(sr, text="Kp:").pack(side=tk.LEFT)
        self.at_sp_kp0_var = tk.StringVar(value="2")
        self.at_sp_kp1_var = tk.StringVar(value="12")
        self.at_sp_kps_var = tk.StringVar(value="1")
        ttk.Entry(sr, width=6, textvariable=self.at_sp_kp0_var).pack(side=tk.LEFT, padx=2)
        ttk.Label(sr, text="…").pack(side=tk.LEFT)
        ttk.Entry(sr, width=6, textvariable=self.at_sp_kp1_var).pack(side=tk.LEFT, padx=2)
        ttk.Label(sr, text="шаг").pack(side=tk.LEFT)
        ttk.Entry(sr, width=5, textvariable=self.at_sp_kps_var).pack(side=tk.LEFT, padx=2)
        ttk.Label(sr, text="Ki,Kd,L берутся из полей Speed PID на главной вкладке.", font=("", 9), foreground="gray").pack(side=tk.LEFT, padx=(12, 0))

        at_common = ttk.LabelFrame(tab_auto, text="Общие параметры шага", padding=8)
        at_common.pack(fill=tk.X, expand=False)
        cr = ttk.Frame(at_common)
        cr.pack(fill=tk.X)
        ttk.Label(cr, text="сек/шаг").pack(side=tk.LEFT)
        self.at_step_s_var = tk.StringVar(value="3")
        ttk.Entry(cr, width=4, textvariable=self.at_step_s_var).pack(side=tk.LEFT, padx=2)
        ttk.Label(cr, text="max× пересечений ошибки").pack(side=tk.LEFT, padx=(12, 0))
        self.at_cross_var = tk.StringVar(value="12")
        ttk.Entry(cr, width=4, textvariable=self.at_cross_var).pack(side=tk.LEFT, padx=2)

        ar2 = ttk.Frame(tab_auto)
        ar2.pack(fill=tk.X, pady=(8, 0))
        ttk.Button(ar2, text="Старт sweep", command=self._autotune_start).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(ar2, text="Стоп", command=self._autotune_stop).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(ar2, text="Продолжить после падения", command=self._autotune_resume).pack(side=tk.LEFT, padx=(0, 6))
        self.at_status_var = tk.StringVar(value="Автотюн: выкл")
        ttk.Label(ar2, textvariable=self.at_status_var, font=("", 9)).pack(side=tk.LEFT, padx=(12, 0))

        self.at_angle_live_var = tk.StringVar(
            value="Angle PID: sweep не запущен.\nВыберите диапазон Kp и «Старт» — здесь будут Kp, Ki, Kd, L и счётчик пересечений ошибки."
        )
        self.at_speed_live_var = tk.StringVar(
            value="Speed PID: sweep не запущен.\nРежим «Оба подряд»: сначала смотрите левый журнал и панель Angle, затем — Speed справа."
        )
        at_live_pan = ttk.PanedWindow(tab_auto, orient=tk.HORIZONTAL)
        at_live_pan.pack(fill=tk.BOTH, expand=False, pady=(10, 6))
        lf_live_a = ttk.LabelFrame(at_live_pan, text="Angle PID — текущий шаг (выбор баланса)", padding=8)
        lf_live_s = ttk.LabelFrame(at_live_pan, text="Speed PID — текущий шаг", padding=8)
        at_live_pan.add(lf_live_a, weight=1)
        at_live_pan.add(lf_live_s, weight=1)
        ttk.Label(lf_live_a, textvariable=self.at_angle_live_var, justify=tk.LEFT, wraplength=400, font=("", 9)).pack(anchor=tk.W)
        ttk.Label(lf_live_s, textvariable=self.at_speed_live_var, justify=tk.LEFT, wraplength=400, font=("", 9)).pack(anchor=tk.W)

        at_split = ttk.PanedWindow(tab_auto, orient=tk.HORIZONTAL)
        at_split.pack(fill=tk.BOTH, expand=True, pady=(4, 4))
        lf_at_a = ttk.LabelFrame(at_split, text="Журнал автотюна — Angle PID", padding=4)
        lf_at_s = ttk.LabelFrame(at_split, text="Журнал автотюна — Speed PID", padding=4)
        at_split.add(lf_at_a, weight=1)
        at_split.add(lf_at_s, weight=1)
        self.at_log_angle = scrolledtext.ScrolledText(
            lf_at_a,
            height=14,
            width=44,
            state=tk.DISABLED,
            wrap=tk.WORD,
            font=("Consolas", 9),
            bg="#faf8f5",
            fg="#1a1a1a",
        )
        self.at_log_speed = scrolledtext.ScrolledText(
            lf_at_s,
            height=14,
            width=44,
            state=tk.DISABLED,
            wrap=tk.WORD,
            font=("Consolas", 9),
            bg="#f2f8fa",
            fg="#1a1a1a",
        )
        self.at_log_angle.pack(fill=tk.BOTH, expand=True)
        self.at_log_speed.pack(fill=tk.BOTH, expand=True)

        at_log_btns = ttk.Frame(tab_auto)
        at_log_btns.pack(fill=tk.X, pady=(0, 4))
        ttk.Button(at_log_btns, text="Очистить журнал Angle", command=lambda: self._clear_autotune_log("angle")).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(at_log_btns, text="Очистить журнал Speed", command=lambda: self._clear_autotune_log("speed")).pack(side=tk.LEFT)

        ttk.Label(
            tab_auto,
            text="Автотюн не записывает EEPROM — после подбора сохраните вручную «Сохранить Angle PID» (w) и «Сохранить Speed PID» (w2) на вкладке PID.",
            font=("", 9),
            foreground="#333",
            wraplength=820,
            justify=tk.LEFT,
        ).pack(anchor=tk.W, pady=(4, 0))

        ttk.Label(
            tab_auto,
            text="Баланс на месте в режимах Angle — прошивка a,1/a,2; Speed sweep — каскад a,1 и команды f2. После падения — «Продолжить после падения».",
            font=("", 9),
            foreground="gray",
            wraplength=820,
            justify=tk.LEFT,
        ).pack(anchor=tk.W, pady=(8, 0))

        # --- Вкладка «Графики PID» (ошибки и P/I/D из TLM) ---
        gf_top = ttk.Frame(tab_graphs)
        gf_top.pack(fill=tk.X, pady=(0, 6))
        ttk.Button(gf_top, text="Очистить графики", command=self._clear_pid_graphs).pack(side=tk.LEFT, padx=(0, 10))
        ttk.Label(
            gf_top,
            text="Включите машинную телеметрию на роботе (x,1). Обновление из строк TLM.",
            font=("", 9),
            foreground="gray",
        ).pack(side=tk.LEFT)
        if HAS_MATPLOTLIB:
            self._g_fig = Figure(figsize=(9, 6), dpi=100)
            self._g_ax_a = self._g_fig.add_subplot(2, 1, 1)
            self._g_ax_s = self._g_fig.add_subplot(2, 1, 2)
            self._pid_graph_init_axes()
            self._g_canvas = FigureCanvasTkAgg(self._g_fig, master=tab_graphs)
            self._g_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        else:
            ttk.Label(
                tab_graphs,
                text="Для графиков установите matplotlib: pip install matplotlib",
                foreground="gray",
            ).pack(anchor=tk.W, pady=12)

        # --- Вкладка «Данные робота»: статус, телеметрия TLM, сводка PID в приложении ---
        data_cols = ttk.Frame(tab_data)
        data_cols.pack(fill=tk.BOTH, expand=True)
        lf_stat = ttk.LabelFrame(data_cols, text="Последний STATUS (опрос ?)", padding=8)
        lf_stat.pack(fill=tk.X, pady=(0, 8))
        self.robot_status_snapshot_var = tk.StringVar(
            value="Подключитесь — здесь будет строка STATUS fall=… hz=… atz=…"
        )
        ttk.Label(lf_stat, textvariable=self.robot_status_snapshot_var, font=("Consolas", 10)).pack(anchor=tk.W)

        lf_tlm = ttk.LabelFrame(data_cols, text="Последняя строка телеметрии TLM (на роботе: x,1)", padding=8)
        lf_tlm.pack(fill=tk.BOTH, expand=True, pady=(0, 8))
        self.telemetry_snap_text = scrolledtext.ScrolledText(
            lf_tlm,
            height=12,
            state=tk.DISABLED,
            wrap=tk.WORD,
            font=("Consolas", 9),
            bg="#fafafa",
            fg="#222",
        )
        self.telemetry_snap_text.pack(fill=tk.BOTH, expand=True)

        lf_local = ttk.LabelFrame(data_cols, text="Коэффициенты в приложении (ОЗУ; EEPROM после «Сохранить» на вкладке PID)", padding=8)
        lf_local.pack(fill=tk.X)
        self.pid_snapshot_var = tk.StringVar(value="—")
        ttk.Label(lf_local, textvariable=self.pid_snapshot_var, font=("Consolas", 9), justify=tk.LEFT).pack(anchor=tk.W)

        snap_btns = ttk.Frame(data_cols)
        snap_btns.pack(fill=tk.X, pady=(8, 0))
        ttk.Button(snap_btns, text="Обновить сводку PID из полей", command=self._refresh_pid_snapshot_tab).pack(side=tk.LEFT, padx=(0, 8))

        # консоль — ниже вкладок
        log_frame = ttk.LabelFrame(main, text="Консоль Serial — команды и ответы Arduino", padding=6)
        log_frame.pack(fill=tk.BOTH, expand=True, pady=(5, 0))
        log_header = ttk.Frame(log_frame)
        log_header.pack(fill=tk.X, pady=(0, 4))
        ttk.Button(log_header, text="Очистить консоль", command=self._clear_log).pack(side=tk.RIGHT)
        self.log_text = scrolledtext.ScrolledText(log_frame, height=16, state=tk.DISABLED, wrap=tk.WORD,
                                                  font=("Consolas", 10), bg="#1e1e1e", fg="#d4d4d4",
                                                  insertbackground="white", selectbackground="#264f78")
        self.log_text.pack(fill=tk.BOTH, expand=True)
        self.log_text.tag_configure("sent", foreground="#4ec9b0")
        self.log_text.tag_configure("recv", foreground="#9cdcfe")

        self._refresh_ports()
        self._update_params_display()

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

    def _slider_changed_lift(self, var, value):
        """Lift: ползунок изменён."""
        if self._updating:
            return
        self._updating = True
        var.set(str(float(value)))
        self._updating = False
        if self._send_lift_after_id:
            self.root.after_cancel(self._send_lift_after_id)
        self._send_lift_after_id = self.root.after(150, self._send_lift_auto)

    def _send_lift_auto(self):
        self._send_lift_after_id = None
        self._send_lift_params()

    def _send_lift_params(self):
        """Отправить параметры lift на Arduino."""
        if not self.serial_port or not self.serial_port.is_open:
            return
        try:
            ae = float(self.lift_angle_err_var.get())
            db = float(self.lift_debounce_var.get())
            rs = float(self.lift_recovery_var.get())
            ae = max(LIFT_ANGLE_ERR_RANGE[0], min(LIFT_ANGLE_ERR_RANGE[1], ae))
            db = max(LIFT_DEBOUNCE_RANGE[0], min(LIFT_DEBOUNCE_RANGE[1], db))
            rs = max(LIFT_RECOVERY_RANGE[0], min(LIFT_RECOVERY_RANGE[1], rs))
            cmd = f"L,{ae},{int(db)},{int(rs)}\n"
            self.serial_port.write(cmd.encode())
            self.serial_port.flush()
            self._log(f">>> {cmd.strip()}\n", "sent")
            self._sync_lift_sliders_from_vars()
        except (ValueError, Exception):
            pass

    def _read_lift_from_arduino(self):
        """Запросить текущие параметры lift с Arduino."""
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Ошибка", "Сначала подключитесь к Arduino")
            return
        try:
            self.serial_port.write(b"L\n")
            self.serial_port.flush()
            self._log(">>> L\n", "sent")
        except Exception:
            pass

    def _sync_lift_sliders_from_vars(self):
        """Синхронизировать ползунки lift с переменными."""
        self._updating = True
        try:
            ae = float(self.lift_angle_err_var.get())
            db = float(self.lift_debounce_var.get())
            rs = float(self.lift_recovery_var.get())
            self.lift_angle_err_slider.set(ae)
            self.lift_debounce_slider.set(db)
            self.lift_recovery_slider.set(rs)
        except ValueError:
            pass
        self._updating = False

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

    def _refresh_pid_snapshot_tab(self):
        """Обновить текстовую сводку PID на вкладке «Данные робота»."""
        self._update_params_display()

    def _format_tlm_snapshot(self, sline: str) -> str:
        """Человекочитаемый разбор строки TLM,... от прошивки."""
        parts = [p.strip() for p in sline.split(",")]
        if len(parts) < 21 or parts[0] != "TLM":
            return sline
        labels = (
            ("fall (0 работа)", 1),
            ("pitch °", 2),
            ("gyro pitch °/с", 3),
            ("целевой угол °", 4),
            ("мотор шаг/с", 5),
            ("Speed PID err", 6),
            ("Speed P", 7),
            ("Speed I", 8),
            ("Speed D", 9),
            ("Angle PID err", 10),
            ("Angle P", 11),
            ("Angle I", 12),
            ("Angle D", 13),
            ("angleOffset °", 14),
            ("v fused м/с", 15),
            ("v колёса м/с", 16),
            ("режим автотюна atz", 17),
            ("recovery settle", 18),
            ("цель скорости м/с", 19),
            ("стабилизация", 20),
        )
        lines = []
        for title, idx in labels:
            if idx < len(parts):
                lines.append(f"{title}: {parts[idx]}")
        return "\n".join(lines)

    def _set_telemetry_snap_ui(self, text: str) -> None:
        try:
            self.telemetry_snap_text.configure(state=tk.NORMAL)
            self.telemetry_snap_text.delete("1.0", tk.END)
            self.telemetry_snap_text.insert(tk.END, text)
            self.telemetry_snap_text.configure(state=tk.DISABLED)
        except tk.TclError:
            pass

    def _update_params_display(self):
        """Обновить панель текущих параметров."""
        try:
            a = (self.kp_var.get(), self.ki_var.get(), self.kd_var.get(), self.limit_var.get())
            s = (self.spd_kp_var.get(), self.spd_ki_var.get(), self.spd_kd_var.get(), self.spd_limit_var.get())
            hz = self.loop_hz_var.get() if hasattr(self, "loop_hz_var") else "100"
            text = (
                f"Angle: Kp={a[0]} Ki={a[1]} Kd={a[2]} L={a[3]}  |  "
                f"Speed: Kp={s[0]} Ki={s[1]} Kd={s[2]} L={s[3]}  |  "
                f"Hz={hz}  Скорость={self.speed_target_var.get()} м/с  Поворот={self.turn_var.get()}"
            )
            self._params_display_var.set(text)
            if hasattr(self, "pid_snapshot_var"):
                snap = (
                    f"Angle PID — Kp={a[0]}  Ki={a[1]}  Kd={a[2]}  Limit={a[3]}\n"
                    f"Speed PID — Kp={s[0]}  Ki={s[1]}  Kd={s[2]}  Limit={s[3]}\n"
                    f"Цикл {hz} Гц   цель V={self.speed_target_var.get()} м/с   поворот={self.turn_var.get()}"
                )
                self.pid_snapshot_var.set(snap)
        except Exception:
            self._params_display_var.set("—")
            if hasattr(self, "pid_snapshot_var"):
                self.pid_snapshot_var.set("—")

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
            self.read_running = True  # после «Отключить» поток чтения останавливался — без этого Serial не парсится
            self._log(f"Подключено к {port}\n")
            self._start_status_poll()
        except Exception as e:
            messagebox.showerror("Ошибка", f"Не удалось подключиться:\n{e}")

    def _disconnect(self):
        self._stop_status_poll()
        self.read_running = False
        self.graph_enabled = False
        self.speed_enabled = False
        self._close_graph_window()
        self._close_imu_graph_window()
        self._close_speed_window()
        try:
            self.autotune.stop()
        except Exception:
            pass
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.serial_port = None
        self.status_label.config(text="Отключено", foreground="gray")
        self.connect_btn.config(text="Подключить")
        self.port_combo.config(state="readonly")
        self._update_fall_indicator(force_reset=True)
        self._log("Отключено\n")

    def _update_fall_indicator(self, force_reset=False):
        """Обновить индикатор падения (красный = упал, зелёный = стоит)."""
        try:
            if force_reset or not (self.serial_port and self.serial_port.is_open):
                self.fall_indicator.config(fg="#888", text="●")
                self.fall_label.config(text="—", foreground="gray")
            elif self.is_fallen:
                self.fall_indicator.config(fg="#c62828", text="●")  # красный
                self.fall_label.config(text="Упал (моторы заблокированы)", foreground="#c62828")
            else:
                self.fall_indicator.config(fg="#2e7d32", text="●")  # зелёный
                self.fall_label.config(text="Стоит", foreground="#2e7d32")
        except tk.TclError:
            pass

    def _start_status_poll(self):
        """Запустить периодический опрос статуса падения."""
        self._stop_status_poll()
        self._poll_status()

    def _stop_status_poll(self):
        if self._status_poll_id:
            self.root.after_cancel(self._status_poll_id)
            self._status_poll_id = None

    def _poll_status(self):
        """Отправить ? и запланировать следующий опрос."""
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(b"?\n")
                self.serial_port.flush()
            except Exception:
                pass
        self._status_poll_id = self.root.after(300, self._poll_status)

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

    def _drain_read_queue(self):
        """Очистить очередь строк от фонового потока (перед синхронным чтением)."""
        while True:
            try:
                self.read_queue.get_nowait()
            except queue.Empty:
                break

    def _serial_readline_blocking(self, timeout_sec: float) -> str:
        """Читать строку с порта; только пока read_running=False (нет конкурирующего потока)."""
        old = self.serial_port.timeout
        self.serial_port.timeout = max(0.05, timeout_sec)
        try:
            raw = self.serial_port.readline()
            return raw.decode("utf-8", errors="ignore").strip()
        finally:
            self.serial_port.timeout = old

    def _suspend_telemetry_stream(self):
        """
        Стоп потока TLM и фонового чтения: при 115200 поток TLM @100 Гц переполняет USB,
        ответы P/P2 теряются. Отключаем x,0 и читаем порт только синхронно.
        """
        self._stop_status_poll()
        self.read_running = False
        time.sleep(0.04)
        self._drain_read_queue()
        try:
            self.serial_port.write(b"x,0\n")
            self.serial_port.flush()
            self._log(">>> x,0 (пауза телеметрии для чтения PID)\n", "sent")
        except Exception:
            pass
        time.sleep(0.14)
        try:
            self.serial_port.reset_input_buffer()
        except Exception:
            pass

    def _ensure_csv_if_graphs_tab_active(self):
        """Вкладка «Графики PID» без отдельного окна тоже нуждается в TLM."""
        try:
            tabs = self.main_tabs.tabs()
            if len(tabs) > self._tab_graphs_index and self.main_tabs.select() == tabs[self._tab_graphs_index]:
                self._ensure_csv_telemetry_for_graphs()
        except (tk.TclError, AttributeError):
            pass

    def _on_notebook_tab_changed(self, _evt=None):
        if self.serial_port and self.serial_port.is_open:
            self._ensure_csv_if_graphs_tab_active()

    def _resume_telemetry_stream(self):
        """Вернуть фоновое чтение и телеметрию для графиков (если нужно)."""
        self.read_running = True
        if getattr(self, "graph_enabled", False):
            self._ensure_csv_telemetry_for_graphs()
        else:
            self._ensure_csv_if_graphs_tab_active()
        self._start_status_poll()

    def _read_from_arduino(self):
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Ошибка", "Сначала подключитесь к Arduino")
            return
        self._suspend_telemetry_stream()
        try:
            self.serial_port.write(b"P\n")
            self.serial_port.flush()
            self._log(">>> P (запрос Angle PID)\n", "sent")
            line = self._serial_readline_blocking(0.65)
            if line:
                self._handle_firmware_text_line(line)

            self.serial_port.write(b"P2\n")
            self.serial_port.flush()
            self._log(">>> P2 (запрос Speed PID)\n", "sent")
            line = self._serial_readline_blocking(0.65)
            if line:
                self._handle_firmware_text_line(line)

            self.serial_port.write(b"H\n")
            self.serial_port.flush()
            self._log(">>> H (запрос частоты цикла)\n", "sent")
            line = self._serial_readline_blocking(0.65)
            if line:
                self._handle_firmware_text_line(line)
        except Exception as e:
            self._log(f"Ошибка чтения PID: {e}\n")
            messagebox.showwarning("Чтение PID", f"Ошибка обмена с портом:\n{e}")
        finally:
            self._resume_telemetry_stream()

    def _read_speed_pid_only(self):
        """Запросить только Speed PID (для отображения значений)."""
        if not self.serial_port or not self.serial_port.is_open:
            return
        self._suspend_telemetry_stream()
        try:
            self.serial_port.write(b"P2\n")
            self.serial_port.flush()
            self._log(">>> P2 (запрос Speed PID)\n", "sent")
            line = self._serial_readline_blocking(0.65)
            if line:
                self._handle_firmware_text_line(line)
        except Exception as e:
            self._log(f"Ошибка: {e}\n")
        finally:
            self._resume_telemetry_stream()

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
        """Окно «График PID»: точки из телеметрии TLM (команда x,1). На Nano поток по G отключён (Flash)."""
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
            self._log(f">>> G (график ПК {'вкл' if self.graph_enabled else 'выкл'}; данные из TLM после x,1)\n")
        except Exception as e:
            self._log(f"Ошибка: {e}\n")
            self.graph_enabled = False
            return
        if self.graph_enabled:
            self._ensure_csv_telemetry_for_graphs()
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

    def _toggle_imu_graph(self):
        """Вкл/выкл графика IMU."""
        if not HAS_MATPLOTLIB:
            messagebox.showerror("Ошибка", "Установите matplotlib: pip install matplotlib")
            return
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Ошибка", "Сначала подключитесь к Arduino")
            return
        self.imu_graph_enabled = not self.imu_graph_enabled
        if self.imu_graph_enabled:
            if not self.graph_enabled:
                self.graph_enabled = True
                try:
                    self.serial_port.write(b"G\n")
                    self.serial_port.flush()
                    self._log(">>> G (график вкл для IMU)\n")
                except Exception:
                    pass
            self._open_imu_graph_window()
        else:
            self._close_imu_graph_window()

    def _open_imu_graph_window(self):
        """Открыть окно графика IMU."""
        if self.imu_graph_window and self.imu_graph_window.winfo_exists():
            self.imu_graph_window.lift()
            return
        self.imu_graph_data.clear()
        self.imu_graph_lines = None
        self.imu_graph_window = tk.Toplevel(self.root)
        self.imu_graph_window.title("График IMU")
        self.imu_graph_window.geometry("950x550")
        self.imu_graph_window.protocol("WM_DELETE_WINDOW", self._close_imu_graph_window)
        if HAS_MATPLOTLIB:
            fig = Figure(figsize=(9.5, 5), dpi=100)
            self.ax_imu1 = fig.add_subplot(211)
            self.ax_imu2 = fig.add_subplot(212)
            fig.tight_layout(pad=2.0)
            self.imu_canvas = FigureCanvasTkAgg(fig, master=self.imu_graph_window)
            self.imu_canvas.draw()
            self.imu_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
            hint = "Акселерометр (g): ax, ay, az  |  Гироскоп (°/s): gx, gy, gz"
            ttk.Label(self.imu_graph_window, text=hint, font=("", 9), foreground="gray").pack(anchor=tk.W, padx=8, pady=2)
        self._schedule_imu_graph_update()

    def _close_imu_graph_window(self):
        """Закрыть окно графика IMU."""
        self.imu_graph_enabled = False
        if self.imu_graph_window and self.imu_graph_window.winfo_exists():
            self.imu_graph_window.destroy()
        self.imu_graph_window = None
        self.imu_graph_lines = None

    def _schedule_imu_graph_update(self):
        """Запланировать обновление графика IMU."""
        try:
            if self.imu_graph_enabled and self.imu_graph_window and self.imu_graph_window.winfo_exists() and HAS_MATPLOTLIB:
                self._update_imu_graph()
                self.root.after(120, self._schedule_imu_graph_update)
        except tk.TclError:
            pass

    def _update_imu_graph(self):
        """Обновить график IMU."""
        if not self.imu_graph_data:
            return
        try:
            data = list(self.imu_graph_data)
            n = len(data)
            step = max(1, n // 250)
            data = data[::step]
            t = list(range(0, n, step))
            ax_vals = [d[0] for d in data]
            ay_vals = [d[1] for d in data]
            az_vals = [d[2] for d in data]
            gx_vals = [d[3] for d in data]
            gy_vals = [d[4] for d in data]
            gz_vals = [d[5] for d in data]

            if self.imu_graph_lines is None:
                self.ax_imu1.set_ylabel("g")
                self.ax_imu1.set_title("Акселерометр (ax, ay, az)")
                self.ax_imu1.grid(True, alpha=0.3)
                l1a, = self.ax_imu1.plot(t, ax_vals, "r-", label="ax", alpha=0.9)
                l1b, = self.ax_imu1.plot(t, ay_vals, "g-", label="ay", alpha=0.9)
                l1c, = self.ax_imu1.plot(t, az_vals, "b-", label="az", alpha=0.9)
                self.ax_imu1.legend(loc="upper right", fontsize=8)
                self.ax_imu2.set_ylabel("°/s")
                self.ax_imu2.set_xlabel("время (отсчёты)")
                self.ax_imu2.set_title("Гироскоп (gx, gy, gz)")
                self.ax_imu2.grid(True, alpha=0.3)
                l2a, = self.ax_imu2.plot(t, gx_vals, "r-", label="gx", alpha=0.9)
                l2b, = self.ax_imu2.plot(t, gy_vals, "g-", label="gy", alpha=0.9)
                l2c, = self.ax_imu2.plot(t, gz_vals, "b-", label="gz", alpha=0.9)
                self.ax_imu2.legend(loc="upper right", fontsize=8)
                self.imu_graph_lines = (l1a, l1b, l1c, l2a, l2b, l2c)
            else:
                l1a, l1b, l1c, l2a, l2b, l2c = self.imu_graph_lines
                l1a.set_data(t, ax_vals)
                l1b.set_data(t, ay_vals)
                l1c.set_data(t, az_vals)
                l2a.set_data(t, gx_vals)
                l2b.set_data(t, gy_vals)
                l2c.set_data(t, gz_vals)

            for ax in (self.ax_imu1, self.ax_imu2):
                ax.relim()
                ax.autoscale_view()
            self.imu_canvas.draw_idle()
        except Exception:
            pass

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

    def _read_loop_hz(self):
        """Запросить текущую частоту с Arduino (команда H без значения)."""
        if not self.serial_port or not self.serial_port.is_open:
            return
        self._suspend_telemetry_stream()
        try:
            self.serial_port.write(b"H\n")
            self.serial_port.flush()
            self._log(">>> H (запрос частоты)\n", "sent")
            line = self._serial_readline_blocking(0.65)
            if line:
                self._handle_firmware_text_line(line)
        except Exception:
            pass
        finally:
            self._resume_telemetry_stream()

    def _send_loop_hz(self):
        """Отправить H,hz — частота дискретизации цикла (25–200 Гц)."""
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Ошибка", "Сначала подключитесь к Arduino")
            return
        try:
            hz = int(self.loop_hz_var.get())
            if 25 <= hz <= 200:
                cmd = f"H,{hz}\n"
                self.serial_port.write(cmd.encode("utf-8"))
                self.serial_port.flush()
                self._log(f">>> {cmd.strip()} (частота дискретизации)\n", "sent")
            else:
                messagebox.showwarning("Ошибка", "Частота: 25–200 Гц")
        except ValueError:
            messagebox.showwarning("Ошибка", "Введите число (25–200)")

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
        """Добавить в лог. tag: 'sent' (бирюзовый) или 'recv' (голубой)."""
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, msg, tag)
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)

    def _clear_log(self):
        """Очистить консоль."""
        self.log_text.config(state=tk.NORMAL)
        self.log_text.delete(1.0, tk.END)
        self.log_text.config(state=tk.DISABLED)

    def _ensure_csv_telemetry_for_graphs(self):
        """Графики строятся из строк TLM (прошивка не шлёт поток по G — Flash). Включаем x,1 один раз при открытии графика."""
        if not self.serial_port or not self.serial_port.is_open:
            return
        try:
            self.serial_port.write(b"x,1\n")
            self.serial_port.flush()
            self._log(">>> x,1 (телеметрия TLM для графиков на ПК)\n", "sent")
        except Exception:
            pass

    def _append_popup_graph_from_tlm(self, sline: str):
        """Окно «График PID»: точки из TLM — target°, pitch°, err, P,I,D, motor."""
        if not self.graph_enabled:
            return
        gw = self.graph_window
        if gw is None:
            return
        try:
            if not gw.winfo_exists():
                return
        except tk.TclError:
            return
        parts = [p.strip() for p in sline.split(",")]
        if len(parts) < 14:
            return
        try:
            tup = (
                float(parts[4]),
                float(parts[2]),
                float(parts[10]),
                float(parts[11]),
                float(parts[12]),
                float(parts[13]),
                float(parts[5]),
            )
            self.graph_data.append(tup)
        except (ValueError, IndexError):
            pass

    def _handle_firmware_text_line(self, line: str) -> bool:
        """Текстовые ответы прошивки (до разбора CSV из 8+ чисел). Возвращает True, если строка обработана."""
        s = line.strip()
        if not s:
            return True
        fw = r"([\d.eE+\-]+)"
        # Частота: прошивка отвечает на H командой вида «100Hz» (без «Loop Hz:»)
        mhz = re.match(r"^(\d{1,3})Hz\s*$", s)
        if mhz and hasattr(self, "loop_hz_var"):
            self._log(f"<<< {line}\n", "recv")
            self._updating = True
            self.loop_hz_var.set(mhz.group(1))
            self._updating = False
            self.root.after(0, self._update_params_display)
            return True
        m = re.search(r"Angle PID:\s*Kp=\s*" + fw + r"\s+Ki=\s*" + fw + r"\s+Kd=\s*" + fw + r"\s+limit=\s*" + fw, s)
        if m:
            self._log(f"<<< {line}\n", "recv")
            self._updating = True
            self.kp_var.set(m.group(1))
            self.ki_var.set(m.group(2))
            self.kd_var.set(m.group(3))
            self.limit_var.set(m.group(4))
            self._updating = False
            self.root.after(0, self._sync_sliders_from_vars)
            return True
        m2 = re.search(r"Speed PID:\s*Kp=\s*" + fw + r"\s+Ki=\s*" + fw + r"\s+Kd=\s*" + fw + r"\s+limit=\s*" + fw, s)
        if m2:
            self._log(f"<<< {line}\n", "recv")
            self._updating = True
            kp, ki, kd, lim = m2.group(1), m2.group(2), m2.group(3), m2.group(4)
            self.spd_kp_var.set(kp)
            self.spd_ki_var.set(ki)
            self.spd_kd_var.set(kd)
            self.spd_limit_var.set(lim)
            self.spd_status_var.set(f"Speed PID: Kp={kp} Ki={ki} Kd={kd} L={lim}")
            self._updating = False
            self.root.after(0, self._sync_sliders_spd_from_vars)
            return True
        m3 = re.search(r"PID:\s*Kp=\s*" + fw + r"\s+Ki=\s*" + fw + r"\s+Kd=\s*" + fw + r"\s+limit=\s*" + fw, s)
        if m3:
            self._log(f"<<< {line}\n", "recv")
            self._updating = True
            self.kp_var.set(m3.group(1))
            self.ki_var.set(m3.group(2))
            self.kd_var.set(m3.group(3))
            self.limit_var.set(m3.group(4))
            self._updating = False
            self.root.after(0, self._sync_sliders_from_vars)
            return True
        if line.startswith("Lift: "):
            self._log(f"<<< {line}\n", "recv")
            try:
                rest = line[6:].strip()
                p = [x.strip() for x in rest.split(",")]
                if len(p) >= 3:
                    ae, db, rs = float(p[0]), float(p[1]), float(p[2])
                    self._updating = True
                    self.lift_angle_err_var.set(str(ae))
                    self.lift_debounce_var.set(str(int(db)))
                    self.lift_recovery_var.set(str(int(rs)))
                    self._updating = False
                    self.root.after(0, self._sync_lift_sliders_from_vars)
            except (ValueError, IndexError):
                pass
            return True
        m4 = re.search(r"STATUS\s+fall=(\d)\s+hz=(\d+)(?:\s+atz=(\d+))?", s)
        if m4:
            self._log(f"<<< {line}\n", "recv")
            prev_fall = self.is_fallen
            self.is_fallen = m4.group(1) == "1"
            if m4.group(3) is not None:
                try:
                    self.autotune_atz = int(m4.group(3))
                except ValueError:
                    pass
            if self.is_fallen and not prev_fall:
                try:
                    self.autotune.feed_fall_marker()
                except Exception:
                    pass
            if m4.group(2) and hasattr(self, "loop_hz_var"):
                self.loop_hz_var.set(m4.group(2))
            fs, hzv = m4.group(1), m4.group(2)
            azv = m4.group(3) if m4.group(3) is not None else "—"
            if hasattr(self, "robot_status_snapshot_var"):
                self.root.after(
                    0,
                    lambda f=fs, h=hzv, z=azv: self.robot_status_snapshot_var.set(
                        f"fall={f}  (1=упал)    hz={h} Гц    atz={z}  (режим автотюна: 0 выкл, 1 каскад, 2 только угол)"
                    ),
                )
            self.root.after(0, self._update_fall_indicator)
            return True
        m5 = re.search(r"Loop\s+Hz:\s*(\d+)", s)
        if m5 and hasattr(self, "loop_hz_var"):
            self._log(f"<<< {line}\n", "recv")
            self._updating = True
            self.loop_hz_var.set(m5.group(1))
            self._updating = False
            self.root.after(0, self._update_params_display)
            return True
        return False

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
                sline = line.strip()
                if sline.startswith("TLM_FALL"):
                    self.autotune.feed_fall_marker()
                    continue
                if sline.startswith("TLM,"):
                    self.autotune.feed_tlm_csv(sline)
                    snap = self._format_tlm_snapshot(sline)
                    self.root.after(0, lambda t=snap: self._set_telemetry_snap_ui(t))
                    self._append_pid_graph_from_tlm(sline)
                    self._append_popup_graph_from_tlm(sline)
                    continue
                if self._handle_firmware_text_line(line):
                    continue
                # Данные графика (устаревший CSV без префикса TLM): 7+ чисел через запятую
                parts = [p.strip() for p in line.split(",")]
                if len(parts) >= 8:
                    try:
                        vals = tuple(float(x) for x in parts)
                        if self.graph_enabled:
                            self.graph_data.append(vals[:7])  # График использует 7
                        if len(vals) >= 14 and (self.imu_graph_enabled or (self.imu_graph_window and self.imu_graph_window.winfo_exists())):
                            self.imu_graph_data.append((vals[8], vals[9], vals[10], vals[11], vals[12], vals[13]))
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
                    continue  # поток графика без префикса TLM — не дублировать в лог
                self._log(f"<<< {line}\n", "recv")
        except queue.Empty:
            pass
        self.root.after(100, self._process_read_queue)

    def _pid_graph_init_axes(self):
        """Пустые оси для вкладки графиков PID."""
        if not HAS_MATPLOTLIB or self._g_ax_a is None:
            return
        self._g_ax_a.clear()
        self._g_ax_s.clear()
        self._g_ax_a.set_title("Angle PID (ошибка °, P/I/D)")
        self._g_ax_a.set_ylabel("значение")
        self._g_ax_a.grid(True, alpha=0.35)
        self._g_ax_s.set_title("Speed PID (ошибка, P/I/D)")
        self._g_ax_s.set_xlabel("Номер образца")
        self._g_ax_s.set_ylabel("значение")
        self._g_ax_s.grid(True, alpha=0.35)

    def _append_pid_graph_from_tlm(self, sline: str):
        """Добавить точку из TLM в буферы графиков (индексы как в _format_tlm_snapshot)."""
        if not HAS_MATPLOTLIB or self._g_fig is None:
            return
        parts = [p.strip() for p in sline.split(",")]
        if len(parts) < 21 or parts[0] != "TLM":
            return
        try:
            self._graph_idx += 1
            self._graph_t.append(float(self._graph_idx))
            self._gs_e.append(float(parts[6]))
            self._gs_p.append(float(parts[7]))
            self._gs_i.append(float(parts[8]))
            self._gs_d.append(float(parts[9]))
            self._ga_e.append(float(parts[10]))
            self._ga_p.append(float(parts[11]))
            self._ga_i.append(float(parts[12]))
            self._ga_d.append(float(parts[13]))
        except (ValueError, IndexError):
            return
        self._request_graph_redraw()

    def _request_graph_redraw(self):
        if not HAS_MATPLOTLIB:
            return
        if self._graph_redraw_scheduled:
            return
        self._graph_redraw_scheduled = True
        self.root.after(120, self._do_redraw_pid_graphs)

    def _do_redraw_pid_graphs(self):
        self._graph_redraw_scheduled = False
        if not HAS_MATPLOTLIB or self._g_ax_a is None or self._g_canvas is None:
            return
        self._g_ax_a.clear()
        self._g_ax_s.clear()
        if len(self._graph_t) == 0:
            self._pid_graph_init_axes()
            try:
                self._g_fig.tight_layout()
            except Exception:
                pass
            self._g_canvas.draw_idle()
            return
        t = list(self._graph_t)
        self._g_ax_a.plot(t, list(self._ga_e), label="err", linewidth=1.1, color="#1f77b4")
        self._g_ax_a.plot(t, list(self._ga_p), label="P", linewidth=0.85, alpha=0.9)
        self._g_ax_a.plot(t, list(self._ga_i), label="I", linewidth=0.85, alpha=0.9)
        self._g_ax_a.plot(t, list(self._ga_d), label="D", linewidth=0.85, alpha=0.9)
        self._g_ax_a.set_title("Angle PID")
        self._g_ax_a.set_ylabel("° / усл.")
        self._g_ax_a.grid(True, alpha=0.35)
        self._g_ax_a.legend(loc="upper right", fontsize=8)

        self._g_ax_s.plot(t, list(self._gs_e), label="err", linewidth=1.1, color="#1f77b4")
        self._g_ax_s.plot(t, list(self._gs_p), label="P", linewidth=0.85, alpha=0.9)
        self._g_ax_s.plot(t, list(self._gs_i), label="I", linewidth=0.85, alpha=0.9)
        self._g_ax_s.plot(t, list(self._gs_d), label="D", linewidth=0.85, alpha=0.9)
        self._g_ax_s.set_title("Speed PID")
        self._g_ax_s.set_xlabel("Номер образца")
        self._g_ax_s.set_ylabel("усл.")
        self._g_ax_s.grid(True, alpha=0.35)
        self._g_ax_s.legend(loc="upper right", fontsize=8)
        try:
            self._g_fig.tight_layout()
        except Exception:
            pass
        self._g_canvas.draw_idle()

    def _clear_pid_graphs(self):
        """Сбросить буферы и перерисовать вкладку графиков."""
        self._graph_idx = 0
        self._graph_t.clear()
        self._ga_e.clear()
        self._ga_p.clear()
        self._ga_i.clear()
        self._ga_d.clear()
        self._gs_e.clear()
        self._gs_p.clear()
        self._gs_i.clear()
        self._gs_d.clear()
        self._graph_redraw_scheduled = False
        self._do_redraw_pid_graphs()

    def _append_autotune_log(self, widget: scrolledtext.ScrolledText, msg: str) -> None:
        widget.config(state=tk.NORMAL)
        widget.insert(tk.END, msg)
        widget.see(tk.END)
        widget.config(state=tk.DISABLED)

    def _log_autotune(self, msg: str, channel=None) -> None:
        """Журналы sweep: angle | speed | both."""
        if channel == "both":
            self._append_autotune_log(self.at_log_angle, msg)
            self._append_autotune_log(self.at_log_speed, msg)
            return
        w = self.at_log_speed if channel == "speed" else self.at_log_angle
        self._append_autotune_log(w, msg)

    def _clear_autotune_log(self, which: str) -> None:
        if which in ("angle", "both"):
            self.at_log_angle.config(state=tk.NORMAL)
            self.at_log_angle.delete("1.0", tk.END)
            self.at_log_angle.config(state=tk.DISABLED)
        if which in ("speed", "both"):
            self.at_log_speed.config(state=tk.NORMAL)
            self.at_log_speed.delete("1.0", tk.END)
            self.at_log_speed.config(state=tk.DISABLED)

    def _update_autotune_live_panel(self) -> None:
        """Две панели: что сейчас угол/скорость sweep на роботе (ОЗУ), без EEPROM."""
        at = self.autotune
        idle_a = (
            "Angle PID: sweep не запущен.\n"
            "Выберите диапазон Kp и «Старт» — здесь появятся Kp, Ki, Kd, L и пересечения ошибки angE."
        )
        idle_s = (
            "Speed PID: sweep не запущен.\n"
            "Режим «Оба подряд»: сначала журнал и панель Angle, затем — Speed справа."
        )
        if at.state == AutotuneState.IDLE:
            self.at_angle_live_var.set(idle_a)
            self.at_speed_live_var.set(idle_s)
            return
        if at.state == AutotuneState.PAUSED_FALL:
            self.at_angle_live_var.set(
                "ПАУЗА: fall (робот упал).\nПоднимите вертикально → «Продолжить после падения»."
            )
            self.at_speed_live_var.set("ПАУЗА: fall — общая для Angle и Speed.")
            return

        p = at.params
        if at.pid_role == PidRole.ANGLE:
            mode = "a,2 только внутренний контур" if at.angle_only_for_angle_phase else "a,1 каскад Speed→Angle"
            self.at_angle_live_var.set(
                f"АКТИВЕН sweep Angle PID\n"
                f"Режим прошивки: {mode}\n"
                f"На роботе сейчас: Kp={at.current_kp:g}  Ki={p.ki:g}  Kd={p.kd:g}  L={p.limit:g}\n"
                f"Пересечений ошибки angE: {at.crossings} / {p.max_crossings}\n"
                f"Диапазон Kp: {p.kp_start:g} … {p.kp_end:g}, шаг {p.kp_step:g}, {p.step_duration_s:g} с на шаг\n"
                f"Подсказка: при откате смотрите последнюю стабильную строку «на роботе» в журнале слева."
            )
            if at.plan == SweepPlan.BOTH:
                self.at_speed_live_var.set(
                    "Ожидание: после конца диапазона Angle автоматически начнётся Speed PID (правый журнал)."
                )
            else:
                self.at_speed_live_var.set("Режим только Angle — Speed sweep не выполняется.")
            return

        ang_hdr = ""
        if at.angle_phase_saved_kp is not None:
            ang_hdr = (
                f"После фазы Angle (ориентир Kp≈{at.angle_phase_saved_kp:g}). Детали — левый журнал.\n\n"
            )
        elif at.plan == SweepPlan.SPEED_ONLY:
            ang_hdr = "Режим только Speed — левый журнал Angle не используется.\n\n"
        self.at_speed_live_var.set(
            f"{ang_hdr}"
            f"АКТИВЕН sweep Speed PID (каскад a,1)\n"
            f"На роботе сейчас: Kp={at.current_kp:g}  Ki={p.ki:g}  Kd={p.kd:g}  L={p.limit:g}\n"
            f"Пересечений ошибки spdE: {at.crossings} / {p.max_crossings}\n"
            f"Диапазон Kp: {p.kp_start:g} … {p.kp_end:g}, шаг {p.kp_step:g}, {p.step_duration_s:g} с на шаг"
        )
        if at.plan == SweepPlan.BOTH and at.angle_phase_saved_kp is not None:
            self.at_angle_live_var.set(
                f"Фаза Angle завершена.\nИтог sweep угла (см. последние «на роботе» в левом журнале): ≈{at.angle_phase_saved_kp:g}"
            )
        elif at.plan == SweepPlan.SPEED_ONLY:
            self.at_angle_live_var.set("Sweep только Speed — настройка угла не менялась в этом запуске.")
        else:
            self.at_angle_live_var.set(idle_a)

    def _autotune_start(self):
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Ошибка", "Сначала подключитесь к порту")
            return
        key = self.at_plan_var.get()
        plan_map = {"angle": SweepPlan.ANGLE_ONLY, "speed": SweepPlan.SPEED_ONLY, "both": SweepPlan.BOTH}
        plan = plan_map.get(key, SweepPlan.ANGLE_ONLY)

        try:
            step_s = float(self.at_step_s_var.get())
            cross = int(self.at_cross_var.get())
        except (ValueError, TypeError):
            messagebox.showwarning("Ошибка", "Проверьте сек/шаг и max×")
            return
        if step_s <= 0:
            messagebox.showwarning("Ошибка", "сек/шаг должно быть > 0")
            return

        angle_only = self.at_angle_only_var.get()
        dummy_angle = SweepParams()

        if plan == SweepPlan.SPEED_ONLY:
            vals_s = self._get_speed_pid_values()
            if vals_s is None:
                messagebox.showwarning("Ошибка", "Некорректные коэффициенты Speed PID в полях")
                return
            _, ski, skd, slim = vals_s
            try:
                speed_sp = SweepParams(
                    kp_start=float(self.at_sp_kp0_var.get()),
                    kp_end=float(self.at_sp_kp1_var.get()),
                    kp_step=float(self.at_sp_kps_var.get()),
                    ki=ski,
                    kd=skd,
                    limit=slim,
                    step_duration_s=step_s,
                    max_crossings=cross,
                )
            except (ValueError, TypeError):
                messagebox.showwarning("Ошибка", "Проверьте числовые поля Speed sweep")
                return
            if speed_sp.kp_step <= 0:
                messagebox.showwarning("Ошибка", "Шаг Kp Speed должен быть > 0")
                return
            if self.autotune.start(plan, False, dummy_angle, speed_sp):
                self.at_status_var.set("Автотюн: Speed sweep…")
                self._clear_autotune_log("both")
                self._update_autotune_live_panel()
            return

        vals_a = self._get_pid_values()
        if vals_a is None:
            messagebox.showwarning("Ошибка", "Некорректные коэффициенты Angle PID в полях")
            return
        try:
            _, ki, kd, lim = vals_a
            angle_sp = SweepParams(
                kp_start=float(self.at_kp0_var.get()),
                kp_end=float(self.at_kp1_var.get()),
                kp_step=float(self.at_kps_var.get()),
                ki=ki,
                kd=kd,
                limit=lim,
                step_duration_s=step_s,
                max_crossings=cross,
            )
        except (ValueError, TypeError):
            messagebox.showwarning("Ошибка", "Проверьте числовые поля Angle sweep")
            return
        if angle_sp.kp_step <= 0:
            messagebox.showwarning("Ошибка", "Шаг Kp Angle должен быть > 0")
            return

        speed_sp = None
        if plan == SweepPlan.BOTH:
            vals_s = self._get_speed_pid_values()
            if vals_s is None:
                messagebox.showwarning("Ошибка", "Некорректные коэффициенты Speed PID для второй фазы")
                return
            _, ski, skd, slim = vals_s
            try:
                speed_sp = SweepParams(
                    kp_start=float(self.at_sp_kp0_var.get()),
                    kp_end=float(self.at_sp_kp1_var.get()),
                    kp_step=float(self.at_sp_kps_var.get()),
                    ki=ski,
                    kd=skd,
                    limit=slim,
                    step_duration_s=step_s,
                    max_crossings=cross,
                )
            except (ValueError, TypeError):
                messagebox.showwarning("Ошибка", "Проверьте числовые поля Speed sweep")
                return
            if speed_sp.kp_step <= 0:
                messagebox.showwarning("Ошибка", "Шаг Kp Speed должен быть > 0")
                return

        if self.autotune.start(plan, angle_only, angle_sp, speed_sp):
            self.at_status_var.set("Автотюн: sweep выполняется…")
            self._clear_autotune_log("both")
            self._update_autotune_live_panel()

    def _autotune_stop(self):
        self.autotune.stop()
        self.at_status_var.set("Автотюн: выкл")
        self._update_autotune_live_panel()

    def _autotune_resume(self):
        self.autotune.resume_after_fall()
        self.at_status_var.set("Автотюн: продолжение…")
        self._update_autotune_live_panel()

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
