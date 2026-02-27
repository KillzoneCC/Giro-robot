#!/usr/bin/env python3
"""
PID Tuner — приложение для управления PID-регулятором Giro-Robot
Подключение к Arduino через USB (Serial)
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import serial
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

# Диапазоны для ползунков (макс. 10000 у всех)
KP_RANGE = (0, 10000)
KI_RANGE = (0, 10000)
KD_RANGE = (0, 10000)
LIMIT_RANGE = (100, 20000)


class PidTunerApp:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Giro-Robot PID Tuner")
        self.root.geometry("560x520")
        self.root.resizable(True, True)

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

        self._build_ui()
        self._start_read_thread()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        main = ttk.Frame(self.root, padding=10)
        main.pack(fill=tk.BOTH, expand=True)

        # === Порт ===
        port_frame = ttk.LabelFrame(main, text="Подключение", padding=5)
        port_frame.pack(fill=tk.X, pady=(0, 5))

        row1 = ttk.Frame(port_frame)
        row1.pack(fill=tk.X)
        ttk.Label(row1, text="COM-порт:").pack(side=tk.LEFT, padx=(0, 5))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(row1, textvariable=self.port_var, width=25, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=(0, 5))
        self.refresh_btn = ttk.Button(row1, text="Обновить", command=self._refresh_ports)
        self.refresh_btn.pack(side=tk.LEFT, padx=(0, 5))
        self.connect_btn = ttk.Button(row1, text="Подключить", command=self._toggle_connect)
        self.connect_btn.pack(side=tk.LEFT)
        self.status_label = ttk.Label(row1, text="Отключено", foreground="gray")
        self.status_label.pack(side=tk.LEFT, padx=(15, 0))

        # === PID параметры (ползунки + поля ввода) ===
        pid_frame = ttk.LabelFrame(main, text="PID параметры", padding=5)
        pid_frame.pack(fill=tk.X, pady=(0, 5))

        self.kp_var = tk.StringVar(value=str(DEFAULT_KP))
        self.ki_var = tk.StringVar(value=str(DEFAULT_KI))
        self.kd_var = tk.StringVar(value=str(DEFAULT_KD))
        self.limit_var = tk.StringVar(value=str(DEFAULT_LIMIT))

        def add_param_row(parent, label, str_var, slider_range, resolution=0.1):
            row = ttk.Frame(parent)
            row.pack(fill=tk.X, pady=4)
            ttk.Label(row, text=label, width=6).pack(side=tk.LEFT, padx=(0, 5))
            slider = tk.Scale(row, from_=slider_range[0], to=slider_range[1], resolution=resolution,
                             orient=tk.HORIZONTAL, length=200, showvalue=0,
                             command=lambda v, sv=str_var: self._slider_changed(sv, v))
            slider.pack(side=tk.LEFT, padx=(0, 8))
            entry = ttk.Entry(row, textvariable=str_var, width=12)
            entry.pack(side=tk.LEFT)
            return slider

        self.kp_slider = add_param_row(pid_frame, "Kp:", self.kp_var, KP_RANGE, 1.0)
        self.ki_slider = add_param_row(pid_frame, "Ki:", self.ki_var, KI_RANGE, 0.01)
        self.kd_slider = add_param_row(pid_frame, "Kd:", self.kd_var, KD_RANGE, 0.01)
        self.limit_slider = add_param_row(pid_frame, "Limit:", self.limit_var, LIMIT_RANGE, 100.0)

        # Z-коррекция (только ручной ввод)
        z_row = ttk.Frame(pid_frame)
        z_row.pack(fill=tk.X, pady=4)
        ttk.Label(z_row, text="Z:", width=6).pack(side=tk.LEFT, padx=(0, 5))
        self.z_var = tk.StringVar(value="0")
        ttk.Entry(z_row, textvariable=self.z_var, width=12).pack(side=tk.LEFT, padx=(0, 5))
        ttk.Button(z_row, text="Сохранить Z в EEPROM", command=self._save_z_to_eeprom).pack(side=tk.LEFT)
        ttk.Label(z_row, text="(коррекция нуля, градусы)", foreground="gray").pack(side=tk.LEFT, padx=(8, 0))

        self._sync_sliders_from_vars()

        # Привязка: при изменении поля ввода — обновить ползунок и отправить
        for var in (self.kp_var, self.ki_var, self.kd_var, self.limit_var):
            var.trace_add("write", lambda *a: self._on_param_changed())

        # === Кнопки ===
        btn_frame = ttk.Frame(main)
        btn_frame.pack(fill=tk.X, pady=5)

        self.read_btn = ttk.Button(btn_frame, text="Прочитать с Arduino", command=self._read_from_arduino)
        self.read_btn.pack(side=tk.LEFT, padx=(0, 5))
        self.save_btn = ttk.Button(btn_frame, text="Сохранить в EEPROM", command=self._save_to_eeprom)
        self.save_btn.pack(side=tk.LEFT, padx=(0, 5))
        self.graph_btn = ttk.Button(btn_frame, text="График PID", command=self._toggle_graph)
        self.graph_btn.pack(side=tk.LEFT, padx=(0, 5))
        ttk.Label(btn_frame, text="(изменения отправляются автоматически)", foreground="gray").pack(side=tk.LEFT, padx=(15, 0))

        # === Лог ===
        log_frame = ttk.LabelFrame(main, text="Лог Arduino", padding=5)
        log_frame.pack(fill=tk.BOTH, expand=True, pady=(5, 0))

        self.log_text = scrolledtext.ScrolledText(log_frame, height=10, state=tk.DISABLED, wrap=tk.WORD)
        self.log_text.pack(fill=tk.BOTH, expand=True)

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

    def _sync_sliders_from_vars(self):
        """Синхронизировать ползунки с текущими значениями в полях."""
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
        self._close_graph_window()
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
        """Автоматическая отправка при изменении (без диалогов)."""
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
            self._log(f">>> {cmd.strip()}\n")
        except Exception as e:
            self._log(f"Ошибка отправки: {e}\n")

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
            self._log(">>> P (запрос PID)\n")
        except Exception as e:
            self._log(f"Ошибка: {e}\n")

    def _save_to_eeprom(self):
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Ошибка", "Сначала подключитесь к Arduino")
            return
        try:
            self.serial_port.write(b"w\n")
            self.serial_port.flush()
            self._log(">>> w (сохранить в EEPROM)\n")
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
            self._log(f">>> {cmd.strip()} (сохранение Z в EEPROM)\n")
        except ValueError:
            messagebox.showwarning("Ошибка", "Введите число для Z-коррекции")
        except Exception as e:
            self._log(f"Ошибка: {e}\n")

    def _log(self, msg):
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, msg)
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
                # Данные графика: target,angle,error,P,I,D,output (7 чисел)
                parts = [p.strip() for p in line.split(",")]
                if len(parts) == 7:
                    try:
                        vals = tuple(float(x) for x in parts)
                        if self.graph_enabled:
                            self.graph_data.append(vals)
                    except ValueError:
                        pass
                    continue  # Не логируем поток графика
                self._log(f"<<< {line}\n")
                # Парсим ответ "PID: Kp=... Ki=... Kd=... limit=..."
                m = re.search(r"Kp=([\d.]+)\s+Ki=([\d.]+)\s+Kd=([\d.]+)\s+limit=([\d.]+)", line)
                if m:
                    self._updating = True
                    self.kp_var.set(m.group(1))
                    self.ki_var.set(m.group(2))
                    self.kd_var.set(m.group(3))
                    self.limit_var.set(m.group(4))
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
    app = PidTunerApp()
    app.run()
