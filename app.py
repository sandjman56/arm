# app.py -- Main ArmUI orchestrator
import tkinter as tk
from tkinter import ttk
import threading
import queue
import os
import json
from collections import deque

from theme import (
    BG_PRIMARY, BG_PANEL, BG_INPUT, BG_HOVER, BG_BUTTON,
    ACCENT_CYAN, ACCENT_CYAN_DIM, ACCENT_GREEN, ACCENT_RED, ACCENT_ORANGE,
    TEXT_PRIMARY, TEXT_SECONDARY, TEXT_ACCENT,
    MODULE_COLORS,
    FONT_TITLE, FONT_HEADING, FONT_BODY, FONT_BODY_BOLD, FONT_BUTTON,
    FONT_DATA, FONT_DATA_LARGE, FONT_LABEL,
    apply_theme,
)
from widgets import (
    AccentButton, StatusDot, StepperPositionBar, PressureBox,
)
from arduino_interface import ArduinoInterface
from logger import CSVLogger
from graph import PressureGraph
from panels import BurstPanel, PIDPanel, CalibrationPanel, ExperimentPanel


class ArmUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Continuum Arm Controller")
        self.root.geometry("1100x950")

        apply_theme(root)

        self.arduino = ArduinoInterface()
        self.queue = queue.Queue()
        self.running = True
        self.logger = CSVLogger()

        self.status = tk.StringVar(value="SELECT PORT AND CONNECT")
        self.connection = tk.StringVar(value="Not connected")
        self.port_var = tk.StringVar()
        self.ports = []

        # Mode state
        self.mode = tk.StringVar(value="burst")
        self.pid_active = False
        self.pid_setpoint_psi = 0.0

        # Graph
        self.graph_max_points = 200

        # Calibration state
        # Per-module step-position limits, keyed by 1-indexed module_id:
        #   self.cal_limits[mid] = {"back": int, "front": int}
        self.step_position = 0  # legacy: last-seen position from P, telemetry
        self.cal_limits = {}
        self.cal_file = os.path.join(os.path.dirname(__file__) or ".", "calibration.json")
        self.calibration_loaded = False
        self._load_calibration_from_disk()

        # IMU state
        self.imu_data = {
            "ax": tk.StringVar(value="---"),
            "ay": tk.StringVar(value="---"),
            "az": tk.StringVar(value="---"),
            "gx": tk.StringVar(value="---"),
            "gy": tk.StringVar(value="---"),
            "gz": tk.StringVar(value="---"),
            "temp": tk.StringVar(value="---"),
        }

        # Module system — configurable list of modules. Pin mapping reflects
        # current wiring; modules beyond wired ones get None pins.
        self.modules = []
        self.next_module_id = 1
        MODULE_CONFIG = [
            ("Module 1", 3, 4),
            ("Module 2", 8, 7),
            ("Module 3", 30, 28),
            ("Module 4", None, None),
            ("Module 5", None, None),
            ("Module 6", None, None),
        ]
        for name, step_pin, dir_pin in MODULE_CONFIG:
            self._add_module_data(name, step_pin=step_pin, dir_pin=dir_pin)

        self.build_ui()
        self.refresh_ports()

        self.root.after(100, self.update_loop)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _add_module_data(self, name, step_pin, dir_pin):
        idx = len(self.modules)
        color = MODULE_COLORS[idx % len(MODULE_COLORS)]
        mod = {
            "id": self.next_module_id,
            "name": name,
            "step_pin": step_pin,
            "dir_pin": dir_pin,
            "pressure_hpa": tk.StringVar(value="---"),
            "pressure_psi": tk.StringVar(value="---"),
            "psi_history": deque(maxlen=self.graph_max_points),
            "color": color,
            "step_position": tk.IntVar(value=0),
        }
        self.modules.append(mod)
        self.next_module_id += 1
        return mod

    # ===========================
    # UI BUILD
    # ===========================
    def build_ui(self):
        # Scrollable content wrapper: everything below is packed into
        # self.content_frame, which lives inside a canvas with a vertical
        # scrollbar. This means the window can be any height and the user
        # can still scroll to see everything.
        self._scroll_canvas = tk.Canvas(
            self.root, bg=BG_PRIMARY, highlightthickness=0,
        )
        self._scroll_bar = ttk.Scrollbar(
            self.root, orient="vertical", command=self._scroll_canvas.yview,
        )
        self._scroll_canvas.configure(yscrollcommand=self._scroll_bar.set)
        self._scroll_bar.pack(side="right", fill="y")
        self._scroll_canvas.pack(side="left", fill="both", expand=True)

        self.content_frame = tk.Frame(self._scroll_canvas, bg=BG_PRIMARY)
        self._content_window = self._scroll_canvas.create_window(
            (0, 0), window=self.content_frame, anchor="nw",
        )

        # Keep the scroll region in sync with the content frame's size.
        def _on_content_resize(event):
            self._scroll_canvas.configure(scrollregion=self._scroll_canvas.bbox("all"))
        self.content_frame.bind("<Configure>", _on_content_resize)

        # Make the inner frame always match the canvas width so children
        # that pack fill='x' actually fill horizontally.
        def _on_canvas_resize(event):
            self._scroll_canvas.itemconfigure(self._content_window, width=event.width)
        self._scroll_canvas.bind("<Configure>", _on_canvas_resize)

        # Mouse wheel / trackpad scroll. macOS sends small delta values;
        # Linux uses Button-4/5. Windows sends delta in multiples of 120.
        def _on_mousewheel(event):
            # Normalize: macOS delta ~= 1 per notch, Windows delta ~= 120.
            if abs(event.delta) >= 120:
                step = int(-event.delta / 120)
            else:
                step = -event.delta
            self._scroll_canvas.yview_scroll(step, "units")
        self._scroll_canvas.bind_all("<MouseWheel>", _on_mousewheel)
        self._scroll_canvas.bind_all("<Button-4>",
                                     lambda e: self._scroll_canvas.yview_scroll(-1, "units"))
        self._scroll_canvas.bind_all("<Button-5>",
                                     lambda e: self._scroll_canvas.yview_scroll(1, "units"))

        # Title bar
        title_frame = tk.Frame(self.content_frame, bg=BG_PRIMARY)
        title_frame.pack(fill="x", padx=20, pady=(10, 0))

        tk.Label(title_frame, text="CONTINUUM ARM CONTROLLER",
                font=FONT_TITLE, fg=ACCENT_CYAN, bg=BG_PRIMARY).pack(side="left")

        # Thin accent line under title
        tk.Frame(self.content_frame, bg=ACCENT_CYAN_DIM, height=1).pack(fill="x", padx=20, pady=(5, 0))

        # Connection frame
        conn_frame = tk.Frame(self.content_frame, bg=BG_PANEL)
        conn_frame.pack(fill="x", padx=20, pady=(8, 0))

        conn_inner = tk.Frame(conn_frame, bg=BG_PANEL)
        conn_inner.pack(padx=10, pady=8)

        tk.Label(conn_inner, text="PORT:", font=FONT_BODY,
                fg=TEXT_SECONDARY, bg=BG_PANEL).grid(row=0, column=0, padx=(0, 5))

        self.port_dropdown = ttk.Combobox(conn_inner, textvariable=self.port_var, width=30)
        self.port_dropdown.grid(row=0, column=1, padx=5)

        refresh_btn = AccentButton(conn_inner, text="Refresh",
                                   command=self.refresh_ports)
        refresh_btn.grid(row=0, column=2, padx=5)

        connect_btn = AccentButton(conn_inner, text="Connect", accent=ACCENT_GREEN,
                                   command=self.connect_selected)
        connect_btn.grid(row=0, column=3, padx=5)

        # Status row
        status_row = tk.Frame(conn_frame, bg=BG_PANEL)
        status_row.pack(padx=10, pady=(0, 8))

        self._conn_dot = StatusDot(status_row, color=ACCENT_RED, size=8)
        self._conn_dot.pack(side="left", padx=(0, 5))
        # Override StatusDot background for panel
        self._conn_dot.configure(bg=BG_PANEL)
        self._conn_dot._canvas.configure(bg=BG_PANEL)

        tk.Label(status_row, textvariable=self.connection,
                font=FONT_BODY, fg=TEXT_SECONDARY, bg=BG_PANEL).pack(side="left", padx=(0, 20))

        tk.Label(status_row, text="STATUS:", font=FONT_BODY,
                fg=TEXT_SECONDARY, bg=BG_PANEL).pack(side="left")
        tk.Label(status_row, textvariable=self.status,
                font=FONT_BODY, fg=TEXT_PRIMARY, bg=BG_PANEL).pack(side="left", padx=5)

        # Stepper Position Bar (NEW)
        stepper_border = tk.Frame(self.content_frame, bg=ACCENT_CYAN_DIM, padx=1, pady=1)
        stepper_border.pack(fill="x", padx=20, pady=(8, 0))
        self.stepper_bar = StepperPositionBar(stepper_border)
        self.stepper_bar.pack(fill="x")
        self.stepper_bar.rebuild(self.modules)

        # Pressure display
        pressure_outer = tk.Frame(self.content_frame, bg=BG_PRIMARY)
        pressure_outer.pack(fill="x", padx=20, pady=(8, 0))

        tk.Frame(pressure_outer, bg=ACCENT_CYAN_DIM, height=1).pack(fill="x")
        tk.Label(pressure_outer, text="LIVE PRESSURE", font=FONT_LABEL,
                fg=ACCENT_CYAN_DIM, bg=BG_PRIMARY).pack(anchor="w", padx=5, pady=(3, 0))

        self.pressure_frame = tk.Frame(pressure_outer, bg=BG_PRIMARY)
        self.pressure_frame.pack(fill="x", padx=5, pady=5)
        self._rebuild_pressure_display()

        # IMU display
        imu_outer = tk.Frame(self.content_frame, bg=BG_PRIMARY)
        imu_outer.pack(fill="x", padx=20, pady=(8, 0))

        tk.Frame(imu_outer, bg=ACCENT_CYAN_DIM, height=1).pack(fill="x")
        tk.Label(imu_outer, text="IMU (MPU-6500)", font=FONT_LABEL,
                fg=ACCENT_CYAN_DIM, bg=BG_PRIMARY).pack(anchor="w", padx=5, pady=(3, 0))

        imu_frame = tk.Frame(imu_outer, bg=BG_PANEL)
        imu_frame.pack(fill="x", padx=5, pady=5)

        imu_row1 = tk.Frame(imu_frame, bg=BG_PANEL)
        imu_row1.pack(fill="x", padx=10, pady=(5, 2))
        tk.Label(imu_row1, text="ACCEL", font=FONT_BODY_BOLD,
                fg=ACCENT_CYAN, bg=BG_PANEL, width=6).pack(side="left")
        for axis in ("ax", "ay", "az"):
            lbl = axis.upper().replace("A", "")
            tk.Label(imu_row1, text=f"{lbl}:", font=FONT_BODY,
                    fg=TEXT_SECONDARY, bg=BG_PANEL).pack(side="left", padx=(10, 0))
            tk.Label(imu_row1, textvariable=self.imu_data[axis],
                    font=FONT_DATA, fg=TEXT_PRIMARY, bg=BG_PANEL,
                    width=8).pack(side="left")

        imu_row2 = tk.Frame(imu_frame, bg=BG_PANEL)
        imu_row2.pack(fill="x", padx=10, pady=(2, 2))
        tk.Label(imu_row2, text="GYRO", font=FONT_BODY_BOLD,
                fg=ACCENT_ORANGE, bg=BG_PANEL, width=6).pack(side="left")
        for axis in ("gx", "gy", "gz"):
            lbl = axis.upper().replace("G", "")
            tk.Label(imu_row2, text=f"{lbl}:", font=FONT_BODY,
                    fg=TEXT_SECONDARY, bg=BG_PANEL).pack(side="left", padx=(10, 0))
            tk.Label(imu_row2, textvariable=self.imu_data[axis],
                    font=FONT_DATA, fg=TEXT_PRIMARY, bg=BG_PANEL,
                    width=8).pack(side="left")

        imu_row3 = tk.Frame(imu_frame, bg=BG_PANEL)
        imu_row3.pack(fill="x", padx=10, pady=(2, 5))
        tk.Label(imu_row3, text="TEMP", font=FONT_BODY_BOLD,
                fg=TEXT_SECONDARY, bg=BG_PANEL, width=6).pack(side="left")
        tk.Label(imu_row3, textvariable=self.imu_data["temp"],
                font=FONT_DATA, fg=TEXT_PRIMARY, bg=BG_PANEL,
                width=8).pack(side="left", padx=(10, 0))
        tk.Label(imu_row3, text="\u00b0C", font=FONT_BODY,
                fg=TEXT_SECONDARY, bg=BG_PANEL).pack(side="left")

        # Mode selector
        mode_frame = tk.Frame(self.content_frame, bg=BG_PRIMARY)
        mode_frame.pack(fill="x", padx=20, pady=(10, 0))

        tk.Label(mode_frame, text="MODE:", font=FONT_BODY_BOLD,
                fg=ACCENT_CYAN, bg=BG_PRIMARY).pack(side="left", padx=(0, 10))

        for text, val in [("Burst Control", "burst"), ("PID Control", "pid"),
                          ("Calibration", "calibration"), ("Experiments", "experiments")]:
            rb = ttk.Radiobutton(mode_frame, text=text, variable=self.mode,
                                 value=val, command=self.switch_mode)
            rb.pack(side="left", padx=10)

        # Panel container
        self.panel_container = tk.Frame(self.content_frame, bg=BG_PRIMARY)
        self.panel_container.pack(fill="both", expand=True, padx=20, pady=5)

        # Create panels
        self.burst_panel = BurstPanel(
            self.panel_container, self.modules, self.send,
            self.emergency_stop)

        self.pid_panel = PIDPanel(
            self.panel_container, self.send,
            self.start_pid, self.stop_pid, self.emergency_stop)

        self.cal_panel = CalibrationPanel(
            self.panel_container,
            modules=self._wired_modules(),
            on_send=self.send,
            on_set_back=self.cal_set_back,
            on_set_front=self.cal_set_front,
            on_save=self.cal_save,
            on_clear=self.cal_clear,
            on_module_change=self._cal_on_module_change,
            on_emergency_stop=self.emergency_stop,
        )

        # Experiments mode: backend + controller + panel.
        from length_calibration import load_or_default
        from experiment_backend import SimBackend
        from experiment_controller import ExperimentController

        self.length_cal_path = os.path.join(os.path.dirname(__file__) or ".", "length_calibration.json")
        self.length_cal = load_or_default(self.length_cal_path)
        # Start with a SimBackend; _refresh_experiment_backend() swaps to
        # LiveBackend whenever serial is connected.
        self.experiment_backend = SimBackend(
            L_rest=self.length_cal.L_rest, num_modules=len(self.modules),
        )
        self.experiment_controller = ExperimentController(
            backend=self.experiment_backend, calibration=self.length_cal,
        )
        self.experiment_panel = ExperimentPanel(
            self.panel_container,
            on_start_zero=self._exp_start_zero,
            on_confirm_zero=self._exp_confirm_zero,
            on_rezero=self._exp_rezero,
            on_recalibrate_length=self._exp_recalibrate_length,
            on_reach=self._exp_reach,
            on_emergency_stop=self._exp_emergency_stop,
        )
        # Rolling yaw history for drift estimation.
        self._yaw_history = deque(maxlen=600)  # ~60s at 10Hz

        # Show default panel
        self.burst_panel.pack(fill="both", expand=True)

        # Logging
        log_frame = tk.Frame(self.content_frame, bg=BG_PRIMARY)
        log_frame.pack(fill="x", padx=20, pady=(5, 0))

        tk.Frame(log_frame, bg=ACCENT_CYAN_DIM, height=1).pack(fill="x")
        tk.Label(log_frame, text="LOGGING", font=FONT_LABEL,
                fg=ACCENT_CYAN_DIM, bg=BG_PRIMARY).pack(anchor="w", padx=5, pady=(3, 0))

        log_btns = tk.Frame(log_frame, bg=BG_PRIMARY)
        log_btns.pack(pady=5)
        AccentButton(log_btns, text="Start Logging", accent=ACCENT_GREEN,
                    command=self.start_log).pack(side="left", padx=5)
        AccentButton(log_btns, text="Stop Logging",
                    command=self.stop_log).pack(side="left", padx=5)

        # Pressure graph
        graph_outer = tk.Frame(self.content_frame, bg=BG_PRIMARY)
        graph_outer.pack(fill="both", expand=True, padx=20, pady=(5, 10))

        tk.Frame(graph_outer, bg=ACCENT_CYAN_DIM, height=1).pack(fill="x")
        tk.Label(graph_outer, text="PRESSURE (PSI)", font=FONT_LABEL,
                fg=ACCENT_CYAN_DIM, bg=BG_PRIMARY).pack(anchor="w", padx=5, pady=(3, 0))

        self.graph = PressureGraph(graph_outer)
        self.graph.pack(fill="both", expand=True, padx=5, pady=5)

    # ===========================
    # DYNAMIC UI
    # ===========================
    def _rebuild_pressure_display(self):
        for w in self.pressure_frame.winfo_children():
            w.destroy()
        for i, mod in enumerate(self.modules):
            PressureBox(self.pressure_frame, mod).grid(row=0, column=i, padx=6, pady=3)

    # ===========================
    # PORT HANDLING
    # ===========================
    def refresh_ports(self):
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        self.ports = [
            p.device for p in ports
            if "usb" in p.device.lower() or "tty." in p.device.lower() or p.device.upper().startswith("COM")
        ]
        self.port_dropdown['values'] = self.ports

        for p in self.ports:
            if "usbmodem" in p or "usbserial" in p:
                self.port_var.set(p)
                return
        if self.ports:
            self.port_var.set(self.ports[0])

    def connect_selected(self):
        port = self.port_var.get()
        if not port:
            self.status.set("No port selected")
            return

        self.status.set("CONNECTING...")

        def connect_thread():
            success = self.arduino.connect(port)
            if success:
                self.connection.set(port)
                self.status.set("CONNECTED")
                self._conn_dot.set_color(ACCENT_GREEN)
                threading.Thread(target=self.reader, daemon=True).start()
                self.root.after(500, self.load_calibration)
            else:
                self.connection.set("Not connected")
                self.status.set("CONNECTION FAILED")
                self._conn_dot.set_color(ACCENT_RED)

        threading.Thread(target=connect_thread, daemon=True).start()

    # ===========================
    # SERIAL
    # ===========================
    def reader(self):
        while self.running:
            line = self.arduino.read_line()
            if line:
                self.queue.put(line)

    def send(self, cmd):
        if not self.arduino.ser or not self.arduino.ser.is_open:
            self.status.set("ERROR: NOT CONNECTED")
            return False
        if self.arduino.send(cmd):
            self.status.set(f"SENT: {cmd}")
            return True
        # Write failed — device was disconnected. Update UI state.
        self.connection.set("Not connected")
        self.status.set("DEVICE DISCONNECTED")
        try:
            self._conn_dot.set_color(ACCENT_RED)
        except Exception:
            pass
        return False

    # ===========================
    # MODE SWITCHING
    # ===========================
    def switch_mode(self):
        self.send("STOP")
        self.pid_active = False
        self.pid_setpoint_psi = 0.0
        self.pid_panel.set_inactive()

        self.burst_panel.pack_forget()
        self.pid_panel.pack_forget()
        self.cal_panel.pack_forget()
        self.experiment_panel.pack_forget()

        mode = self.mode.get()
        if mode == "burst":
            self.burst_panel.pack(fill="both", expand=True)
        elif mode == "pid":
            self.pid_panel.pack(fill="both", expand=True)
        elif mode == "calibration":
            # Reset each wired module's stepper position and either push its
            # saved limits to firmware or clear any stale limits for modules
            # that have not been calibrated yet.
            for mod in self._wired_modules():
                mid = mod["id"]
                self.send(f"RESET_POS,{mid}")
                mod["step_position"].set(0)
                lim = self.cal_limits.get(mid)
                if lim is not None:
                    self.send(f"SET_LIMITS {mid},{lim['back']},{lim['front']}")
                else:
                    self.send(f"CLEAR_LIMITS,{mid}")
            self.step_position = 0
            # Refresh the panel for whichever module the user has selected.
            self._cal_on_module_change(self.cal_panel.selected_module_id.get())
            self.cal_panel.pack(fill="both", expand=True)
        elif mode == "experiments":
            # Select backend based on serial state.
            self._refresh_experiment_backend()
            self.experiment_panel.set_workspace(
                r_max=self.length_cal.L_max,
                L_rest=self.length_cal.L_rest,
                L_max=self.length_cal.L_max,
                theta_max_rad=1.0472,
            )
            self.experiment_panel.pack(fill="both", expand=True)

    # ===========================
    # EXPERIMENTS
    # ===========================

    def _refresh_experiment_backend(self):
        """Swap between LiveBackend and SimBackend based on serial connection state."""
        from experiment_backend import LiveBackend, SimBackend
        connected = bool(self.arduino.ser and self.arduino.ser.is_open)
        if connected and self.experiment_backend.is_sim:
            self.experiment_backend = LiveBackend(arduino=self.arduino, num_modules=len(self.modules))
            self.experiment_panel.set_backend_badge("")
        elif not connected and not self.experiment_backend.is_sim:
            self.experiment_backend = SimBackend(
                L_rest=self.length_cal.L_rest, num_modules=len(self.modules),
            )
            self.experiment_panel.set_backend_badge("MODE: SIMULATED")
        elif not connected:
            # Already sim, just make sure the badge is visible.
            self.experiment_panel.set_backend_badge("MODE: SIMULATED")
        self.experiment_controller.backend = self.experiment_backend

    def _exp_start_zero(self):
        self.experiment_controller.start_zeroing()
        self.experiment_panel.set_status("Zeroing — confirm trunk is at rest, then Confirm Zero")

    def _exp_confirm_zero(self):
        self.experiment_controller.confirm_zero()
        self.experiment_panel.set_status("Zero captured — pick a target")

    def _exp_rezero(self):
        if self.experiment_controller.rezero():
            self.experiment_panel.set_status("Re-zeroed at current rest pose")
        else:
            self.experiment_panel.set_status("Re-zero blocked — pressures not near rest")

    def _exp_recalibrate_length(self):
        if not (self.arduino.ser and self.arduino.ser.is_open):
            self.experiment_panel.set_status("Recalibrate needs a serial connection")
            return
        from panels.experiment_calibration import run_length_calibration_dialog
        live_modules = [
            mid for mid, psi in self.experiment_backend.read_state().module_pressures_psi.items()
            if isinstance(psi, (int, float))
        ]
        cal = run_length_calibration_dialog(
            root=self.root,
            available_modules=live_modules or [m["id"] for m in self.modules],
            command_pressure=lambda mid, psi: self.experiment_backend.set_module_pressure(mid, psi),
            wait_for_steady=lambda: self.root.after(1500),
        )
        if cal is not None:
            cal.save(self.length_cal_path)
            self.length_cal = cal
            self.experiment_controller.cal = cal
            self.experiment_panel.set_workspace(
                r_max=cal.L_max, L_rest=cal.L_rest, L_max=cal.L_max,
                theta_max_rad=1.0472,
            )
            self.experiment_panel.set_status("Length calibration saved")

    def _exp_reach(self, target_display):
        # Translate target from display frame (zero-relative) to physics frame
        # (base-relative). Display frame origin = (0, 0, L_rest).
        tx, ty, tz = target_display
        target_physics = (tx, ty, tz + self.length_cal.L_rest)
        try:
            self.experiment_controller.reach(target_physics)
            self.experiment_panel.set_status(f"Reaching toward {target_display}")
            self.experiment_panel.set_target_marker(target_physics)
        except ValueError as e:
            self.experiment_panel.set_status(f"Rejected: {e}")

    def _exp_emergency_stop(self):
        self.experiment_controller.emergency_stop()
        self.experiment_panel.set_status("EMERGENCY STOP")

    def _yaw_drift_deg_per_min(self, current_yaw_rad: float) -> float:
        """Estimate yaw drift over the last ~60s as degrees per minute."""
        import math as _m
        import time as _t
        now = _t.monotonic()
        self._yaw_history.append((now, current_yaw_rad))
        if len(self._yaw_history) < 2:
            return 0.0
        t0, y0 = self._yaw_history[0]
        dt = now - t0
        if dt < 5.0:
            return 0.0
        drift_rad_per_s = (current_yaw_rad - y0) / dt
        return _m.degrees(drift_rad_per_s) * 60.0

    # ===========================
    # PID CONTROL
    # ===========================
    def start_pid(self, val):
        try:
            psi = float(val)
            if psi <= 0:
                self.status.set("PSI must be positive")
                return
        except ValueError:
            self.status.set("Invalid PSI value")
            return

        hpa = psi / 0.0145038
        self.pid_setpoint_psi = psi
        self.pid_active = True

        self.send(f"SET {hpa:.1f}")
        self.pid_panel.set_active(psi, hpa)

    def stop_pid(self):
        self.send("STOP")
        self.pid_active = False
        self.pid_setpoint_psi = 0.0
        self.pid_panel.set_inactive("PID STOPPED")

    def emergency_stop(self):
        self.send("STOP")
        self.pid_active = False
        self.pid_setpoint_psi = 0.0
        self.pid_panel.set_emergency()
        self.status.set("EMERGENCY STOP SENT")

    # ===========================
    # CALIBRATION
    # ===========================
    def _wired_modules(self):
        """Return module dicts whose stepper pins are wired (non-None)."""
        return [m for m in self.modules if m["step_pin"] is not None and m["dir_pin"] is not None]

    def _module_step_position(self, module_id):
        for m in self.modules:
            if m["id"] == module_id:
                return int(m["step_position"].get())
        return 0

    def _cal_on_module_change(self, module_id):
        """Refresh the panel's back/front/range/position for the newly-selected module."""
        pos = self._module_step_position(module_id)
        self.cal_panel.cal_pos_var.set(f"{pos}")
        lim = self.cal_limits.get(module_id)
        if lim is not None:
            self.cal_panel.set_limits_display(module_id, lim["back"], lim["front"])
        else:
            self.cal_panel.clear_display(module_id)

    def cal_set_back(self, module_id):
        pos = self._module_step_position(module_id)
        entry = self.cal_limits.setdefault(module_id, {"back": None, "front": None})
        entry["back"] = pos
        self.send(f"SET_MIN,{module_id}")
        self.cal_panel.cal_back_var.set(f"Back wall: {pos} steps")
        self._cal_check_ready(module_id)

    def cal_set_front(self, module_id):
        pos = self._module_step_position(module_id)
        entry = self.cal_limits.setdefault(module_id, {"back": None, "front": None})
        entry["front"] = pos
        self.send(f"SET_MAX,{module_id}")
        self.cal_panel.cal_front_var.set(f"Front wall: {pos} steps")
        self._cal_check_ready(module_id)

    def _cal_check_ready(self, module_id):
        lim = self.cal_limits.get(module_id) or {}
        back = lim.get("back")
        front = lim.get("front")
        if back is not None and front is not None:
            if back < front:
                self.cal_panel.cal_range_var.set(f"{front - back} steps")
                self.cal_panel.enable_save(True)
            else:
                self.cal_panel.cal_range_var.set("Error: back must be < front")
                self.cal_panel.enable_save(False)

    def cal_save(self):
        # Only save modules whose back+front are both set and valid.
        payload = {}
        for mid, lim in self.cal_limits.items():
            back = lim.get("back")
            front = lim.get("front")
            if back is None or front is None or back >= front:
                continue
            payload[str(mid)] = {"back": back, "front": front}
        if not payload:
            return
        cal_data = {"modules": payload}
        try:
            with open(self.cal_file, "w") as f:
                json.dump(cal_data, f, indent=2)
            self.calibration_loaded = True
            summary = ", ".join(f"M{mid}:[{v['back']},{v['front']}]" for mid, v in payload.items())
            self.status.set(f"CALIBRATION SAVED: {summary}")
        except Exception as e:
            self.status.set(f"Error saving calibration: {e}")

    def cal_clear(self, module_id):
        """Clear limits for a single module (the one currently selected in the panel)."""
        self.send(f"CLEAR_LIMITS,{module_id}")
        self.cal_limits.pop(module_id, None)
        self.cal_panel.clear_display(module_id)
        # Rewrite the on-disk file so this module is removed from persistence.
        try:
            if self.cal_limits:
                payload = {str(mid): dict(v) for mid, v in self.cal_limits.items()
                           if v.get("back") is not None and v.get("front") is not None}
                with open(self.cal_file, "w") as f:
                    json.dump({"modules": payload}, f, indent=2)
            elif os.path.exists(self.cal_file):
                os.remove(self.cal_file)
        except Exception as e:
            print(f"[WARN] Could not update calibration file: {e}")
        self.calibration_loaded = bool(self.cal_limits)
        self.status.set(f"LIMITS CLEARED M{module_id}")

    def _load_calibration_from_disk(self):
        if not os.path.exists(self.cal_file):
            return
        try:
            with open(self.cal_file, "r") as f:
                cal_data = json.load(f)
        except Exception as e:
            print(f"[WARN] Could not load calibration: {e}")
            return
        # New format: {"modules": {"1": {"back": x, "front": y}, ...}}
        modules_dict = cal_data.get("modules")
        if isinstance(modules_dict, dict):
            for mid_str, lim in modules_dict.items():
                try:
                    mid = int(mid_str)
                    back = int(lim["back"])
                    front = int(lim["front"])
                except (ValueError, KeyError, TypeError):
                    continue
                self.cal_limits[mid] = {"back": back, "front": front}
        else:
            # Legacy flat format: {"back_wall": X, "front_wall": Y} → Module 1
            back = cal_data.get("back_wall")
            front = cal_data.get("front_wall")
            if back is not None and front is not None:
                self.cal_limits[1] = {"back": int(back), "front": int(front)}
        if self.cal_limits:
            self.calibration_loaded = True
            summary = ", ".join(f"M{mid}:[{v['back']},{v['front']}]"
                                for mid, v in sorted(self.cal_limits.items()))
            print(f"[INFO] Calibration loaded from disk: {summary}")

    def load_calibration(self):
        """Push all saved per-module limits to firmware after a fresh connect."""
        if not self.cal_limits and os.path.exists(self.cal_file):
            self._load_calibration_from_disk()
        if not self.cal_limits:
            return
        for mid, lim in self.cal_limits.items():
            self.send(f"SET_LIMITS {mid},{lim['back']},{lim['front']}")
        summary = ", ".join(f"M{mid}:[{v['back']},{v['front']}]"
                            for mid, v in sorted(self.cal_limits.items()))
        self.status.set(f"CALIBRATION LOADED: {summary}")

    # ===========================
    # UPDATE LOOP
    # ===========================
    def update_loop(self):
        while not self.queue.empty():
            line = self.queue.get()

            # Multi-module pressure: PM,module_id,time_ms,hPa,psi[,pos]
            if line.startswith("PM,"):
                parts = line.split(",")
                if len(parts) >= 5:
                    try:
                        mod_id = int(parts[1])
                        hpa = float(parts[3])
                        psi = float(parts[4])

                        for mod in self.modules:
                            if mod["id"] == mod_id:
                                mod["pressure_hpa"].set(f"{hpa:.1f}")
                                mod["pressure_psi"].set(f"{psi:.3f}")
                                mod["psi_history"].append(psi)

                                # Parse position if available
                                if len(parts) >= 6:
                                    try:
                                        pos = int(parts[5])
                                        mod["step_position"].set(pos)
                                        self.stepper_bar.set_position(mod_id, pos)
                                        self.cal_panel.set_position(mod_id, pos)
                                    except ValueError:
                                        pass
                                break
                    except (ValueError, IndexError):
                        pass

            # Legacy single-sensor: P,time_ms,hPa,psi[,pos]
            elif line.startswith("P,"):
                parts = line.split(",")
                if len(parts) >= 4:
                    t_ms = parts[1]
                    hpa = float(parts[2])
                    psi = float(parts[3])

                    if self.modules:
                        mod = self.modules[0]
                        mod["pressure_hpa"].set(f"{hpa:.1f}")
                        mod["pressure_psi"].set(f"{psi:.3f}")
                        mod["psi_history"].append(psi)

                    # Update step position
                    if len(parts) >= 5:
                        try:
                            pos = int(parts[4])
                            self.step_position = pos
                            if self.modules:
                                mod0 = self.modules[0]
                                mod0["step_position"].set(pos)
                                self.stepper_bar.set_position(mod0["id"], pos)
                                self.cal_panel.set_position(mod0["id"], pos)
                        except ValueError:
                            pass

                    self.logger.log([float(t_ms), hpa, psi, 0.0])

            # IMU data: IMU,time_ms,ax,ay,az,gx,gy,gz,tempC
            elif line.startswith("IMU,"):
                parts = line.split(",")
                if len(parts) >= 9:
                    try:
                        self.imu_data["ax"].set(f"{float(parts[2]):.4f}")
                        self.imu_data["ay"].set(f"{float(parts[3]):.4f}")
                        self.imu_data["az"].set(f"{float(parts[4]):.4f}")
                        self.imu_data["gx"].set(f"{float(parts[5]):.2f}")
                        self.imu_data["gy"].set(f"{float(parts[6]):.2f}")
                        self.imu_data["gz"].set(f"{float(parts[7]):.2f}")
                        self.imu_data["temp"].set(f"{float(parts[8]):.1f}")
                    except (ValueError, IndexError):
                        pass

            elif line.startswith("SETPOINT UPDATED"):
                self.status.set(line)
            elif line == "STOPPED":
                self.status.set("ARDUINO: STOPPED")
            else:
                self.status.set(line)

            # Route telemetry into the experiments backend when in live mode.
            if not self.experiment_backend.is_sim:
                try:
                    self.experiment_backend.ingest_serial_line(line)
                except Exception as e:
                    print(f"[WARN] experiment backend ingest failed: {e}")

        try:
            self.graph.draw(self.modules, self.pid_active, self.pid_setpoint_psi)
        except Exception as e:
            print(f"[GRAPH ERROR] {e}")

        # Advance experiment controller ~10Hz (matches update_loop cadence).
        try:
            self.experiment_controller.tick(dt=0.1)
            # Push readouts to the panel - in DISPLAY frame for the pickers,
            # in both frames for the readout labels.
            s = self.experiment_backend.read_state()
            import math as _m
            from kinematics import forward_kinematics
            theta_now = _m.hypot(s.pitch, s.roll)
            phi_now = _m.atan2(s.roll, s.pitch) if theta_now > 1e-9 else 0.0
            tip_base = forward_kinematics(s.total_length_mm, theta_now, phi_now)
            tip_zero = (tip_base[0], tip_base[1], tip_base[2] - self.length_cal.L_rest)
            yaw_drift = self._yaw_drift_deg_per_min(s.yaw)
            self.experiment_panel.set_readouts(
                tip_from_zero=tip_zero,
                tip_from_base=tip_base,
                pitch=s.pitch, roll=s.roll, yaw=s.yaw,
                yaw_drift_deg_per_min=yaw_drift,
                phase=self.experiment_controller.state.value,
                error_text=(
                    f"err {_m.degrees(self.experiment_controller.last_result.final_pitch_err_rad):.1f}° / "
                    f"{self.experiment_controller.last_result.final_position_err_mm:.1f}mm  "
                    f"in {self.experiment_controller.last_result.elapsed_s:.1f}s"
                    if self.experiment_controller.last_result else ""
                ),
            )
            # Pass tip in display frame (picker rendering) and arc in physics
            # frame (preview rendering). Panel handles the frame split.
            arc_pts_base = _sample_arc_points(s.total_length_mm, theta_now, phi_now, n=20)
            self.experiment_panel.set_tip_position(tip_zero, arc_pts_base)
        except Exception as e:
            # Keep the main loop alive even if the experiment panel breaks.
            print(f"[WARN] experiment tick failed: {e}")

        self.root.after(100, self.update_loop)

    # ===========================
    # LOGGING
    # ===========================
    def start_log(self):
        name = self.logger.start()
        if name:
            self.status.set(f"LOGGING: {name}")

    def stop_log(self):
        self.logger.stop()
        self.status.set("LOGGING STOPPED")

    # ===========================
    # CLOSE
    # ===========================
    def on_close(self):
        self.running = False
        try:
            self.arduino.send("STOP")
        except:
            pass
        self.arduino.close()
        self.root.destroy()


def _sample_arc_points(L: float, theta: float, phi: float, n: int = 20):
    """Sample n+1 points along the PCC arc from base to tip."""
    from kinematics import forward_kinematics
    return [forward_kinematics(L * i / n, theta * i / n, phi) for i in range(n + 1)]
