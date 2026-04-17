# PID Controller UI — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a fully functional PID pressure controller mode to `armuimac.py`, completely separated from burst mode, with emergency stops for both.

**Architecture:** The UI gets a mode toggle (Burst / PID) that swaps which control panel is visible. Each mode has its own emergency stop. PID mode lets the user enter a goal PSI, converts to hPa, and sends `SET <hpa>` to the Arduino. The Arduino already handles PID natively — no firmware changes needed. The graph shows the setpoint line when PID is active.

**Tech Stack:** Python 3, tkinter (already in use), serial (already in use)

---

## Arduino Changes: NONE

The Arduino firmware already supports everything needed:

| Command | Arduino Behavior |
|---------|-----------------|
| `SET <hpa_float>` | Enables PID, sets setpoint in hPa, resets integral/error |
| `STOP` | Disables PID, stops motor |
| `INFLATE` | Disables PID, single burst CCW |
| `DEFLATE` | Disables PID, single burst CW |

The current `send_pid()` in Python sends `{val}P` which the Arduino doesn't recognize (it prints `UNKNOWN`). We fix this by sending `SET <hpa>` instead.

**You do NOT need to change or re-flash the Arduino.**

---

## File Structure

Only one file changes:

- **Modify:** `armuimac.py` — the entire UI application

Changes are organized into these logical areas:
1. **Mode state** — track current mode (burst vs pid), setpoint
2. **Mode toggle UI** — radio buttons to switch modes
3. **Burst panel** — existing inflate/deflate controls + emergency stop, in a togglable frame
4. **PID panel** — goal PSI input, start/stop, status, emergency stop, in a togglable frame
5. **Graph enhancement** — setpoint horizontal line when PID active
6. **Fix `send_pid`** — send correct `SET` command with PSI→hPa conversion

---

### Task 1: Add Mode State Variables

**Files:**
- Modify: `armuimac.py:59-85` (inside `__init__`)

- [ ] **Step 1: Add mode tracking variables to `__init__`**

Add these after the existing `self.log_start = None` line (after line 79):

```python
# Mode state: "burst" or "pid"
self.mode = tk.StringVar(value="burst")
self.pid_active = False
self.pid_setpoint_psi = 0.0
```

- [ ] **Step 2: Commit**

```bash
git add armuimac.py
git commit -m "feat: add mode state variables for burst/pid separation"
```

---

### Task 2: Restructure UI — Mode Toggle + Separate Panels

**Files:**
- Modify: `armuimac.py:97-174` (the `build_ui` method)

This is the largest task. We replace the current single "Inflation Control" frame with:
- A mode selector (two radio buttons)
- A Burst panel (visible in burst mode)
- A PID panel (visible in pid mode)

- [ ] **Step 1: Replace the main control area in `build_ui`**

Replace everything from line 130 (`main = tk.Frame(self.root)`) through line 173 (end of bending section) with:

```python
        # =========================
        # MODE SELECTOR
        # =========================
        mode_frame = tk.Frame(self.root)
        mode_frame.pack(fill="x", padx=20, pady=(10, 0))

        tk.Label(mode_frame, text="Mode:", font=("Arial", 14, "bold")).pack(side="left")
        tk.Radiobutton(mode_frame, text="Burst Control", variable=self.mode,
                        value="burst", font=("Arial", 12),
                        command=self.switch_mode).pack(side="left", padx=10)
        tk.Radiobutton(mode_frame, text="PID Control", variable=self.mode,
                        value="pid", font=("Arial", 12),
                        command=self.switch_mode).pack(side="left", padx=10)

        # Container for swappable panels
        self.panel_container = tk.Frame(self.root)
        self.panel_container.pack(fill="both", expand=True, padx=10, pady=5)

        # =========================
        # BURST PANEL
        # =========================
        self.burst_panel = tk.LabelFrame(self.panel_container, text="Burst Control")

        main_burst = tk.Frame(self.burst_panel)
        main_burst.pack(fill="both", expand=True)

        # Inflation controls (left side)
        infl = tk.LabelFrame(main_burst, text="Inflation Control")
        infl.pack(side="left", fill="both", expand=True, padx=10)

        for i in range(4):
            tk.Label(infl, text=f"Module {i+1}").grid(row=i, column=0)
            tk.Button(infl, text="Inflate",
                      command=lambda i=i: self.send(f"DEFLATE,{i+1}")).grid(row=i, column=1)
            tk.Button(infl, text="Deflate",
                      command=lambda i=i: self.send(f"INFLATE,{i+1}")).grid(row=i, column=2)

        # Bending controls (right side)
        bend = tk.LabelFrame(main_burst, text="Bending")
        bend.pack(side="right", fill="both", expand=True, padx=10)

        tk.Button(bend, text="Left",
                  command=lambda: self.send("BEND,LEFT")).pack(pady=10)
        tk.Button(bend, text="Right",
                  command=lambda: self.send("BEND,RIGHT")).pack(pady=10)
        tk.Button(bend, text="Center",
                  command=lambda: self.send("BEND,CENTER")).pack(pady=10)

        # Burst emergency stop
        tk.Button(self.burst_panel, text="EMERGENCY STOP", bg="red", fg="white",
                  font=("Arial", 16, "bold"), height=2,
                  command=self.emergency_stop).pack(fill="x", padx=10, pady=10)

        # =========================
        # PID PANEL
        # =========================
        self.pid_panel = tk.LabelFrame(self.panel_container, text="PID Pressure Control")

        # Goal PSI input row
        input_row = tk.Frame(self.pid_panel)
        input_row.pack(pady=15)

        tk.Label(input_row, text="Goal Pressure (PSI):", font=("Arial", 13)).pack(side="left")
        self.pid_entry = tk.Entry(input_row, width=10, font=("Arial", 14))
        self.pid_entry.pack(side="left", padx=10)

        self.pid_start_btn = tk.Button(input_row, text="Start PID", bg="#2196F3", fg="white",
                                        font=("Arial", 13, "bold"),
                                        command=self.start_pid)
        self.pid_start_btn.pack(side="left", padx=5)

        self.pid_stop_btn = tk.Button(input_row, text="Stop PID", font=("Arial", 13),
                                       command=self.stop_pid, state="disabled")
        self.pid_stop_btn.pack(side="left", padx=5)

        # PID status display
        self.pid_status_var = tk.StringVar(value="PID inactive")
        self.pid_status_label = tk.Label(self.pid_panel, textvariable=self.pid_status_var,
                                          font=("Arial", 14), fg="gray")
        self.pid_status_label.pack(pady=5)

        # PID emergency stop
        tk.Button(self.pid_panel, text="EMERGENCY STOP", bg="red", fg="white",
                  font=("Arial", 16, "bold"), height=2,
                  command=self.emergency_stop).pack(fill="x", padx=10, pady=10)

        # Show default panel
        self.burst_panel.pack(fill="both", expand=True)
```

- [ ] **Step 2: Verify the old individual Stop buttons per module and "STOP ALL" are removed**

The old per-module Stop buttons and "STOP ALL" button (lines 146-150) are gone — replaced by the emergency stop. The burst panel no longer has per-module stop buttons since the emergency stop covers all cases.

- [ ] **Step 3: Commit**

```bash
git add armuimac.py
git commit -m "feat: restructure UI with mode toggle, burst panel, and PID panel"
```

---

### Task 3: Implement Mode Switching Logic

**Files:**
- Modify: `armuimac.py` — add `switch_mode` method

- [ ] **Step 1: Add `switch_mode` method**

Add after the `send_pid` method (which will be replaced in Task 4):

```python
    def switch_mode(self):
        """Switch between burst and PID panels. Sends STOP to Arduino on any switch."""
        # Always stop whatever is happening when switching modes
        self.send("STOP")
        self.pid_active = False
        self.pid_setpoint_psi = 0.0
        self.pid_status_var.set("PID inactive")
        self.pid_status_label.config(fg="gray")
        self.pid_start_btn.config(state="normal")
        self.pid_stop_btn.config(state="disabled")

        # Hide both panels
        self.burst_panel.pack_forget()
        self.pid_panel.pack_forget()

        # Show selected panel
        if self.mode.get() == "burst":
            self.burst_panel.pack(fill="both", expand=True)
        else:
            self.pid_panel.pack(fill="both", expand=True)
```

- [ ] **Step 2: Commit**

```bash
git add armuimac.py
git commit -m "feat: add mode switching logic with auto-stop on transition"
```

---

### Task 4: Implement PID Start/Stop and Emergency Stop

**Files:**
- Modify: `armuimac.py` — replace `send_pid`, add `start_pid`, `stop_pid`, `emergency_stop`

- [ ] **Step 1: Replace `send_pid` with `start_pid` and `stop_pid`**

Delete the existing `send_pid` method (lines 250-256) and replace with:

```python
    def start_pid(self):
        """Convert PSI goal to hPa and send SET command to Arduino."""
        val = self.pid_entry.get().strip()
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

        self.pid_status_var.set(f"Holding {psi:.2f} PSI ({hpa:.1f} hPa)")
        self.pid_status_label.config(fg="#2196F3")
        self.pid_start_btn.config(state="disabled")
        self.pid_stop_btn.config(state="normal")

    def stop_pid(self):
        """Stop PID control."""
        self.send("STOP")
        self.pid_active = False
        self.pid_setpoint_psi = 0.0
        self.pid_status_var.set("PID stopped")
        self.pid_status_label.config(fg="gray")
        self.pid_start_btn.config(state="normal")
        self.pid_stop_btn.config(state="disabled")

    def emergency_stop(self):
        """Emergency stop — works in both modes, sends STOP to Arduino."""
        self.send("STOP")
        self.pid_active = False
        self.pid_setpoint_psi = 0.0

        # Reset PID UI state
        self.pid_status_var.set("EMERGENCY STOPPED")
        self.pid_status_label.config(fg="red")
        self.pid_start_btn.config(state="normal")
        self.pid_stop_btn.config(state="disabled")

        self.status.set("EMERGENCY STOP SENT")
```

- [ ] **Step 2: Commit**

```bash
git add armuimac.py
git commit -m "feat: add PID start/stop with PSI→hPa conversion and emergency stop"
```

---

### Task 5: Add Setpoint Line to Pressure Graph

**Files:**
- Modify: `armuimac.py` — the `draw_graph` method

- [ ] **Step 1: Add setpoint line rendering at the end of `draw_graph`**

Add this block just before the X-axis label line (`c.create_text(w // 2, h - 3, text="Time →"...)`):

```python
        # Setpoint line (when PID is active)
        if self.pid_active and self.pid_setpoint_psi > 0:
            sp = self.pid_setpoint_psi
            if y_min <= sp <= y_max:
                sp_y = pad_t + plot_h * (1 - (sp - y_min) / (y_max - y_min))
                c.create_line(pad_l, sp_y, w - pad_r, sp_y,
                              fill="#ff4444", width=1, dash=(6, 3))
                c.create_text(pad_l + 5, sp_y - 10, anchor="w",
                              text=f"Goal: {sp:.2f} PSI", fill="#ff4444",
                              font=("Arial", 9, "bold"))
```

- [ ] **Step 2: Commit**

```bash
git add armuimac.py
git commit -m "feat: draw PID setpoint line on pressure graph"
```

---

### Task 6: Update `on_close` to Use Emergency Stop

**Files:**
- Modify: `armuimac.py` — the `on_close` method

- [ ] **Step 1: Update `on_close`**

Replace the existing `on_close` method:

```python
    def on_close(self):
        self.running = False
        try:
            self.arduino.send("STOP")
        except:
            pass
        self.arduino.close()
        self.root.destroy()
```

The change: use `self.arduino.send("STOP")` directly instead of `self.send("STOP,ALL")` — the Arduino only understands `STOP`, not `STOP,ALL`.

- [ ] **Step 2: Commit**

```bash
git add armuimac.py
git commit -m "fix: send correct STOP command on window close"
```

---

### Task 7: Handle Arduino PID Feedback in Update Loop

**Files:**
- Modify: `armuimac.py` — the `update_loop` method

- [ ] **Step 1: Add parsing for Arduino `SETPOINT UPDATED` and `STOPPED` messages**

In the `update_loop` method, in the `else` branch (line 278 area, `self.status.set(line)`), add specific handling:

```python
            elif line.startswith("SETPOINT UPDATED"):
                self.status.set(line)
            elif line == "STOPPED":
                self.status.set("Arduino: STOPPED")
            else:
                self.status.set(line)
```

This replaces the plain `else: self.status.set(line)` so specific Arduino responses are handled clearly.

- [ ] **Step 2: Commit**

```bash
git add armuimac.py
git commit -m "feat: handle Arduino PID feedback messages in update loop"
```

---

### Task 8: Manual Smoke Test

- [ ] **Step 1: Run the application**

```bash
cd /Users/sanderschulman/Developer/arm
python armuimac.py
```

- [ ] **Step 2: Verify without Arduino connected**

1. App opens without crash
2. Mode toggle switches between Burst and PID panels
3. Burst panel shows inflate/deflate buttons + red emergency stop
4. PID panel shows PSI input, Start/Stop buttons, status label, red emergency stop
5. Clicking "Start PID" without connection shows "ERROR: Not connected"
6. Emergency stop button works in both modes

- [ ] **Step 3: Verify with Arduino connected**

1. Connect to Arduino
2. Switch to PID mode
3. Enter a PSI value (e.g., `15.0`), click Start PID
4. Status shows "Holding 15.00 PSI (1034.2 hPa)"
5. Graph shows red dashed setpoint line at 15.0 PSI
6. Arduino responds with "SETPOINT UPDATED -> 1034.2"
7. Click Stop PID — Arduino stops, status resets
8. Switch to Burst mode — verify inflate/deflate work
9. Emergency stop works in both modes

- [ ] **Step 4: Final commit**

```bash
git add armuimac.py
git commit -m "feat: complete PID controller mode with mode separation and emergency stops"
```

---

## Summary of Arduino Commands Used

| Python sends | When | Arduino does |
|---|---|---|
| `SET {hpa:.1f}` | User clicks "Start PID" in PID mode | Enables PID, sets setpoint |
| `STOP` | Emergency stop, Stop PID, mode switch, window close | Disables PID, stops motor |
| `INFLATE` | Burst mode inflate button | Single burst CW (PID auto-disabled) |
| `DEFLATE` | Burst mode deflate button | Single burst CCW (PID auto-disabled) |

**No Arduino firmware changes required.**
