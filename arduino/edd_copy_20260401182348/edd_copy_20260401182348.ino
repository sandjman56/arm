#include <Wire.h>
#include <Adafruit_MPRLS.h>
#include <Servo.h>

// Forward-declare MPUData so Arduino's auto-prototype of mpuRead() compiles
struct MPUData {
  float ax, ay, az;   // g
  float gx, gy, gz;   // deg/s
  float tempC;
};

// =========================
// Bend servos (XY and XZ planes)
// =========================
// Must be PWM-capable pins. On Uno: 3,5,6,9,10,11 are PWM.
// Pins 3,4,5,7,8 are used by steppers + EN, so use 9-12.
#define BEND_SERVO_A_PIN 9   // XY left
#define BEND_SERVO_B_PIN 10  // XY right
#define BEND_SERVO_C_PIN 11  // XZ left
#define BEND_SERVO_D_PIN 24  // XZ right

// Neutral angle and max deflection (degrees)
#define BEND_CENTER_DEG 90
#define BEND_MAX_DEG    90

Servo bendServoA;
Servo bendServoB;
Servo bendServoC;
Servo bendServoD;

// Last angle each servo was commanded to. Updated on every SERVOS command.
// Startup default positions (also re-applied on every boot).
#define BEND_DEFAULT_A    -8
#define BEND_DEFAULT_B   -11
#define BEND_DEFAULT_C   157
#define BEND_DEFAULT_D   272
int lastAngleA = BEND_DEFAULT_A;
int lastAngleB = BEND_DEFAULT_B;
int lastAngleC = BEND_DEFAULT_C;
int lastAngleD = BEND_DEFAULT_D;

// =========================
// I2C Multiplexer (PCA9548A)
// =========================
#define MUX_ADDR 0x70

void tcaselect(uint8_t channel) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// =========================
// IMU — MPU-6500 via mux (raw register access)
// =========================
#define IMU_CHANNEL   2    // SD2/SC2 — matches your test wiring
#define MPU_ADDR      0x68 // AD0→GND

// MPU-6500 registers
#define REG_WHO_AM_I     0x75
#define REG_PWR_MGMT_1   0x6B
#define REG_PWR_MGMT_2   0x6C
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_ACCEL_XOUT_H  0x3B  // 14-byte burst: accel + temp + gyro

float accelScale = 1.0;
float gyroScale  = 1.0;
bool  imuPresent = false;

// =========================
// Low-level I2C register helpers (for MPU-6500)
// =========================
void writeRegister(uint8_t devAddr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(devAddr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t devAddr, uint8_t reg) {
  Wire.beginTransmission(devAddr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(devAddr, (uint8_t)1);
  return Wire.read();
}

void readRegisters(uint8_t devAddr, uint8_t reg, uint8_t *buf, uint8_t count) {
  Wire.beginTransmission(devAddr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(devAddr, count);
  for (uint8_t i = 0; i < count; i++) {
    buf[i] = Wire.read();
  }
}

// =========================
// MPU-6500 init (must call tcaselect(IMU_CHANNEL) first)
// =========================
bool mpuInit() {
  // Wake up — clear sleep bit, PLL with gyro X clock
  writeRegister(MPU_ADDR, REG_PWR_MGMT_1, 0x01);
  delay(100);

  // Verify identity
  uint8_t whoami = readRegister(MPU_ADDR, REG_WHO_AM_I);
  Serial.print("WHO_AM_I = 0x");
  Serial.println(whoami, HEX);
  if (whoami == 0xFF || whoami == 0x00) {
    Serial.println("ERROR: no device on I2C bus (got 0xFF/0x00)");
    return false;
  }
  if (whoami != 0x70 && whoami != 0x71 && whoami != 0x73) {
    Serial.print("WARNING: unexpected WHO_AM_I 0x");
    Serial.print(whoami, HEX);
    Serial.println(" — may not be MPU-6500, continuing anyway");
  }

  // Sample rate: 1000/(1+9) = 100 Hz
  writeRegister(MPU_ADDR, REG_SMPLRT_DIV, 9);

  // DLPF mode 3: accel BW 44Hz, gyro BW 42Hz
  writeRegister(MPU_ADDR, REG_CONFIG, 0x03);

  // Gyro: ±500 deg/s (bits 4:3 = 01)
  writeRegister(MPU_ADDR, REG_GYRO_CONFIG, 0x08);
  gyroScale = 500.0 / 32768.0;

  // Accel: ±4g (bits 4:3 = 01)
  writeRegister(MPU_ADDR, REG_ACCEL_CONFIG, 0x08);
  accelScale = 4.0 / 32768.0;

  // Enable all axes
  writeRegister(MPU_ADDR, REG_PWR_MGMT_2, 0x00);

  return true;
}

// =========================
// MPU-6500 burst read (must call tcaselect(IMU_CHANNEL) first)
// =========================
MPUData mpuRead() {
  MPUData d;
  uint8_t buf[14];

  // 0x3B–0x48: AXH AXL AYH AYL AZH AZL TH TL GXH GXL GYH GYL GZH GZL
  readRegisters(MPU_ADDR, REG_ACCEL_XOUT_H, buf, 14);

  int16_t rawAx = (int16_t)(buf[0]  << 8 | buf[1]);
  int16_t rawAy = (int16_t)(buf[2]  << 8 | buf[3]);
  int16_t rawAz = (int16_t)(buf[4]  << 8 | buf[5]);
  int16_t rawT  = (int16_t)(buf[6]  << 8 | buf[7]);
  int16_t rawGx = (int16_t)(buf[8]  << 8 | buf[9]);
  int16_t rawGy = (int16_t)(buf[10] << 8 | buf[11]);
  int16_t rawGz = (int16_t)(buf[12] << 8 | buf[13]);

  d.ax = rawAx * accelScale;
  d.ay = rawAy * accelScale;
  d.az = rawAz * accelScale;
  d.gx = rawGx * gyroScale;
  d.gy = rawGy * gyroScale;
  d.gz = rawGz * gyroScale;
  d.tempC = (rawT / 333.87) + 21.0;

  return d;
}

// =========================
// Sensor objects
// =========================
Adafruit_MPRLS mpr = Adafruit_MPRLS();

// =========================
// Direction constants
// =========================
#define EN_PIN   5

#define DIR_CW  LOW
#define DIR_CCW HIGH

// =========================
// Multi-module support
// =========================
#define MAX_MODULES 6

// Pressure sensor channel mapping (via PCA9548A mux)
#define NUM_PRESSURE_SENSORS 5
uint8_t pressureChannels[MAX_MODULES] = {3, 0, 1, 4, 5};
// index 0 -> Module 1 on ch 1 (SD1/SC1)
// index 1 -> Module 2 on ch 0 (SD0/SC0)
// index 2 -> Module 3 on ch 3 (SD3/SC3)
// index 3 -> Module 4 on ch 4 (SD4/SC4)
// index 4 -> Module 5 on ch 5 (SD5/SC5)
bool    pressurePresent[MAX_MODULES]  = {false};

struct Module {
  int  stepPin;
  int  dirPin;
  bool active;
  long stepPosition;
  long minStepLimit;
  long maxStepLimit;
  bool limitsActive;
  bool dirInvert;        // true = swap CW/CCW (motor wired backwards)
  // Non-blocking step state
  int  stepsRemaining;
  int  stepDirSign;      // +1 (CW) or -1 (CCW)
  bool stepHigh;         // true = pin currently HIGH (mid-step)
  unsigned long lastStepUs;
};

Module modules[MAX_MODULES];
int moduleCount = 0;

// =========================
// Motion tuning
// =========================
const int BURST_STEPS = 40;
// Pressure deadband (hPa). PID skips stepping when |err| is under this, so the
// stepper doesn't hunt/overshoot on the ~0.1 psi pneumatic noise floor. 5 hPa
// ≈ 0.07 psi.
const float PID_DEADBAND_HPA = 5.0;
const int STEP_DELAY_US = 2000;

// =========================
// Serial buffer
// =========================
String inputBuffer = "";

// =========================
// PID params (per-module)
// =========================
// Legacy globals kept so existing `SET <hpa>` (no module id) still works on module 0.
float SETPOINT_HPA = 0.0;
float currentPressure = 0.0;

float Kp = 25.0;
float Ki = 2.0;
float Kd = 8.0;

// Per-module PID state. Modules with no stepper wired will accept SET commands
// but the PID loop will no-op for them (stepBurstModule guards on active/idx).
float currentPressureM[MAX_MODULES] = {0};
float setpointHpaM[MAX_MODULES]     = {0};
bool  pidEnabledM[MAX_MODULES]      = {false};
float integralM[MAX_MODULES]        = {0};
float lastErrorM[MAX_MODULES]       = {0};

// Legacy shared PID state (module 0 mirror); kept for diagnostics only.
float integral = 0;
float lastError = 0;

// =========================
// Stepper state
// =========================
unsigned long stepInterval = 0;
unsigned long lastStepTime = 0;

int currentDir = DIR_CCW;
bool pidEnabled = false;  // legacy master flag; true iff any pidEnabledM[] is true

// =========================
// Tendon servos (Experiments mode) — mapped onto existing bend servos A/B/C/D.
// servo 1 -> A (0°),  2 -> B (90°),  3 -> C (180°),  4 -> D (270°).
// Pairs 1/3 and 2/4 are antagonists (matches Python LiveBackend mapping).
// =========================
// Continuous-rotation interpretation: write(90) = stop;
// rate in [-1000, +1000] mapped to angle 90 + (rate/1000)*TENDON_MAX_OFFSET_DEG.
const int TENDON_MAX_OFFSET_DEG = 30;  // gentle bring-up per user selection
int tendonRate[4] = {0, 0, 0, 0};  // signed -1000..1000; index 0 -> servo 1 -> A

// =========================
// Timing
// =========================
const unsigned long PID_INTERVAL_US = 25000;
unsigned long lastPIDTime = 0;

// =========================
// Helper: initialize a module slot
// =========================
void initModule(int idx, int stepPin, int dirPin) {
  modules[idx].stepPin       = stepPin;
  modules[idx].dirPin        = dirPin;
  modules[idx].active        = true;
  modules[idx].stepPosition  = 0;
  modules[idx].minStepLimit  = -999999L;
  modules[idx].maxStepLimit  = 999999L;
  modules[idx].limitsActive  = false;
  modules[idx].dirInvert     = false;
  modules[idx].stepsRemaining = 0;
  modules[idx].stepDirSign   = 1;
  modules[idx].stepHigh      = false;
  modules[idx].lastStepUs    = 0;

  // Per-module PID state.
  currentPressureM[idx] = 0.0;
  setpointHpaM[idx]     = 0.0;
  pidEnabledM[idx]      = false;
  integralM[idx]        = 0.0;
  lastErrorM[idx]       = 0.0;

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  digitalWrite(stepPin, LOW);
  digitalWrite(dirPin, DIR_CCW);
}

// =========================
// Setup
// =========================
void setup() {
  Serial.begin(115200);

  // Enable pin (shared across drivers)
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);  // ENABLE DRIVER

  // M1: STEP=3,  DIR=4   (original wiring)
  // M2: STEP=8,  DIR=7
  // M3: STEP=30, DIR=28
  // M4: STEP=34, DIR=32
  // M5: STEP=36, DIR=38
  // M6: STEP=42, DIR=40
  initModule(0, 3, 4);
  initModule(1, 8, 7);
  initModule(2, 30, 28);
  initModule(3, 34, 32);
  initModule(4, 36, 38);
  initModule(5, 42, 40);
  modules[4].dirInvert = true;  // M5 motor wired backwards
  modules[5].dirInvert = true;  // M6 motor wired backwards
  moduleCount = 6;

  // Attach bend servos on boot and drive to startup defaults so the arm
  // starts in a known pose without waiting for a GUI command.
  bendServoA.attach(BEND_SERVO_A_PIN, 500, 2500);
  bendServoB.attach(BEND_SERVO_B_PIN, 500, 2500);
  bendServoC.attach(BEND_SERVO_C_PIN, 500, 2500);
  bendServoD.attach(BEND_SERVO_D_PIN, 500, 2500);
  bendServoA.writeMicroseconds(map(BEND_DEFAULT_A, -360, 360, 500, 2500));
  bendServoB.writeMicroseconds(map(BEND_DEFAULT_B, -360, 360, 500, 2500));
  bendServoC.writeMicroseconds(map(BEND_DEFAULT_C, -360, 360, 500, 2500));
  bendServoD.writeMicroseconds(map(BEND_DEFAULT_D, -360, 360, 500, 2500));

  // Initialize I2C bus
  Wire.begin();
  Wire.setClock(400000);  // 400kHz fast mode
  Wire.setWireTimeout(3000, true);  // 3ms I2C timeout; auto-resets bus on lockup
  delay(50);

  // Verify mux is responding
  Wire.beginTransmission(MUX_ADDR);
  uint8_t muxErr = Wire.endTransmission();
  if (muxErr == 0) {
    Serial.print("MUX_OK addr=0x");
    Serial.println(MUX_ADDR, HEX);
  } else {
    Serial.print("MUX_FAIL addr=0x");
    Serial.print(MUX_ADDR, HEX);
    Serial.print(" err=");
    Serial.println(muxErr);
  }

  // Initialize pressure sensors through mux
  for (int i = 0; i < NUM_PRESSURE_SENSORS; i++) {
    tcaselect(pressureChannels[i]);
    if (mpr.begin()) {
      pressurePresent[i] = true;
      Serial.print("SENSOR_OK MPRLS ch=");
      Serial.println(pressureChannels[i]);
    } else {
      pressurePresent[i] = false;
      Serial.print("SENSOR_FAIL MPRLS ch=");
      Serial.println(pressureChannels[i]);
    }
  }

  // Initialize MPU-6500 IMU through mux
  tcaselect(IMU_CHANNEL);
  delay(10);
  if (mpuInit()) {
    imuPresent = true;
    Serial.print("SENSOR_OK MPU6500 ch=");
    Serial.println(IMU_CHANNEL);
  } else {
    imuPresent = false;
    Serial.print("SENSOR_FAIL MPU6500 ch=");
    Serial.println(IMU_CHANNEL);
  }

  Serial.println("READY");
}

// =========================
// Stepper burst for a specific module — non-blocking.
// Queues BURST_STEPS steps; loop() drains them via runSteppers().
// =========================
void stepBurstModule(int idx, int dir) {
  if (idx < 0 || idx >= moduleCount || !modules[idx].active) return;

  Module &m = modules[idx];
  int effDir = m.dirInvert ? (dir == DIR_CW ? DIR_CCW : DIR_CW) : dir;
  int dirSign = (effDir == DIR_CW) ? 1 : -1;

  // Check first step against limits before queuing.
  if (m.limitsActive) {
    long nextPos = m.stepPosition + dirSign;
    if (nextPos < m.minStepLimit || nextPos > m.maxStepLimit) {
      Serial.print("LIMIT_HIT ");
      Serial.print(idx + 1);
      Serial.print(",");
      Serial.println(m.stepPosition);
      return;
    }
  }

  digitalWrite(m.dirPin, effDir);
  m.stepDirSign    = dirSign;
  m.stepsRemaining = BURST_STEPS;
  m.stepHigh       = false;
  m.lastStepUs     = 0;  // fire on next runSteppers() call
}

// =========================
// Non-blocking step driver — call once per loop() iteration.
// Toggles step pins at STEP_DELAY_US intervals without blocking.
// =========================
void runSteppers() {
  unsigned long nowUs = micros();
  for (int i = 0; i < moduleCount; i++) {
    Module &m = modules[i];
    if (!m.active || m.stepsRemaining <= 0) continue;
    if ((unsigned long)(nowUs - m.lastStepUs) < (unsigned long)STEP_DELAY_US) continue;

    m.lastStepUs = nowUs;

    if (m.stepHigh) {
      // Falling edge — step completes, advance position.
      digitalWrite(m.stepPin, LOW);
      m.stepHigh = false;
      m.stepPosition += m.stepDirSign;
      m.stepsRemaining--;
    } else {
      // Rising edge — check limit before committing.
      long nextPos = m.stepPosition + m.stepDirSign;
      if (m.limitsActive && (nextPos < m.minStepLimit || nextPos > m.maxStepLimit)) {
        Serial.print("LIMIT_HIT ");
        Serial.print(i + 1);
        Serial.print(",");
        Serial.println(m.stepPosition);
        m.stepsRemaining = 0;
      } else {
        digitalWrite(m.stepPin, HIGH);
        m.stepHigh = true;
      }
    }
  }
}

// Legacy wrapper: burst on module 0 (for PID / calibration)
void stepBurst(int dir) {
  stepBurstModule(0, dir);
}

// =========================
// Parse module index from args (1-based in protocol, 0-based internally)
// Returns 0 if args is empty or invalid (default to module 1)
// =========================
int parseModuleIdx(String &args) {
  if (args.length() == 0) return 0;
  int id = args.toInt();
  if (id < 1 || id > moduleCount) return 0;
  return id - 1;
}

// =========================
// Command handling
// =========================
void handleCommand(String cmd) {
  cmd.trim();

  // Extract first word as command (split on first space or comma)
  String command = cmd;
  String args = "";

  int spaceIdx = cmd.indexOf(' ');
  int commaIdx = cmd.indexOf(',');

  int splitIdx = -1;
  if (spaceIdx != -1 && commaIdx != -1) {
    splitIdx = min(spaceIdx, commaIdx);
  } else if (spaceIdx != -1) {
    splitIdx = spaceIdx;
  } else if (commaIdx != -1) {
    splitIdx = commaIdx;
  }

  if (splitIdx != -1) {
    command = cmd.substring(0, splitIdx);
    args = cmd.substring(splitIdx + 1);
    args.trim();
  }

  // === PID commands ===

  if (command == "SET") {
    // Two forms:
    //   SET <hpa>            -> legacy, targets module 0 (module id 1)
    //   SET <mod_id> <hpa>   -> experiments-mode, per-module setpoint (1-based)
    int sp = args.indexOf(' ');
    int modIdx = 0;
    float val = 0.0;
    if (sp == -1) {
      // Legacy: single value -> module 0.
      val = args.toFloat();
      modIdx = 0;
    } else {
      int id = args.substring(0, sp).toInt();
      val = args.substring(sp + 1).toFloat();
      if (id < 1 || id > MAX_MODULES) {
        Serial.print("SET_BAD_MOD ");
        Serial.println(id);
        return;
      }
      modIdx = id - 1;
    }

    setpointHpaM[modIdx] = val;
    pidEnabledM[modIdx]  = true;
    integralM[modIdx]    = 0;
    lastErrorM[modIdx]   = 0;
    pidEnabled = true;  // master flag

    // Keep legacy single-module globals in sync when module 0 is targeted.
    if (modIdx == 0) {
      SETPOINT_HPA = val;
      integral = 0;
      lastError = 0;
    }

    Serial.print("SETPOINT UPDATED M");
    Serial.print(modIdx + 1);
    Serial.print(" -> ");
    Serial.println(val);
  }

  else if (command == "STOP") {
    pidEnabled = false;
    stepInterval = 0;
    // Clear every per-module PID state.
    for (int i = 0; i < MAX_MODULES; i++) {
      pidEnabledM[i] = false;
      setpointHpaM[i] = 0;
      integralM[i] = 0;
      lastErrorM[i] = 0;
    }
    // Halt all tendons and release their servo outputs.
    for (int i = 0; i < 4; i++) tendonRate[i] = 0;
    if (bendServoA.attached()) bendServoA.detach();
    if (bendServoB.attached()) bendServoB.detach();
    if (bendServoC.attached()) bendServoC.detach();
    if (bendServoD.attached()) bendServoD.detach();
    Serial.println("STOPPED");
  }

  // === Burst commands (with optional module ID) ===
  // GUI sends: INFLATE,1  INFLATE,2  DEFLATE,1  etc.
  // Legacy (no arg) defaults to module 1

  else if (command == "INFLATE") {
    pidEnabled = false;
    int idx = parseModuleIdx(args);
    Serial.print("ACTION: INFLATE M");
    Serial.println(idx + 1);
    stepBurstModule(idx, DIR_CW);
  }

  // === Bend servo single set ===
  // SERVO,<id>,<angle>  — id is 1..4 (A/B/C/D -> pins 9/10/11/24),
  // angle is 0..180. Attaches the servo if detached and holds it at
  // the commanded angle. Sent live as the GUI slider is dragged.
  else if (command == "SERVO") {
    int c1 = args.indexOf(',');
    if (c1 < 0) {
      Serial.println("SERVO FAIL need id,angle");
      return;
    }
    int id    = args.substring(0, c1).toInt();
    int angle = constrain(args.substring(c1 + 1).toInt(), -360, 360);
    Servo *s = nullptr;
    int pin = -1;
    int *lastAngle = nullptr;
    switch (id) {
      case 1: s = &bendServoA; pin = BEND_SERVO_A_PIN; lastAngle = &lastAngleA; break;
      case 2: s = &bendServoB; pin = BEND_SERVO_B_PIN; lastAngle = &lastAngleB; break;
      case 3: s = &bendServoC; pin = BEND_SERVO_C_PIN; lastAngle = &lastAngleC; break;
      case 4: s = &bendServoD; pin = BEND_SERVO_D_PIN; lastAngle = &lastAngleD; break;
      default:
        Serial.print("SERVO FAIL bad_id ");
        Serial.println(id);
        return;
    }
    if (!s->attached()) s->attach(pin, 500, 2500);
    s->writeMicroseconds(map(angle, -360, 360, 500, 2500));
    *lastAngle = angle;

    Serial.print("SERVO ");
    Serial.print(id);
    Serial.print(" ");
    Serial.println(angle);
  }

  // === Bend servo bulk set ===
  // SERVOS,<a>,<b>,<c>,<d>  — absolute angles (0..180) for servos A/B/C/D
  // (pins 9/10/11/24). Writes all four at once. Used by the GUI "Save"
  // button to resend the current state.
  else if (command == "SERVOS") {
    int c1 = args.indexOf(',');
    int c2 = (c1 >= 0) ? args.indexOf(',', c1 + 1) : -1;
    int c3 = (c2 >= 0) ? args.indexOf(',', c2 + 1) : -1;
    if (c1 < 0 || c2 < 0 || c3 < 0) {
      Serial.println("SERVOS FAIL need a,b,c,d");
      return;
    }
    int aA = constrain(args.substring(0, c1).toInt(),       -360, 360);
    int aB = constrain(args.substring(c1 + 1, c2).toInt(),  -360, 360);
    int aC = constrain(args.substring(c2 + 1, c3).toInt(),  -360, 360);
    int aD = constrain(args.substring(c3 + 1).toInt(),      -360, 360);

    if (!bendServoA.attached()) bendServoA.attach(BEND_SERVO_A_PIN, 500, 2500);
    if (!bendServoB.attached()) bendServoB.attach(BEND_SERVO_B_PIN, 500, 2500);
    if (!bendServoC.attached()) bendServoC.attach(BEND_SERVO_C_PIN, 500, 2500);
    if (!bendServoD.attached()) bendServoD.attach(BEND_SERVO_D_PIN, 500, 2500);

    bendServoA.writeMicroseconds(map(aA, -360, 360, 500, 2500)); lastAngleA = aA;
    bendServoB.writeMicroseconds(map(aB, -360, 360, 500, 2500)); lastAngleB = aB;
    bendServoC.writeMicroseconds(map(aC, -360, 360, 500, 2500)); lastAngleC = aC;
    bendServoD.writeMicroseconds(map(aD, -360, 360, 500, 2500)); lastAngleD = aD;

    Serial.print("SERVOS ");
    Serial.print(aA); Serial.print(",");
    Serial.print(aB); Serial.print(",");
    Serial.print(aC); Serial.print(",");
    Serial.println(aD);
  }

  else if (command == "DEFLATE") {
    pidEnabled = false;
    int idx = parseModuleIdx(args);
    Serial.print("ACTION: DEFLATE M");
    Serial.println(idx + 1);
    stepBurstModule(idx, DIR_CCW);
  }

  // === Tendon command (Experiments mode) ===
  // TEND <servo_id> <rate>
  //   servo_id: 1..4 (1->A, 2->B, 3->C, 4->D; pairs 1/3 and 2/4 antagonistic)
  //   rate:     signed int -1000..+1000
  // Continuous-rotation servo: write(90) = stop. Rate mapped onto
  // 90 +/- TENDON_MAX_OFFSET_DEG for gentle bring-up.
  else if (command == "TEND") {
    int sp = args.indexOf(' ');
    if (sp == -1) {
      Serial.println("TEND_BAD_ARGS");
      return;
    }
    int servoId = args.substring(0, sp).toInt();
    int rate    = args.substring(sp + 1).toInt();
    if (servoId < 1 || servoId > 4) {
      Serial.print("TEND_BAD_SERVO ");
      Serial.println(servoId);
      return;
    }
    if (rate < -1000) rate = -1000;
    if (rate >  1000) rate =  1000;
    tendonRate[servoId - 1] = rate;

    // Map rate -> angle (90 +/- TENDON_MAX_OFFSET_DEG).
    int offset = (int)((long)rate * TENDON_MAX_OFFSET_DEG / 1000);
    int angle  = BEND_CENTER_DEG + offset;

    // Select which Servo object backs this tendon.
    Servo *s = nullptr;
    int pin = 0;
    switch (servoId) {
      case 1: s = &bendServoA; pin = BEND_SERVO_A_PIN; break;
      case 2: s = &bendServoB; pin = BEND_SERVO_B_PIN; break;
      case 3: s = &bendServoC; pin = BEND_SERVO_C_PIN; break;
      case 4: s = &bendServoD; pin = BEND_SERVO_D_PIN; break;
    }

    if (rate == 0) {
      // Stop: detach to release the output (CR servos at write(90) still hold).
      if (s->attached()) s->detach();
    } else {
      if (!s->attached()) s->attach(pin);
      s->write(angle);
    }

    Serial.print("TEND ");
    Serial.print(servoId);
    Serial.print(" rate=");
    Serial.print(rate);
    Serial.print(" angle=");
    Serial.println(angle);
  }

  // === PID gain tuning ===

  else if (command == "KP") {
    Kp = args.toFloat();
    Serial.print("KP -> ");
    Serial.println(Kp);
  }

  else if (command == "KI") {
    Ki = args.toFloat();
    integral = 0;
    Serial.print("KI -> ");
    Serial.println(Ki);
  }

  else if (command == "KD") {
    Kd = args.toFloat();
    Serial.print("KD -> ");
    Serial.println(Kd);
  }

  // === Calibration commands (per-module, 1-indexed) ===
  // Every command below accepts an optional module ID (1..moduleCount) as its
  // first arg. Omitting it (or passing an out-of-range value) defaults to
  // Module 1, preserving the legacy single-module protocol.
  //   RESET_POS            RESET_POS,<mod>
  //   SET_MIN              SET_MIN,<mod>
  //   SET_MAX              SET_MAX,<mod>
  //   CLEAR_LIMITS         CLEAR_LIMITS,<mod>
  //   GET_POS              GET_POS,<mod>
  //   SET_LIMITS <min>,<max>        (legacy, Module 1)
  //   SET_LIMITS <mod>,<min>,<max>  (per-module)

  else if (command == "RESET_POS") {
    int idx = parseModuleIdx(args);
    modules[idx].stepPosition = 0;
    Serial.print("POS_RESET M");
    Serial.println(idx + 1);
  }

  else if (command == "SET_MIN") {
    int idx = parseModuleIdx(args);
    modules[idx].minStepLimit = modules[idx].stepPosition;
    modules[idx].limitsActive = (modules[idx].minStepLimit != -999999L || modules[idx].maxStepLimit != 999999L);
    Serial.print("MIN_SET M");
    Serial.print(idx + 1);
    Serial.print(" ");
    Serial.println(modules[idx].minStepLimit);
  }

  else if (command == "SET_MAX") {
    int idx = parseModuleIdx(args);
    modules[idx].maxStepLimit = modules[idx].stepPosition;
    modules[idx].limitsActive = (modules[idx].minStepLimit != -999999L || modules[idx].maxStepLimit != 999999L);
    Serial.print("MAX_SET M");
    Serial.print(idx + 1);
    Serial.print(" ");
    Serial.println(modules[idx].maxStepLimit);
  }

  else if (command == "SET_LIMITS") {
    int firstComma  = args.indexOf(',');
    int secondComma = (firstComma != -1) ? args.indexOf(',', firstComma + 1) : -1;
    if (firstComma != -1) {
      int idx;
      long minVal;
      long maxVal;
      if (secondComma != -1) {
        // New 3-arg form: <mod>,<min>,<max>
        int id = args.substring(0, firstComma).toInt();
        if (id < 1 || id > moduleCount) {
          Serial.print("SET_LIMITS_BAD_MOD ");
          Serial.println(id);
          return;
        }
        idx    = id - 1;
        minVal = args.substring(firstComma + 1, secondComma).toInt();
        maxVal = args.substring(secondComma + 1).toInt();
      } else {
        // Legacy 2-arg form: <min>,<max> targets Module 1
        idx    = 0;
        minVal = args.substring(0, firstComma).toInt();
        maxVal = args.substring(firstComma + 1).toInt();
      }
      modules[idx].minStepLimit = minVal;
      modules[idx].maxStepLimit = maxVal;
      modules[idx].limitsActive = true;
      Serial.print("LIMITS_SET M");
      Serial.print(idx + 1);
      Serial.print(" ");
      Serial.print(modules[idx].minStepLimit);
      Serial.print(",");
      Serial.println(modules[idx].maxStepLimit);
    }
  }

  else if (command == "CLEAR_LIMITS") {
    int idx = parseModuleIdx(args);
    modules[idx].limitsActive = false;
    modules[idx].minStepLimit = -999999L;
    modules[idx].maxStepLimit = 999999L;
    Serial.print("LIMITS_CLEARED M");
    Serial.println(idx + 1);
  }

  else if (command == "GET_POS") {
    int idx = parseModuleIdx(args);
    Serial.print("POS M");
    Serial.print(idx + 1);
    Serial.print(" ");
    Serial.println(modules[idx].stepPosition);
  }

  else {
    Serial.print("UNKNOWN: ");
    Serial.println(cmd);
  }
}

// =========================
// Main loop
// =========================
void loop() {

  // Non-blocking stepper driver — runs first so step timing is as tight as possible.
  runSteppers();

  // =========================
  // SERIAL INPUT
  // =========================
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      handleCommand(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }

  // =========================
  // SENSOR UPDATE + STREAM (20 Hz)
  // =========================
  static unsigned long lastStreamTime = 0;
  unsigned long nowMs = millis();

  if (nowMs - lastStreamTime >= 50) {
    lastStreamTime = nowMs;

    // Read and stream each pressure sensor
    for (int i = 0; i < NUM_PRESSURE_SENSORS; i++) {
      if (!pressurePresent[i]) continue;

      tcaselect(pressureChannels[i]);
      float hpa = mpr.readPressure();
      float psi = hpa * 0.0145038;

      // Module IDs are 1-based (matches GUI)
      int modId = i + 1;
      long stepPos = (i < moduleCount) ? modules[i].stepPosition : 0;

      Serial.print("PM,");
      Serial.print(modId);
      Serial.print(",");
      Serial.print(nowMs);
      Serial.print(",");
      Serial.print(hpa, 1);
      Serial.print(",");
      Serial.print(psi, 4);
      Serial.print(",");
      Serial.println(stepPos);

      // Store per-module pressure for the per-module PID loop.
      if (i < MAX_MODULES) currentPressureM[i] = hpa;
      // Keep legacy module-0 pressure global in sync.
      if (i == 0) currentPressure = hpa;
    }

    // Stream step position for every wired module, independent of pressure
    // sensor presence. Modules without a pressure sensor still need their
    // stepper position visible on the GUI so calibration can work.
    for (int i = 0; i < moduleCount; i++) {
      if (!modules[i].active) continue;
      Serial.print("POS,");
      Serial.print(i + 1);
      Serial.print(",");
      Serial.println(modules[i].stepPosition);
    }

    // Read and stream IMU data
    if (imuPresent) {
      tcaselect(IMU_CHANNEL);
      MPUData d = mpuRead();

      Serial.print("IMU,");
      Serial.print(nowMs);
      Serial.print(",");
      Serial.print(d.ax, 4);
      Serial.print(",");
      Serial.print(d.ay, 4);
      Serial.print(",");
      Serial.print(d.az, 4);
      Serial.print(",");
      Serial.print(d.gx, 2);
      Serial.print(",");
      Serial.print(d.gy, 2);
      Serial.print(",");
      Serial.print(d.gz, 2);
      Serial.print(",");
      Serial.println(d.tempC, 1);
    }
  }

  // =========================
  // PID LOOP (per-module)
  // =========================
  // Iterates over every module with pidEnabledM[i] = true, runs a PID step
  // using that module's setpoint + currentPressureM[i], and bursts its
  // stepper. Modules without a wired stepper or sensor are simply skipped.
  unsigned long nowUs = micros();

  if (pidEnabled && (nowUs - lastPIDTime >= PID_INTERVAL_US)) {
    lastPIDTime = nowUs;

    for (int i = 0; i < moduleCount; i++) {
      if (!pidEnabledM[i]) continue;
      if (!modules[i].active) continue;
      if (modules[i].stepsRemaining > 0) continue;  // mid-burst, wait

      float err = setpointHpaM[i] - currentPressureM[i];

      // Deadband: within tolerance of setpoint, hold position and freeze the
      // integrator so it doesn't wind up on sensor noise.
      if (fabs(err) < PID_DEADBAND_HPA) {
        lastErrorM[i] = err;
        continue;
      }

      integralM[i] += err * 0.025;
      float derivative = (err - lastErrorM[i]) / 0.025;
      float output = Kp * err + Ki * integralM[i] + Kd * derivative;
      lastErrorM[i] = err;

      if (output > 0) {
        stepBurstModule(i, DIR_CW);    // inflate
      } else if (output < 0) {
        stepBurstModule(i, DIR_CCW);   // deflate
      }

      // Keep legacy globals in sync for module 0 observers.
      if (i == 0) {
        SETPOINT_HPA = setpointHpaM[0];
        integral = integralM[0];
        lastError = lastErrorM[0];
      }
    }
  }
}
