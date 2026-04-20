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
#define BEND_MAX_DEG    60

Servo bendServoA;
Servo bendServoB;
Servo bendServoC;
Servo bendServoD;
int bendValue = 0;    // XY -100..100
int bendValueXZ = 0;  // XZ -100..100
int bendValueAll = 0; // All servos -100..100

// Running center per servo. Shifts on CENTER (v=0) to latch the last commanded
// angle, so subsequent slider moves are incremental from the held position
// rather than snapping back to BEND_CENTER_DEG.
int centerAngleA = BEND_CENTER_DEG;
int centerAngleB = BEND_CENTER_DEG;
int centerAngleC = BEND_CENTER_DEG;
int centerAngleD = BEND_CENTER_DEG;
// Last angle each servo was commanded to (source for the center latch).
int lastAngleA = BEND_CENTER_DEG;
int lastAngleB = BEND_CENTER_DEG;
int lastAngleC = BEND_CENTER_DEG;
int lastAngleD = BEND_CENTER_DEG;

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
#define MAX_MODULES 8

// Pressure sensor channel mapping (via PCA9548A mux)
#define NUM_PRESSURE_SENSORS 3
uint8_t pressureChannels[MAX_MODULES] = {1, 0, 3};
// index 0 -> Module 1 on ch 1 (SD1/SC1)
// index 1 -> Module 2 on ch 0 (SD0/SC0)
// index 2 -> Module 3 on ch 3 (SD3/SC3)
bool    pressurePresent[MAX_MODULES]  = {false};

struct Module {
  int  stepPin;
  int  dirPin;
  bool active;
  long stepPosition;
  long minStepLimit;
  long maxStepLimit;
  bool limitsActive;
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
// Set true once PID has emitted a "blocked, no limits calibrated" warning for
// this module, so the warning fires at most once per pid-enable cycle.
bool  pidLimitWarned[MAX_MODULES]   = {false};

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

  // Default Module 1: STEP=3, DIR=4 (matches original wiring)
  // Module 3: STEP=30, DIR=28
  initModule(0, 3, 4);
  initModule(1, 8, 7);
  initModule(2, 30, 28);
  moduleCount = 3;

  // Bend servos start detached (no power) — attached on demand by BEND command
  pinMode(BEND_SERVO_A_PIN, OUTPUT);
  pinMode(BEND_SERVO_B_PIN, OUTPUT);
  pinMode(BEND_SERVO_C_PIN, OUTPUT);
  pinMode(BEND_SERVO_D_PIN, OUTPUT);
  digitalWrite(BEND_SERVO_A_PIN, LOW);
  digitalWrite(BEND_SERVO_B_PIN, LOW);
  digitalWrite(BEND_SERVO_C_PIN, LOW);
  digitalWrite(BEND_SERVO_D_PIN, LOW);

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
  int dirSign = (dir == DIR_CW) ? 1 : -1;

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

  digitalWrite(m.dirPin, dir);
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
    pidLimitWarned[modIdx] = false;
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

  // === Bend (XY plane servos) ===
  // BEND,<int>  where int is -100..100. 0 = center.
  // Legacy: BEND,LEFT | BEND,RIGHT | BEND,CENTER
  else if (command == "BEND") {
    int v;
    if (args == "LEFT")       v = -100;
    else if (args == "RIGHT") v =  100;
    else if (args == "CENTER" || args.length() == 0) v = 0;
    else                      v = args.toInt();

    if (v < -100) v = -100;
    if (v >  100) v =  100;
    bendValue = v;

    // Only one servo powered at a time. A = LEFT, B = RIGHT.
    // Slider left (v<0) -> drive A, detach B. Slider right (v>0) -> drive B, detach A.
    int mag = v < 0 ? -v : v;
    int offset = (int)((long)mag * BEND_MAX_DEG / 100);
    int angleA = centerAngleA;
    int angleB = centerAngleB;

    if (v < 0) {
      if (bendServoB.attached()) bendServoB.detach();
      if (!bendServoA.attached()) bendServoA.attach(BEND_SERVO_A_PIN);
      angleA = constrain(centerAngleA + offset, 0, 180);
      bendServoA.write(angleA);
      lastAngleA = angleA;
    } else if (v > 0) {
      if (bendServoA.attached()) bendServoA.detach();
      if (!bendServoB.attached()) bendServoB.attach(BEND_SERVO_B_PIN);
      angleB = constrain(centerAngleB + offset, 0, 180);
      bendServoB.write(angleB);
      lastAngleB = angleB;
    } else {
      // Latch last commanded angles as new centers. Keep the active servo
      // attached and holding; do not detach on center.
      centerAngleA = lastAngleA;
      centerAngleB = lastAngleB;
      if (bendServoA.attached()) bendServoA.write(lastAngleA);
      if (bendServoB.attached()) bendServoB.write(lastAngleB);
      angleA = lastAngleA;
      angleB = lastAngleB;
    }

    Serial.print("BEND ");
    Serial.print(v);
    Serial.print(" A=");
    Serial.print(angleA);
    Serial.print(" B=");
    Serial.println(angleB);
  }

  // === Bend XZ plane (servos C/D) ===
  // BEND_XZ,<int>  where int is -100..100. 0 = center.
  else if (command == "BEND_XZ") {
    int v;
    if (args == "CENTER" || args.length() == 0) v = 0;
    else                                        v = args.toInt();

    if (v < -100) v = -100;
    if (v >  100) v =  100;
    bendValueXZ = v;

    int mag = v < 0 ? -v : v;
    int offset = (int)((long)mag * BEND_MAX_DEG / 100);
    int angleC = centerAngleC;
    int angleD = centerAngleD;

    if (v < 0) {
      if (bendServoD.attached()) bendServoD.detach();
      if (!bendServoC.attached()) bendServoC.attach(BEND_SERVO_C_PIN);
      angleC = constrain(centerAngleC + offset, 0, 180);
      bendServoC.write(angleC);
      lastAngleC = angleC;
    } else if (v > 0) {
      if (bendServoC.attached()) bendServoC.detach();
      if (!bendServoD.attached()) bendServoD.attach(BEND_SERVO_D_PIN);
      angleD = constrain(centerAngleD + offset, 0, 180);
      bendServoD.write(angleD);
      lastAngleD = angleD;
    } else {
      // Latch last commanded angles as new centers. Keep the active servo
      // attached and holding; do not detach on center.
      centerAngleC = lastAngleC;
      centerAngleD = lastAngleD;
      if (bendServoC.attached()) bendServoC.write(lastAngleC);
      if (bendServoD.attached()) bendServoD.write(lastAngleD);
      angleC = lastAngleC;
      angleD = lastAngleD;
    }

    Serial.print("BEND_XZ ");
    Serial.print(v);
    Serial.print(" C=");
    Serial.print(angleC);
    Serial.print(" D=");
    Serial.println(angleD);
  }

  // === Bend ALL servos to the same angle ===
  // BEND_ALL,<int>  where int is -100..100. 0 = center/detach.
  else if (command == "BEND_ALL") {
    int v;
    if (args == "CENTER" || args.length() == 0) v = 0;
    else                                        v = args.toInt();

    if (v < -100) v = -100;
    if (v >  100) v =  100;
    bendValueAll = v;

    int allAngle = BEND_CENTER_DEG;
    if (v == 0) {
      // Latch each servo's last commanded angle as its new center. Keep
      // whichever servos are attached holding their positions.
      centerAngleA = lastAngleA;
      centerAngleB = lastAngleB;
      centerAngleC = lastAngleC;
      centerAngleD = lastAngleD;
      if (bendServoA.attached()) bendServoA.write(lastAngleA);
      if (bendServoB.attached()) bendServoB.write(lastAngleB);
      if (bendServoC.attached()) bendServoC.write(lastAngleC);
      if (bendServoD.attached()) bendServoD.write(lastAngleD);
      allAngle = lastAngleA;
    } else {
      int mag = v < 0 ? -v : v;
      int offset = (int)((long)mag * BEND_MAX_DEG / 100);
      int signedOffset = (v > 0) ? offset : -offset;

      if (!bendServoA.attached()) bendServoA.attach(BEND_SERVO_A_PIN);
      if (!bendServoB.attached()) bendServoB.attach(BEND_SERVO_B_PIN);
      if (!bendServoC.attached()) bendServoC.attach(BEND_SERVO_C_PIN);
      if (!bendServoD.attached()) bendServoD.attach(BEND_SERVO_D_PIN);

      int aA = constrain(centerAngleA + signedOffset, 0, 180);
      int aB = constrain(centerAngleB + signedOffset, 0, 180);
      int aC = constrain(centerAngleC + signedOffset, 0, 180);
      int aD = constrain(centerAngleD + signedOffset, 0, 180);

      bendServoA.write(aA); lastAngleA = aA;
      bendServoB.write(aB); lastAngleB = aB;
      bendServoC.write(aC); lastAngleC = aC;
      bendServoD.write(aD); lastAngleD = aD;
      allAngle = aA;
    }

    Serial.print("BEND_ALL ");
    Serial.print(v);
    Serial.print(" angle=");
    Serial.println(allAngle);
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

  // === Calibration commands (operate on module 0) ===

  else if (command == "RESET_POS") {
    modules[0].stepPosition = 0;
    Serial.println("POS_RESET 0");
  }

  else if (command == "SET_MIN") {
    modules[0].minStepLimit = modules[0].stepPosition;
    modules[0].limitsActive = (modules[0].minStepLimit != -999999L || modules[0].maxStepLimit != 999999L);
    Serial.print("MIN_SET ");
    Serial.println(modules[0].minStepLimit);
  }

  else if (command == "SET_MAX") {
    modules[0].maxStepLimit = modules[0].stepPosition;
    modules[0].limitsActive = (modules[0].minStepLimit != -999999L || modules[0].maxStepLimit != 999999L);
    Serial.print("MAX_SET ");
    Serial.println(modules[0].maxStepLimit);
  }

  else if (command == "SET_LIMITS") {
    int commaPos = args.indexOf(',');
    if (commaPos != -1) {
      modules[0].minStepLimit = args.substring(0, commaPos).toInt();
      modules[0].maxStepLimit = args.substring(commaPos + 1).toInt();
      modules[0].limitsActive = true;
      Serial.print("LIMITS_SET ");
      Serial.print(modules[0].minStepLimit);
      Serial.print(",");
      Serial.println(modules[0].maxStepLimit);
    }
  }

  else if (command == "CLEAR_LIMITS") {
    modules[0].limitsActive = false;
    modules[0].minStepLimit = -999999L;
    modules[0].maxStepLimit = 999999L;
    Serial.println("LIMITS_CLEARED");
  }

  else if (command == "GET_POS") {
    Serial.print("POS ");
    Serial.println(modules[0].stepPosition);
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
      // Defensive: refuse to auto-step a module whose stepper limits have not
      // been calibrated. Experiments mode drives the rig via pressure (SET),
      // which enables PID here — without this guard, an uncalibrated module
      // could be driven past its physical wall. Burst/one-shot commands (e.g.
      // INFLATE/DEFLATE) bypass this loop and still work during calibration.
      if (!modules[i].limitsActive) {
        if (!pidLimitWarned[i]) {
          Serial.print("PID_BLOCKED_NO_LIMITS M");
          Serial.println(i + 1);
          pidLimitWarned[i] = true;
        }
        continue;
      }
      if (modules[i].stepsRemaining > 0) continue;  // mid-burst, wait

      float err = setpointHpaM[i] - currentPressureM[i];

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
