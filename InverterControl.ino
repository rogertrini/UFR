/* =============================================================================
 *  InverterControl.ino
 *  Formula SAE EV -- Accelerator + Regen torque control over CAN
 *  Cascadia Motion / Rinehart (RMS) inverter, Seeed CAN-BUS Shield V2.0
 *
 *  Flow:  APPS + brake sensors -> plausibility -> blending -> ramp -> CAN
 *
 *  >>> START HERE: MAX_TORQUE_NM is set to 1.0 Nm. Prove the car behaves,
 *      then raise it in small steps. Do NOT jump to full torque. <<<
 *
 *  SAFETY: jack stands, e-stop, HV PPE, no one near the driveline. 300V+.
 *
 *  LIBRARY: Seeed_Arduino_CAN only.
 * ========================================================================== */

#include <SPI.h>
#include "mcp2515_can.h"

/* =============================================================================
 *  SECTION 1 -- CONFIG CONSTANTS (tune these)
 * ========================================================================== */

/* ---- CAN / hardware ----------------------------------------------------- */
const int  SPI_CS_PIN = 9;
mcp2515_can CAN(SPI_CS_PIN);

const unsigned long CMD_ID            = 0x0C0;
const unsigned long ID_INTERNAL_STATE = 0x0AA;
const unsigned long ID_FAULT_CODES    = 0x0AB;
const unsigned long ID_MOTOR_POS      = 0x0A5;
const unsigned long ID_CURRENT        = 0x0A6;   // phase + DC bus current
const unsigned long ID_VOLTAGE        = 0x0A7;   // DC bus + output voltage

/* ---- Analog pins -------------------------------------------------------- */
const int APPS1_PIN = A0;     // accelerator sensor 1
const int APPS2_PIN = A1;     // accelerator sensor 2
const int BRAKE_PIN = A2;     // brake pressure sensor
const int ENABLE_SWITCH_PIN = 7;  // driver enable / RTD gate, active LOW

/* ---- ADC / electrical --------------------------------------------------- */
const float ADC_VREF   = 5.0;     // Arduino Mega ADC reference (volts)
const int   ADC_COUNTS = 1023;    // 10-bit ADC full scale

/* ---- Torque limits (the main "how fast does it go" knobs) --------------- */
const float MAX_TORQUE_NM  = 1.0;   // <<< START AT 1.0, raise slowly
const float MAX_REGEN_NM   = 50.0;  // magnitude of max regen (negative) torque
const float TORQUE_LIMIT_NM = 100.0;// commanded torque limit (bytes 6-7)

/* ---- Torque ramp (Nm per second) --------------------------------------- */
const float RAMP_UP_NM_PER_S   = 200.0;  // how fast torque may increase
const float RAMP_DOWN_NM_PER_S = 800.0;  // faster release = safer

/* ---- APPS calibration (CALIBRATE THESE on your pedal) ------------------ */
/* Voltages at fully released (0%) and fully pressed (100%) for each sensor. */
const float APPS1_V_MIN = 0.50, APPS1_V_MAX = 4.50;
const float APPS2_V_MIN = 0.50, APPS2_V_MAX = 4.50;

/* Pedal shaping */
const float APPS_DEADBAND_PCT = 3.0;   // below this -> 0% (mechanical slop)
const float APPS_FULL_PCT     = 98.0;  // above this -> 100%
const float APPS_FILTER_ALPHA = 0.20;  // EMA smoothing (0..1, lower=smoother)

/* ---- APPS plausibility (FSAE T.4/EV.5: 10% disagreement, 100 ms) ------- */
const float APPS_PLAUS_PCT_MAX = 10.0;    // max allowed |pct1 - pct2|
const unsigned long APPS_PLAUS_TIME_MS = 100;
/* Out-of-range guard rails (sensor short/open). Volts. */
const float APPS_V_FLOOR = 0.20, APPS_V_CEIL = 4.80;

/* ---- Brake pressure sensor (CALIBRATE) --------------------------------- */
/* Example: 0.5 V = 0 psi, 4.5 V = 1000 psi (a common 0.5-4.5V sensor). */
const float BRAKE_V_AT_0PSI   = 0.50;
const float BRAKE_PSI_PER_VOLT = 250.0;  // (psi at 4.5V - 0) / (4.5 - 0.5)
const float BRAKE_V_FLOOR = 0.20, BRAKE_V_CEIL = 4.80;  // range fault guards
const float BRAKE_ON_PSI  = 20.0;   // "brakes are being applied" threshold
const float BRAKE_HARD_PSI = 150.0; // "hard braking" for BPPC check

/* ---- Regen scaling (your table: 100psi=-10, 300=-30, 500=-50) ----------- */
const float REGEN_PSI_START = 0.0;    // regen begins above this pressure
const float REGEN_PSI_FULL  = 500.0;  // full MAX_REGEN_NM at/above this
const int   REGEN_MIN_RPM   = 500;    // no regen below this speed (anti-rollback)

/* ---- Blending / BPPC (FSAE EV.5.7) ------------------------------------- */
const float BPPC_APPS_TRIP_PCT  = 25.0;  // APPS >25% + hard brake -> cut power
const float BPPC_APPS_RESET_PCT = 5.0;   // power returns when APPS < 5%
const float REGEN_APPS_RELEASE_PCT = 5.0;// regen only when pedal ~released

/* ---- Pre-charge gate ---------------------------------------------------- */
/* The inverter must NOT be enabled until the DC bus has pre-charged up to
 * (close to) pack voltage. Pre-charge itself is done by your accumulator /
 * AIR + pre-charge resistor circuit; here we just watch the bus via 0x0A7
 * and refuse to leave DISABLED until it is high enough.                      */
const bool  REQUIRE_PRECHARGE   = true;     // set false ONLY for LV bench tests
const float PACK_NOMINAL_V      = 300.0;    // your accumulator nominal voltage
const float PRECHARGE_READY_V   = PACK_NOMINAL_V * 0.90; // ~90% = "ready"

/* ---- Timing ------------------------------------------------------------- */
const unsigned long CMD_PERIOD_MS   = 10;   // command heartbeat
const unsigned long PRINT_PERIOD_MS = 200;
const unsigned long RX_TIMEOUT_MS   = 200;  // "frames currently flowing?" window
/* CAN timeout is evaluated in fixed windows: a window with zero new frames
 * counts as 'missed'. We only fault after several CONSECUTIVE missed windows,
 * so a single scheduling hiccup or dropped frame can't false-trip.          */
const unsigned long RX_CHECK_PERIOD_MS   = 50;  // window length
const byte          RX_MAX_MISSED_WINDOWS = 4;  // 4 * 50ms = 200ms sustained loss

/* =============================================================================
 *  SECTION 2 -- STATE
 * ========================================================================== */
enum State {
  ST_INIT, ST_CLEAR_LOCKOUT, ST_DISABLED, ST_ENABLING, ST_RUNNING, ST_FAULT
};
State state = ST_INIT;

/* Fault latch: bitmask of why we faulted (for diagnostics). */
enum FaultBits {
  F_NONE        = 0,
  F_INVERTER    = 1 << 0,  // inverter reported a fault word
  F_APPS_PLAUS  = 1 << 1,  // two APPS disagree >10% for >100ms
  F_APPS_RANGE  = 1 << 2,  // APPS out of valid voltage range
  F_BRAKE_RANGE = 1 << 3,  // brake sensor out of range
  F_CAN_TIMEOUT = 1 << 4,  // lost inverter telemetry
  F_BPPC        = 1 << 5,  // accel+brake plausibility (EV.5.7)
  F_PRECHARGE   = 1 << 6   // pre-charge timed out / failed
};
int faultLatch = F_NONE;

/* Telemetry mirror */
bool  invEnabled = false, invLockout = true, invFault = false;
int   motorRpm = 0;
float dcBusVolts = 0;   // V   (from 0x0A7, updated in PowerMonitor.ino)
float dcBusAmps  = 0;   // A   (from 0x0A6)
bool  prechargeDone = false;

/* Filtered pedal state */
float apps1Pct = 0, apps2Pct = 0, appsPct = 0;
float brakePsi = 0;
bool  appsPlausBad = false;
unsigned long appsPlausStartMs = 0;
bool  bppcLatched = false;   // EV.5.7 latch, separate from main fault latch

/* Commanded torque (Nm) after ramp */
float torqueCmdNm = 0;

unsigned long lastCmdMs = 0, lastPrintMs = 0, lastRxMs = 0, stateMs = 0;
unsigned long lastLoopMs = 0;
unsigned long rxFrames = 0;   // total CAN frames received (for windowed timeout)

/* =============================================================================
 *  SECTION 3 -- HELPERS
 * ========================================================================== */

/* clampf() -- constrain a float to [lo, hi]. */
float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

/* adcToVolts() -- raw 10-bit ADC count to volts. */
float adcToVolts(int raw) {
  return ((float)raw * ADC_VREF) / (float)ADC_COUNTS;
}

/* voltsToPct() -- map a sensor voltage to 0..100% using its cal endpoints. */
float voltsToPct(float v, float vMin, float vMax) {
  float pct = (v - vMin) / (vMax - vMin) * 100.0;
  return clampf(pct, 0.0, 100.0);
}

/* shapePedal() -- apply deadband at bottom and saturation at top. */
float shapePedal(float pct) {
  if (pct <= APPS_DEADBAND_PCT) return 0.0;
  if (pct >= APPS_FULL_PCT)     return 100.0;
  /* rescale the usable band back to 0..100 so the driver gets full range */
  return (pct - APPS_DEADBAND_PCT) / (APPS_FULL_PCT - APPS_DEADBAND_PCT) * 100.0;
}

/* enterState() -- transition + timestamp. */
void enterState(State s) { state = s; stateMs = millis(); }

/* =============================================================================
 *  SECTION 4 -- ACCELERATOR (APPS)
 *  Reads both sensors, converts to %, filters, and runs FSAE plausibility.
 *  Sets appsPct (0..100) and raises F_APPS_* faults as needed.
 * ========================================================================== */
void readApps() {
  float v1 = adcToVolts(analogRead(APPS1_PIN));
  float v2 = adcToVolts(analogRead(APPS2_PIN));

  /* Out-of-range = broken/shorted sensor -> fault (FSAE EV.5.6). */
  if (v1 < APPS_V_FLOOR || v1 > APPS_V_CEIL ||
      v2 < APPS_V_FLOOR || v2 > APPS_V_CEIL) {
    faultLatch |= F_APPS_RANGE;
  }

  float p1 = shapePedal(voltsToPct(v1, APPS1_V_MIN, APPS1_V_MAX));
  float p2 = shapePedal(voltsToPct(v2, APPS2_V_MIN, APPS2_V_MAX));

  /* Exponential moving average filter on each channel. */
  apps1Pct += APPS_FILTER_ALPHA * (p1 - apps1Pct);
  apps2Pct += APPS_FILTER_ALPHA * (p2 - apps2Pct);

  /* FSAE plausibility: if the two channels disagree by >10% for >100 ms,
   * shut down power to the motor.                                            */
  if (fabs(apps1Pct - apps2Pct) > APPS_PLAUS_PCT_MAX) {
    if (!appsPlausBad) { appsPlausBad = true; appsPlausStartMs = millis(); }
    if (millis() - appsPlausStartMs > APPS_PLAUS_TIME_MS) {
      faultLatch |= F_APPS_PLAUS;
    }
  } else {
    appsPlausBad = false;
  }

  /* Use the LOWER of the two channels = fail-safe (never over-request). */
  appsPct = (apps1Pct < apps2Pct) ? apps1Pct : apps2Pct;
}

/* =============================================================================
 *  SECTION 5 -- BRAKE / REGEN
 *  Reads pressure, range-checks, and computes a regen torque request (>=0 Nm
 *  magnitude; sign is applied later in blending).
 * ========================================================================== */
float readBrakeAndRegen() {
  float v = adcToVolts(analogRead(BRAKE_PIN));

  if (v < BRAKE_V_FLOOR || v > BRAKE_V_CEIL) {
    faultLatch |= F_BRAKE_RANGE;
  }

  brakePsi = (v - BRAKE_V_AT_0PSI) * BRAKE_PSI_PER_VOLT;
  if (brakePsi < 0) brakePsi = 0;

  /* Map pressure to regen magnitude (linear between start and full). */
  if (brakePsi <= REGEN_PSI_START) return 0.0;
  float frac = (brakePsi - REGEN_PSI_START) /
               (REGEN_PSI_FULL - REGEN_PSI_START);
  frac = clampf(frac, 0.0, 1.0);
  return frac * MAX_REGEN_NM;   // positive magnitude
}

/* =============================================================================
 *  SECTION 6 -- BLENDING  (FSAE-appropriate)
 *  Returns the TARGET torque in Nm (signed): + = drive, - = regen.
 *
 *  Rules:
 *   - Accel only            -> positive torque scaled by pedal.
 *   - Brake only (released) -> negative regen torque (if fast enough).
 *   - Both                  -> NEVER positive+negative at once. Brake wins.
 *   - BPPC (EV.5.7): APPS >25% AND hard braking -> cut all positive torque,
 *     latch until APPS < 5%.
 * ========================================================================== */
float computeTargetTorque() {
  float accelNm = (appsPct / 100.0) * MAX_TORQUE_NM;   // >= 0
  float regenNm = readBrakeAndRegen();                 // >= 0 magnitude
  bool  braking = brakePsi >= BRAKE_ON_PSI;
  bool  hardBrake = brakePsi >= BRAKE_HARD_PSI;

  /* ---- BPPC latch (FSAE EV.5.7) ---------------------------------------- */
  if (hardBrake && appsPct > BPPC_APPS_TRIP_PCT) bppcLatched = true;
  if (appsPct < BPPC_APPS_RESET_PCT)             bppcLatched = false;
  if (bppcLatched) accelNm = 0.0;   // positive torque cut until pedal released

  /* ---- Anti-rollback: no regen at very low / zero speed ---------------- */
  if (motorRpm < REGEN_MIN_RPM) regenNm = 0.0;

  /* ---- Choose one sign, never both ------------------------------------- */
  if (braking) {
    /* While braking, suppress drive torque. Allow regen only when the
     * accelerator is essentially released (driver clearly wants to slow). */
    if (appsPct <= REGEN_APPS_RELEASE_PCT) return -regenNm;
    return 0.0;   // pedal still pressed while braking -> coast (0 Nm)
  }

  /* Not braking -> normal drive torque. */
  return accelNm;
}

/* =============================================================================
 *  SECTION 7 -- TORQUE RAMP
 *  Slew-rate limit the commanded torque toward the target. Keeps the
 *  driveline from getting step inputs and makes faults less violent.
 * ========================================================================== */
void rampTorque(float targetNm, float dtSec) {
  float upStep   = RAMP_UP_NM_PER_S   * dtSec;
  float downStep = RAMP_DOWN_NM_PER_S * dtSec;

  /* "toward zero" always uses the fast (down) rate; "away from zero" uses up. */
  if (targetNm > torqueCmdNm) {
    float step = (torqueCmdNm >= 0) ? upStep : downStep; // accel vs releasing regen
    torqueCmdNm += step;
    if (torqueCmdNm > targetNm) torqueCmdNm = targetNm;
  } else if (targetNm < torqueCmdNm) {
    float step = (torqueCmdNm <= 0) ? upStep : downStep; // regen vs releasing accel
    torqueCmdNm -= step;
    if (torqueCmdNm < targetNm) torqueCmdNm = targetNm;
  }

  /* Final hard clamp -- belt and suspenders. */
  torqueCmdNm = clampf(torqueCmdNm, -MAX_REGEN_NM, MAX_TORQUE_NM);
}

/* =============================================================================
 *  SECTION 8 -- CAN COMMAND
 * ========================================================================== */

/* nmToCmd() -- Nm (float) to signed int16 in 0.1 Nm units. */
int nmToCmd(float nm) {
  long v = (long)lround(nm * 10.0);
  if (v >  32767) v =  32767;
  if (v < -32768) v = -32768;
  return (int)v;
}

/* sendCommand() -- build + transmit the 8-byte command frame (Seeed API). */
void sendCommand(float torqueNm, bool enable) {
  int t   = nmToCmd(torqueNm);
  int lim = nmToCmd(TORQUE_LIMIT_NM);
  byte cmd[8];
  cmd[0] = (byte)(t & 0xFF);
  cmd[1] = (byte)((t >> 8) & 0xFF);
  cmd[2] = 0x00;                       // speed cmd low  (torque mode: unused)
  cmd[3] = 0x00;                       // speed cmd high
  cmd[4] = 0x01;                       // direction = forward
  cmd[5] = enable ? 0x01 : 0x00;       // bit0 enable; torque mode (bit2=0)
  cmd[6] = (byte)(lim & 0xFF);         // torque limit low
  cmd[7] = (byte)((lim >> 8) & 0xFF);  // torque limit high
  CAN.sendMsgBuf(CMD_ID, 0, 8, cmd);
}

/* =============================================================================
 *  SECTION 9 -- CAN RX
 * ========================================================================== */
void pollCan() {
  while (CAN.checkReceive() == CAN_MSGAVAIL) {
    byte len = 0, buf[8];
    CAN.readMsgBuf(&len, buf);
    unsigned long id = CAN.getCanId();
    lastRxMs = millis();
    rxFrames++;            // counted per window for the CAN-timeout check

    switch (id) {
      case ID_INTERNAL_STATE:
        /* decodeInternalStates() (InverterStates.ino tab) updates
         * invEnabled / invLockout and prints the decoded state on change. */
        decodeInternalStates(buf);
        break;
      case ID_FAULT_CODES:
        /* decodeFaults() (FaultCodes.ino tab) prints names on change and
         * returns whether any fault bit is set. */
        invFault = decodeFaults(buf);
        if (invFault) faultLatch |= F_INVERTER;
        break;
      case ID_MOTOR_POS:
        motorRpm = (int)((int16_t)(buf[2] | (buf[3] << 8)));
        break;
      case ID_CURRENT:
        decodeCurrent(buf);   // PowerMonitor.ino -> updates dcBusAmps
        break;
      case ID_VOLTAGE:
        decodeVoltage(buf);   // PowerMonitor.ino -> updates dcBusVolts, prints
        break;
      default: break;
    }
  }
}

/* =============================================================================
 *  SECTION 10 -- SETUP / LOOP
 * ========================================================================== */
void setup() {
  Serial.begin(115200);
  pinMode(ENABLE_SWITCH_PIN, INPUT_PULLUP);
  prechargeSetup();   // PrechargeControl.ino: contactor pins LOW (open) first

  while (CAN.begin(CAN_250KBPS) != CAN_OK) {
    Serial.println(F("CAN init FAIL - retrying..."));
    delay(200);
  }
  Serial.print(F("CAN OK. MAX_TORQUE_NM=")); Serial.println(MAX_TORQUE_NM);
  lastRxMs = millis();
  lastLoopMs = millis();
  enterState(ST_INIT);
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastLoopMs) / 1000.0;
  if (dt <= 0) dt = 0.001;
  lastLoopMs = now;

  /* ---- Inputs ----------------------------------------------------------- */
  pollCan();
  readApps();                         // sets appsPct, may set APPS faults
  float target = computeTargetTorque();// reads brake, runs blending + BPPC

  /* ---- Pre-charge contactor sequencing --------------------------------- */
  /* Want HV up whenever the driver switch is on and we are not faulted.
   * prechargeUpdate() drives AIR-/pre-charge/AIR+ and reports ready/fault. */
  bool wantHV = (state != ST_FAULT) && (digitalRead(ENABLE_SWITCH_PIN) == LOW);
  prechargeUpdate(wantHV);
  prechargeDone = prechargeReady();
  if (prechargeFaulted()) faultLatch |= F_PRECHARGE;

  /* ---- Fault detection (sets latch) ------------------------------------ */
  /* Windowed CAN-timeout: count windows with zero new frames; fault only
   * after RX_MAX_MISSED_WINDOWS in a row. One late/dropped frame resets it. */
  static unsigned long lastRxCheckMs = 0;
  static unsigned long framesAtLastCheck = 0;
  static byte missedWindows = 0;
  if (now - lastRxCheckMs >= RX_CHECK_PERIOD_MS) {
    lastRxCheckMs = now;
    if (rxFrames == framesAtLastCheck) {
      if (missedWindows < 255) missedWindows++;
    } else {
      missedWindows = 0;                 // got at least one frame -> healthy
    }
    framesAtLastCheck = rxFrames;
    if (missedWindows >= RX_MAX_MISSED_WINDOWS) faultLatch |= F_CAN_TIMEOUT;
  }
  if (faultLatch != F_NONE && state != ST_FAULT) {
    Serial.print(F("!! FAULT latch=0x")); Serial.println(faultLatch, HEX);
    enterState(ST_FAULT);
  }

  /* ---- State machine ---------------------------------------------------- */
  bool enable = false;
  float cmdTarget = 0.0;

  switch (state) {
    case ST_INIT:
      enterState(ST_CLEAR_LOCKOUT);
      break;

    case ST_CLEAR_LOCKOUT:                 // send enable=0 to clear lockout
      if (now - stateMs > 200) enterState(ST_DISABLED);
      break;

    case ST_DISABLED:
      /* Require: pre-charge complete (contactors closed, bus up), switch ON,
       * pedal released, no lockout, no fault. prechargeDone is set above. */
      if (prechargeDone && digitalRead(ENABLE_SWITCH_PIN) == LOW &&
          appsPct < 3.0 && !invLockout && faultLatch == F_NONE) {
        enterState(ST_ENABLING);
      }
      break;

    case ST_ENABLING:                      // enable=1, torque forced 0
      enable = true;
      if (invEnabled) { Serial.println(F("RUNNING")); enterState(ST_RUNNING); }
      else if (now - stateMs > 1000) enterState(ST_DISABLED);
      break;

    case ST_RUNNING:
      enable = true;
      cmdTarget = target;                  // live torque from blending
      if (digitalRead(ENABLE_SWITCH_PIN) != LOW) enterState(ST_DISABLED);
      break;

    case ST_FAULT:
      /* Safe state: enable off, zero torque. Recover only when fully safe. */
      torqueCmdNm = 0.0;
      if (digitalRead(ENABLE_SWITCH_PIN) != LOW && appsPct < 3.0 &&
          !invFault && (now - lastRxMs < RX_TIMEOUT_MS) && !appsPlausBad) {
        faultLatch = F_NONE;               // manual reset: switch off resets
        bppcLatched = false;
        enterState(ST_CLEAR_LOCKOUT);
      }
      break;
  }

  /* ---- Ramp toward the allowed target ---------------------------------- */
  rampTorque(enable ? cmdTarget : 0.0, dt);

  /* ---- Heartbeat: always transmit on a fixed period -------------------- */
  if (now - lastCmdMs >= CMD_PERIOD_MS) {
    lastCmdMs = now;
    sendCommand(enable ? torqueCmdNm : 0.0, enable);
  }

  /* ---- Telemetry -------------------------------------------------------- */
  if (now - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = now;
    Serial.print(F("st="));   Serial.print(state);
    Serial.print(F(" apps="));Serial.print(appsPct, 1);
    Serial.print(F(" psi=")); Serial.print(brakePsi, 0);
    Serial.print(F(" tgt=")); Serial.print(target, 1);
    Serial.print(F(" cmd=")); Serial.print(torqueCmdNm, 1);
    Serial.print(F(" rpm=")); Serial.print(motorRpm);
    Serial.print(F(" Vdc=")); Serial.print(dcBusVolts, 1);
    Serial.print(F(" Idc=")); Serial.print(dcBusAmps, 1);
    Serial.print(F(" pc="));  Serial.print(prechargeDone);
    Serial.print(F(" en="));  Serial.print(invEnabled);
    Serial.print(F(" flt=0x"));Serial.println(faultLatch, HEX);
  }
}
