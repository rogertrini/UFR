/* =============================================================================
 *  PrechargeControl.ino  --  HV pre-charge + contactor (AIR) sequencer
 *
 *  Drives the negative contactor (AIR-), the pre-charge relay, and the
 *  positive/main contactor (AIR+) in the correct order, watching the DC bus
 *  voltage (from PowerMonitor.ino) to decide when pre-charge is complete.
 *
 *  Sequence when HV is requested:
 *     1. Close AIR-                      (optional: see CONTROL_AIR_MINUS)
 *     2. Close pre-charge relay          -> bus charges through resistor
 *     3. Wait until Vdc >= PRECHARGE_READY_V (or TIMEOUT -> fault)
 *     4. Close AIR+                      -> main path live
 *     5. Open pre-charge relay           -> resistor now bypassed
 *     -> DONE
 *  When HV is NOT requested (or on fault): open AIR+, pre-charge, AIR-.
 *
 *  ##########################################################################
 *  #  SAFETY -- READ THIS                                                    #
 *  #  This is a CONVENIENCE sequencer, NOT a safety device. Your hardware    #
 *  #  shutdown circuit (BSPD, IMD, BOTS, e-stops, interlocks) MUST be able   #
 *  #  to open these contactors INDEPENDENTLY of this Arduino, with the       #
 *  #  Mega powered off or hung. Wire the contactor coils so the shutdown     #
 *  #  loop removes coil power directly; this code only ADDS a condition,     #
 *  #  it must never be the ONLY thing holding a contactor closed.            #
 *  #                                                                         #
 *  #  Drive coils through proper drivers (relay/MOSFET + flyback diode),     #
 *  #  NOT directly from a 5V pin. Contactor coils draw far more than a pin   #
 *  #  can source and generate large inductive kick.                          #
 *  ##########################################################################
 * ========================================================================== */

/* Globals owned by InverterControl.ino */
extern float dcBusVolts;
extern const float PRECHARGE_READY_V;
extern const bool  REQUIRE_PRECHARGE;

/* ---- Output pins (choose free pins; avoid SPI 50-53 & CS 9) ------------- */
const int PIN_AIR_MINUS = 4;   // negative contactor coil driver
const int PIN_PRECHARGE = 5;   // pre-charge relay coil driver
const int PIN_AIR_PLUS  = 6;   // positive / main contactor coil driver

/* ---- Driver polarity ---------------------------------------------------- */
/* true  = driver/relay closes the contactor when the pin is HIGH.
 * Set to match YOUR coil driver hardware.                                   */
const bool RELAY_ACTIVE_HIGH = true;

/* ---- If AIR- is held closed by your shutdown circuit (common), set false
 *      and this code leaves PIN_AIR_MINUS alone / skips step 1.            */
const bool CONTROL_AIR_MINUS = true;

/* ---- Timing ------------------------------------------------------------- */
const unsigned long NEG_SETTLE_MS       = 100;   // let AIR- seat before precharge
const unsigned long MAIN_SETTLE_MS      = 100;   // let AIR+ seat before dropping PC
const unsigned long PRECHARGE_TIMEOUT_MS = 2000; // bus must reach ready in this time

/* ---- Bus-collapse detection (in PC_CLOSED) ----------------------------- */
/* Bus must read below the collapse level CONTINUOUSLY for this long before
 * we fault, so one noisy/dropped CAN voltage frame can't false-trip.        */
const float         COLLAPSE_FRAC       = 0.5;   // collapse if Vdc < 50% ready
const unsigned long COLLAPSE_DEBOUNCE_MS = 150;  // sustained time below level

/* ---- Pre-charge sub-state machine -------------------------------------- */
enum PcState {
  PC_OPEN,        // everything open (safe)
  PC_CLOSE_NEG,   // AIR- closed, settling
  PC_PRECHARGE,   // pre-charge relay closed, bus rising
  PC_CLOSE_MAIN,  // AIR+ closed, settling before dropping pre-charge
  PC_CLOSED,      // DONE: AIR+ closed, pre-charge open
  PC_FAULT        // timed out / failed; all open until request clears
};
static PcState pcState = PC_OPEN;
static unsigned long pcMs = 0;   // timestamp of last sub-state entry

/* setRelay() -- drive one coil pin respecting RELAY_ACTIVE_HIGH. */
static void setRelay(int pin, bool closed) {
  bool level = RELAY_ACTIVE_HIGH ? closed : !closed;
  digitalWrite(pin, level ? HIGH : LOW);
}

/* openAll() -- force every contactor/relay open (the safe state). */
static void openAll() {
  setRelay(PIN_AIR_PLUS,  false);
  setRelay(PIN_PRECHARGE, false);
  if (CONTROL_AIR_MINUS) setRelay(PIN_AIR_MINUS, false);
}

/* pcEnter() -- transition helper. */
static void pcEnter(PcState s) { pcState = s; pcMs = millis(); }

/* =============================================================================
 *  prechargeSetup()  -- call once from setup(). Pins to outputs, all OPEN.
 * ========================================================================== */
void prechargeSetup() {
  pinMode(PIN_AIR_MINUS, OUTPUT);
  pinMode(PIN_PRECHARGE, OUTPUT);
  pinMode(PIN_AIR_PLUS,  OUTPUT);
  openAll();
  pcEnter(PC_OPEN);
}

/* =============================================================================
 *  prechargeUpdate(want)  -- call every loop.
 *    want == true  -> drive the sequence toward CLOSED (HV up)
 *    want == false -> open everything and reset to OPEN
 *  Bus voltage comes from dcBusVolts (updated by PowerMonitor.ino).
 * ========================================================================== */
void prechargeUpdate(bool want) {
  /* If pre-charge control is disabled (LV bench), keep everything open and
   * do nothing else; prechargeReady() will report ready instead.            */
  if (!REQUIRE_PRECHARGE) { openAll(); return; }

  unsigned long now = millis();

  /* Drop-out: any time HV is not wanted, go safe (except let FAULT latch
   * stay visible until the request clears, which it does here too).         */
  if (!want && pcState != PC_OPEN) {
    openAll();
    pcEnter(PC_OPEN);
    Serial.println(F("[precharge] request cleared -> all open"));
    return;
  }

  switch (pcState) {
    case PC_OPEN:
      openAll();
      if (want) {
        if (CONTROL_AIR_MINUS) setRelay(PIN_AIR_MINUS, true);
        Serial.println(F("[precharge] AIR- closing"));
        pcEnter(PC_CLOSE_NEG);
      }
      break;

    case PC_CLOSE_NEG:
      if (now - pcMs >= NEG_SETTLE_MS) {
        setRelay(PIN_PRECHARGE, true);
        Serial.println(F("[precharge] pre-charge relay closed, bus rising"));
        pcEnter(PC_PRECHARGE);
      }
      break;

    case PC_PRECHARGE:
      if (dcBusVolts >= PRECHARGE_READY_V) {
        setRelay(PIN_AIR_PLUS, true);          // main contactor in
        Serial.println(F("[precharge] AIR+ closing"));
        pcEnter(PC_CLOSE_MAIN);
      } else if (now - pcMs >= PRECHARGE_TIMEOUT_MS) {
        openAll();                             // FAIL: never reached voltage
        Serial.println(F("[precharge] !! TIMEOUT -> FAULT, all open"));
        pcEnter(PC_FAULT);
      }
      break;

    case PC_CLOSE_MAIN:
      if (now - pcMs >= MAIN_SETTLE_MS) {
        setRelay(PIN_PRECHARGE, false);        // resistor now bypassed by AIR+
        Serial.println(F("[precharge] pre-charge relay open -> DONE"));
        pcEnter(PC_CLOSED);
      }
      break;

    case PC_CLOSED: {
      /* Hold. Sanity: if AIR+ is in but the bus collapses, fault out -- but
       * only after Vdc stays low for COLLAPSE_DEBOUNCE_MS, so a single noisy
       * or dropped voltage frame cannot false-trip.                         */
      static bool          collapsing = false;
      static unsigned long collapseSinceMs = 0;
      float collapseV = PRECHARGE_READY_V * COLLAPSE_FRAC;

      if (dcBusVolts < collapseV) {
        if (!collapsing) { collapsing = true; collapseSinceMs = now; }
        if (now - collapseSinceMs >= COLLAPSE_DEBOUNCE_MS) {
          openAll();
          Serial.println(F("[precharge] !! bus collapsed -> FAULT"));
          collapsing = false;
          pcEnter(PC_FAULT);
        }
      } else {
        collapsing = false;   // bus recovered before debounce elapsed
      }
      break;
    }

    case PC_FAULT:
      openAll();   // stay open; cleared only when 'want' goes false (above)
      break;
  }
}

/* prechargeReady()   -- true when the main contactor is in and bus is up. */
bool prechargeReady() {
  if (!REQUIRE_PRECHARGE) return true;   // bench mode: pretend ready
  return pcState == PC_CLOSED;
}

/* prechargeFaulted() -- true if the sequence timed out / failed. */
bool prechargeFaulted() {
  return pcState == PC_FAULT;
}
