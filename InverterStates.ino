/* =============================================================================
 *  InverterStates.ino  --  Cascadia / Rinehart (RMS) Internal States decoder
 *
 *  Decodes the Internal States message (default ID 0x0AA) so you can see WHY
 *  the inverter is (or isn't) leaving lockout / making torque. Second tab of
 *  InverterControl; compiled together with the main file.
 *
 *  0x0AA frame layout (8 bytes):
 *     [0..1] VSM State (16-bit enum)
 *     [2]    Inverter State (enum)
 *     [3]    Relay State (bitfield, application-specific)
 *     [4]    bit0     = Inverter Run Mode (0 Torque / 1 Speed)
 *            bits5..7 = Inverter Active Discharge State (enum)
 *     [5]    bit0     = Inverter Command Mode (0 VSM / 1 CAN)
 *     [6]    bit0     = Inverter Enable State (1 = enabled)
 *            bit7     = Inverter Enable Lockout (1 = locked out)
 *     [7]    Direction Command (0 reverse / 1 forward)
 *
 *  >>> Confirm enum values + bit positions against YOUR CAN protocol PDF.
 *      The VSM-state list is well documented; the Inverter-state and
 *      discharge-state numeric values vary more across firmware. <<<
 * ========================================================================== */

#include <avr/pgmspace.h>

/* These globals live in InverterControl.ino; we update them here. */
extern bool invEnabled;
extern bool invLockout;

/* ---- VSM State enum (bytes 0-1) ---------------------------------------- */
const char vsm0[]  PROGMEM = "Start";
const char vsm1[]  PROGMEM = "Pre-charge Init";
const char vsm2[]  PROGMEM = "Pre-charge Active";
const char vsm3[]  PROGMEM = "Pre-charge Complete";
const char vsm4[]  PROGMEM = "Wait";
const char vsm5[]  PROGMEM = "Ready";
const char vsm6[]  PROGMEM = "Motor Running";
const char vsm7[]  PROGMEM = "Blink Fault Code";
const char vsm8[]  PROGMEM = "Reserved(8)";
const char vsm9[]  PROGMEM = "Reserved(9)";
const char vsm10[] PROGMEM = "Reserved(10)";
const char vsm11[] PROGMEM = "Reserved(11)";
const char vsm12[] PROGMEM = "Reserved(12)";
const char vsm13[] PROGMEM = "Reserved(13)";
const char vsm14[] PROGMEM = "Shutdown In Process";
const char vsm15[] PROGMEM = "Recycle Power";
const char* const VSM_TBL[] PROGMEM = {
  vsm0,vsm1,vsm2,vsm3,vsm4,vsm5,vsm6,vsm7,
  vsm8,vsm9,vsm10,vsm11,vsm12,vsm13,vsm14,vsm15 };

/* ---- Inverter State enum (byte 2) -------------------------------------- */
const char inv0[]  PROGMEM = "Power On";
const char inv1[]  PROGMEM = "Stop";
const char inv2[]  PROGMEM = "Open Loop";
const char inv3[]  PROGMEM = "Closed Loop";
const char inv4[]  PROGMEM = "Wait";
const char inv5[]  PROGMEM = "Reserved(5)";
const char inv6[]  PROGMEM = "Reserved(6)";
const char inv7[]  PROGMEM = "Reserved(7)";
const char inv8[]  PROGMEM = "Idle Run";
const char inv9[]  PROGMEM = "Idle Stop";
const char inv10[] PROGMEM = "Reserved(10)";
const char inv11[] PROGMEM = "Reserved(11)";
const char inv12[] PROGMEM = "Reserved(12)";
const char* const INV_TBL[] PROGMEM = {
  inv0,inv1,inv2,inv3,inv4,inv5,inv6,inv7,inv8,inv9,inv10,inv11,inv12 };

/* ---- Active Discharge State enum (byte 4, bits 5-7) -------------------- */
const char dis0[] PROGMEM = "Disabled";
const char dis1[] PROGMEM = "Enabled-Waiting";
const char dis2[] PROGMEM = "Speed Check";
const char dis3[] PROGMEM = "Active";
const char dis4[] PROGMEM = "Complete";
const char dis5[] PROGMEM = "Reserved(5)";
const char dis6[] PROGMEM = "Reserved(6)";
const char dis7[] PROGMEM = "Reserved(7)";
const char* const DIS_TBL[] PROGMEM = { dis0,dis1,dis2,dis3,dis4,dis5,dis6,dis7 };

/* printEnum() -- safely print a name from a PROGMEM table by index. */
static void printEnum(const char *label, byte idx, byte tblSize,
                      const char* const *tbl) {
  char name[24];
  Serial.print(label);
  if (idx < tblSize) {
    strcpy_P(name, (PGM_P)pgm_read_ptr(&tbl[idx]));
    Serial.print(name);
  } else {
    Serial.print(F("Unknown"));
  }
  Serial.print(F(" ("));  Serial.print(idx);  Serial.print(F(")"));
}

/* =============================================================================
 *  decodeInternalStates()
 *  Always updates invEnabled / invLockout (every frame). Prints the full
 *  decoded state only when something CHANGES, so Serial stays readable.
 * ========================================================================== */
void decodeInternalStates(byte *b) {
  /* --- fields --- */
  unsigned int vsm   = b[0] | (b[1] << 8);
  byte invState      = b[2];
  byte relayState    = b[3];
  byte runMode       = b[4] & 0x01;          // 0 torque / 1 speed
  byte discharge     = (b[4] >> 5) & 0x07;   // bits 5-7
  byte cmdMode       = b[5] & 0x01;          // 0 VSM / 1 CAN
  bool enableState   = (b[6] & 0x01) != 0;
  bool lockout       = (b[6] & 0x80) != 0;
  byte direction     = b[7];

  /* Always refresh the control-side flags. */
  invEnabled = enableState;
  invLockout = lockout;

  /* Edge-trigger on the bytes that matter for diagnostics. */
  static byte prev[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  bool changed = false;
  for (byte i = 0; i < 8; i++) if (b[i] != prev[i]) { changed = true; break; }
  if (!changed) return;
  for (byte i = 0; i < 8; i++) prev[i] = b[i];

  Serial.println(F("==== INVERTER STATE ===="));
  printEnum("  VSM: ", (byte)vsm, 16, VSM_TBL);  Serial.println();
  printEnum("  Inv: ", invState, 13, INV_TBL);   Serial.println();
  Serial.print(F("  Mode: ")); Serial.print(runMode ? F("Speed") : F("Torque"));
  Serial.print(F(" | Cmd src: ")); Serial.println(cmdMode ? F("CAN") : F("VSM"));
  printEnum("  Discharge: ", discharge, 8, DIS_TBL); Serial.println();
  Serial.print(F("  Enabled: ")); Serial.print(enableState ? F("YES") : F("no"));
  Serial.print(F(" | Lockout: ")); Serial.print(lockout ? F("YES") : F("no"));
  Serial.print(F(" | Dir: ")); Serial.println(direction ? F("FWD") : F("REV"));
  Serial.print(F("  Relays: 0x")); Serial.println(relayState, HEX);
  Serial.println(F("========================"));
}
