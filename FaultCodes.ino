/* =============================================================================
 *  FaultCodes.ino  --  Cascadia Motion / Rinehart (RMS) fault decoder
 *
 *  Decodes the Fault Codes message (default ID 0x0AB) into human-readable
 *  names and prints them to Serial. This is a SECOND TAB of InverterControl;
 *  Arduino compiles all .ino files in the sketch folder together, so
 *  decodeFaults() is callable from the main file.
 *
 *  0x0AB frame layout (8 bytes, little-endian 16-bit words):
 *     [0..1] POST Fault Lo     [2..3] POST Fault Hi
 *     [4..5] Run  Fault Lo     [6..7] Run  Fault Hi
 *
 *  POST faults = power-on self test (latched at boot, often config/wiring).
 *  Run  faults = operational faults that occurred while running.
 *
 *  Bit names use PROGMEM so the 64 strings live in flash, not SRAM.
 *  >>> Confirm bit positions against YOUR Cascadia CAN protocol PDF. <<<
 * ========================================================================== */

#include <avr/pgmspace.h>

/* ---- POST Fault Lo (bytes 0-1) ----------------------------------------- */
const char pl0[]  PROGMEM = "HW Gate/Desaturation";
const char pl1[]  PROGMEM = "HW Overcurrent";
const char pl2[]  PROGMEM = "Accelerator Shorted";
const char pl3[]  PROGMEM = "Accelerator Open";
const char pl4[]  PROGMEM = "Current Sensor Low";
const char pl5[]  PROGMEM = "Current Sensor High";
const char pl6[]  PROGMEM = "Module Temp Low";
const char pl7[]  PROGMEM = "Module Temp High";
const char pl8[]  PROGMEM = "Ctrl PCB Temp Low";
const char pl9[]  PROGMEM = "Ctrl PCB Temp High";
const char pl10[] PROGMEM = "Gate Drv PCB Temp Low";
const char pl11[] PROGMEM = "Gate Drv PCB Temp High";
const char pl12[] PROGMEM = "5V Sense Low";
const char pl13[] PROGMEM = "5V Sense High";
const char pl14[] PROGMEM = "12V Sense Low";
const char pl15[] PROGMEM = "12V Sense High";
const char* const POST_LO[] PROGMEM = {
  pl0,pl1,pl2,pl3,pl4,pl5,pl6,pl7,pl8,pl9,pl10,pl11,pl12,pl13,pl14,pl15 };

/* ---- POST Fault Hi (bytes 2-3) ----------------------------------------- */
const char ph0[]  PROGMEM = "2.5V Sense Low";
const char ph1[]  PROGMEM = "2.5V Sense High";
const char ph2[]  PROGMEM = "1.5V Sense Low";
const char ph3[]  PROGMEM = "1.5V Sense High";
const char ph4[]  PROGMEM = "DC Bus Voltage High";
const char ph5[]  PROGMEM = "DC Bus Voltage Low";
const char ph6[]  PROGMEM = "Precharge Timeout";
const char ph7[]  PROGMEM = "Precharge Voltage Fail";
const char ph8[]  PROGMEM = "EEPROM Checksum Invalid";
const char ph9[]  PROGMEM = "EEPROM Data Out Of Range";
const char ph10[] PROGMEM = "EEPROM Update Required";
const char ph11[] PROGMEM = "Reserved (POSThi 11)";
const char ph12[] PROGMEM = "Reserved (POSThi 12)";
const char ph13[] PROGMEM = "Reserved (POSThi 13)";
const char ph14[] PROGMEM = "Brake Shorted";
const char ph15[] PROGMEM = "Brake Open";
const char* const POST_HI[] PROGMEM = {
  ph0,ph1,ph2,ph3,ph4,ph5,ph6,ph7,ph8,ph9,ph10,ph11,ph12,ph13,ph14,ph15 };

/* ---- Run Fault Lo (bytes 4-5) ------------------------------------------ */
const char rl0[]  PROGMEM = "Motor Overspeed";
const char rl1[]  PROGMEM = "Overcurrent";
const char rl2[]  PROGMEM = "Overvoltage";
const char rl3[]  PROGMEM = "Inverter Overtemp";
const char rl4[]  PROGMEM = "Accelerator Shorted";
const char rl5[]  PROGMEM = "Accelerator Open";
const char rl6[]  PROGMEM = "Direction Command Fault";
const char rl7[]  PROGMEM = "Inverter Response Timeout";
const char rl8[]  PROGMEM = "HW Gate/Desaturation";
const char rl9[]  PROGMEM = "HW Overcurrent";
const char rl10[] PROGMEM = "Undervoltage";
const char rl11[] PROGMEM = "CAN Command Msg Lost";
const char rl12[] PROGMEM = "Motor Overtemp";
const char rl13[] PROGMEM = "Reserved (Runlo 13)";
const char rl14[] PROGMEM = "Reserved (Runlo 14)";
const char rl15[] PROGMEM = "Reserved (Runlo 15)";
const char* const RUN_LO[] PROGMEM = {
  rl0,rl1,rl2,rl3,rl4,rl5,rl6,rl7,rl8,rl9,rl10,rl11,rl12,rl13,rl14,rl15 };

/* ---- Run Fault Hi (bytes 6-7) ------------------------------------------ */
const char rh0[]  PROGMEM = "Brake Shorted";
const char rh1[]  PROGMEM = "Brake Open";
const char rh2[]  PROGMEM = "Module A Overtemp";
const char rh3[]  PROGMEM = "Module B Overtemp";
const char rh4[]  PROGMEM = "Module C Overtemp";
const char rh5[]  PROGMEM = "PCB Overtemp";
const char rh6[]  PROGMEM = "Gate Drv 1 Overtemp";
const char rh7[]  PROGMEM = "Gate Drv 2 Overtemp";
const char rh8[]  PROGMEM = "Gate Drv 3 Overtemp";
const char rh9[]  PROGMEM = "Current Sensor Fault";
const char rh10[] PROGMEM = "Reserved (Runhi 10)";
const char rh11[] PROGMEM = "Reserved (Runhi 11)";
const char rh12[] PROGMEM = "HW Overvoltage";
const char rh13[] PROGMEM = "Reserved (Runhi 13)";
const char rh14[] PROGMEM = "Reserved (Runhi 14)";
const char rh15[] PROGMEM = "Resolver Not Connected";
const char* const RUN_HI[] PROGMEM = {
  rh0,rh1,rh2,rh3,rh4,rh5,rh6,rh7,rh8,rh9,rh10,rh11,rh12,rh13,rh14,rh15 };

/* =============================================================================
 *  printWord()
 *  Prints every set bit of one 16-bit fault word using its name table.
 *  'table' is a PROGMEM array of PROGMEM strings.
 * ========================================================================== */
static void printWord(const char *label, unsigned int word,
                      const char* const *table) {
  if (word == 0) return;
  char name[28];
  for (byte b = 0; b < 16; b++) {
    if (word & (1U << b)) {
      /* read the string pointer from flash, then the string itself */
      strcpy_P(name, (PGM_P)pgm_read_ptr(&table[b]));
      Serial.print(F("  ["));   Serial.print(label);
      Serial.print(F(" b"));    Serial.print(b);
      Serial.print(F("] "));    Serial.println(name);
    }
  }
}

/* =============================================================================
 *  decodeFaults()
 *  Pass the raw 8-byte 0x0AB frame. Edge-triggered: only prints when the
 *  fault set CHANGES (including clearing to none), so it never floods Serial.
 *  Returns true if any fault bit is currently set.
 * ========================================================================== */
bool decodeFaults(byte *f) {
  static byte prev[8] = {0};
  static bool first = true;

  bool changed = first;
  for (byte i = 0; i < 8; i++) if (f[i] != prev[i]) changed = true;
  first = false;

  unsigned int postLo = f[0] | (f[1] << 8);
  unsigned int postHi = f[2] | (f[3] << 8);
  unsigned int runLo  = f[4] | (f[5] << 8);
  unsigned int runHi  = f[6] | (f[7] << 8);
  bool any = (postLo | postHi | runLo | runHi) != 0;

  if (changed) {
    for (byte i = 0; i < 8; i++) prev[i] = f[i];
    Serial.println(F("==== INVERTER FAULTS ===="));
    if (!any) {
      Serial.println(F("  (none)"));
    } else {
      printWord("POSTlo", postLo, POST_LO);
      printWord("POSThi", postHi, POST_HI);
      printWord("RUNlo",  runLo,  RUN_LO);
      printWord("RUNhi",  runHi,  RUN_HI);
    }
    Serial.println(F("========================="));
  }
  return any;
}
