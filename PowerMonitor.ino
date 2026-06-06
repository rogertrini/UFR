/* =============================================================================
 *  PowerMonitor.ino  --  Cascadia / Rinehart (RMS) voltage + current decoder
 *
 *  Lets you WATCH PRE-CHARGE happen: decodes DC bus voltage (0x0A7) and bus
 *  current (0x0A6), and prints pre-charge progress as the bus rises. Second
 *  tab of InverterControl; compiled together with the main file.
 *
 *  Current Information (0x0A6), all signed int16 little-endian, 0.1 A units:
 *     [0..1] Phase A   [2..3] Phase B   [4..5] Phase C   [6..7] DC Bus
 *
 *  Voltage Information (0x0A7), signed int16 little-endian, 0.1 V units:
 *     [0..1] DC Bus Voltage   [2..3] Output Voltage
 *     [4..5] VAB_Vd           [6..7] VBC_Vq
 *
 *  >>> Confirm scaling (0.1 units) + byte order against YOUR protocol PDF. <<<
 * ========================================================================== */

/* Globals owned by InverterControl.ino */
extern float dcBusVolts;
extern float dcBusAmps;
extern const float PRECHARGE_READY_V;
extern const float PACK_NOMINAL_V;

/* s16le() -- read a signed 16-bit little-endian value from two bytes. */
static int s16le(byte lo, byte hi) {
  return (int)((int16_t)(lo | (hi << 8)));
}

/* =============================================================================
 *  decodeCurrent()  -- parse 0x0A6, update dcBusAmps.
 *  Phase currents are parsed too but only the DC bus current is exported,
 *  since that is what you watch during pre-charge / load.
 * ========================================================================== */
void decodeCurrent(byte *b) {
  dcBusAmps = s16le(b[6], b[7]) * 0.1;
}

/* =============================================================================
 *  decodeVoltage() -- parse 0x0A7, update dcBusVolts, and report pre-charge.
 *
 *  Prints a progress line each time the bus crosses a 10% step of pack
 *  voltage (so you see it climb without flooding Serial), plus a one-shot
 *  "PRECHARGE COMPLETE" when it passes the ready threshold.
 * ========================================================================== */
void decodeVoltage(byte *b) {
  dcBusVolts = s16le(b[0], b[1]) * 0.1;

  /* progress in whole 10% steps of pack nominal */
  static int  lastStep = -1;
  static bool announcedReady = false;

  int step = (int)((dcBusVolts / PACK_NOMINAL_V) * 10.0);  // 0..10+
  if (step != lastStep) {
    lastStep = step;
    Serial.print(F("[precharge] Vdc="));
    Serial.print(dcBusVolts, 1);
    Serial.print(F("V ("));
    Serial.print((int)((dcBusVolts / PACK_NOMINAL_V) * 100.0));
    Serial.println(F("% of pack)"));
  }

  if (!announcedReady && dcBusVolts >= PRECHARGE_READY_V) {
    announcedReady = true;
    Serial.print(F(">>> PRECHARGE COMPLETE @ "));
    Serial.print(dcBusVolts, 1);
    Serial.println(F("V - safe to enable"));
  }
  /* re-arm the one-shot if the bus collapses (HV off / fault) */
  if (dcBusVolts < (PRECHARGE_READY_V * 0.5)) announcedReady = false;
}
