# FSAE EV — Inverter Torque & Regen Control

Accelerator (APPS) torque control + regenerative braking for a **Cascadia Motion /
Rinehart (RMS)** inverter, running on an **Arduino Mega 2560 / Rugged MEGA** with a
**Seeed CAN-BUS Shield V2.0** (MCP2515 + MCP2551).

> ⚠️ **300V+ tractive system.** Car on jack stands, HV PPE, fire extinguisher, and a
> working hardware shutdown loop before any powered test. The Arduino is **not** a
> safety device — see the safety note below.

---

## Which program do I upload?

There are **two independent sketches** (each folder = one program). You flash **one** at a time.

| Folder | Purpose | When to use |
|--------|---------|-------------|
| **`InverterControl/`** | **The real car program.** Full state machine, APPS, regen, blending, faults, pre-charge. | Normal use — **this is the one you run on the car.** |
| **`Bringup/`** | Bench bring-up. Enables the inverter and commands **0.0 Nm only** (can never make torque). | First-ever power-on checks only. |

**To run the car program:** open **`InverterControl/InverterControl.ino`** in the Arduino
IDE → its sibling tabs load automatically → select board **Arduino Mega 2560** → **Upload**.

---

## How the files fit together

Arduino concatenates **every `.ino` file in a sketch folder into one program** before
compiling (main tab first, then the others alphabetically). They are *not* separate
modules — there is no `#include` between them. This means:

- Exactly **one `setup()` and one `loop()`**, both in `InverterControl.ino`.
- Other tabs are just function libraries the main tab calls directly.
- The splitting is purely for readability.

### `InverterControl/` tabs

| File | Owns |
|------|------|
| **`InverterControl.ino`** | **Main tab.** `setup()`, `loop()`, state machine, APPS/regen/blending math, torque ramp, CAN heartbeat send, fault latch. Calls everything else. |
| `PrechargeControl.ino` | Drives AIR−/pre-charge/AIR+ contactor pins. `prechargeSetup/Update/Ready/Faulted()`. |
| `PowerMonitor.ino` | Decodes `0x0A6` (current) / `0x0A7` (voltage) → updates `dcBusVolts`/`dcBusAmps`, prints pre-charge progress. |
| `FaultCodes.ino` | Decodes `0x0AB` fault words into human-readable names. |
| `InverterStates.ino` | Decodes `0x0AA` → updates `invEnabled`/`invLockout`, prints VSM state. |

### Runtime flow (every loop)

```
loop()  [InverterControl.ino]
  ├─ pollCan()              read CAN; dispatch each ID to its decoder tab
  │     0x0AA → decodeInternalStates()   0x0AB → decodeFaults()
  │     0x0A6 → decodeCurrent()          0x0A7 → decodeVoltage()
  ├─ readApps()            accelerator %, plausibility
  ├─ computeTargetTorque() brake + blending + BPPC → target Nm
  ├─ prechargeUpdate()     step contactor sequencer (uses dcBusVolts)
  ├─ fault detection       windowed CAN timeout + latch
  ├─ state machine         INIT→…→RUNNING / FAULT
  ├─ rampTorque()          slew-limit toward target
  └─ sendCommand()         transmit 0x0C0 every 10 ms (heartbeat)
```

---

## Hardware / CAN summary

- **Controller:** Arduino Mega 2560 / Rugged MEGA
- **Shield:** Seeed CAN-BUS Shield V2.0 — **CS = D9**, **250 kbps**
- **Library:** [Seeed_Arduino_CAN](https://github.com/Seeed-Studio/Seeed_Arduino_CAN) **only**
- **Inverter base address:** `0x0A0`
- **Command frame `0x0C0`** (standard, 8 bytes, every 10 ms):
  - `[0..1]` torque, signed LE, 0.1 Nm   `[2..3]` speed (unused in torque mode)
  - `[4]` direction (1=fwd)   `[5]` bit0 enable, bit1 discharge, bit2 speed-mode
  - `[6..7]` torque limit, 0.1 Nm
- **Key RX:** `0x0A5` motor speed · `0x0A6` current · `0x0A7` DC bus V ·
  `0x0AA` internal state/lockout · `0x0AB` fault codes

### Pin map

| Pin | Use |
|-----|-----|
| D9 | CAN shield CS |
| D7 | Driver enable / RTD switch (active LOW, `INPUT_PULLUP`) |
| D4 / D5 / D6 | AIR− / pre-charge relay / AIR+ coil drivers |
| A0 / A1 | APPS 1 / APPS 2 |
| A2 | Brake pressure sensor |

---

## Safety layer (in `InverterControl`)

- APPS dual-sensor plausibility (10% / 100 ms), lower-channel fail-safe
- APPS + brake sensor out-of-range faults
- Brake/accelerator plausibility (BPPC, FSAE EV.5.7) with latch
- Regen anti-rollback (min RPM); never positive + negative torque at once
- Windowed CAN timeout (sustained loss, not single dropped frame)
- Pre-charge sequencer: ordered contactors, hard timeout fault, debounced bus-collapse
- Fault latch + single safe state; operator-acknowledged reset
- Clear-lockout startup, zero-torque-before-enable, 10 ms heartbeat
- `MAX_TORQUE_NM` starts at **1.0** on purpose

> **The pre-charge sequencer is a convenience layer, not a safety device.** Your hardware
> shutdown loop (BSPD/IMD/BOTS/e-stops) **must** open the contactors independently with
> the Mega off or hung. Drive contactor coils through proper drivers + flyback diodes,
> never directly from a pin.

---

## Recommended bring-up order

1. **`Bringup.ino`, LV / no HV** — confirm lockout clears, inverter reports enabled, heartbeat holds.
2. **`InverterControl.ino`, `REQUIRE_PRECHARGE = false`, wheels off ground** — confirm
   APPS %, plausibility trip, BPPC trip, faults latch. Still a 0 Nm path.
3. **Dry pre-charge test** (`REQUIRE_PRECHARGE = true`, no HV) — watch the contactor pins
   sequence on a meter/LEDs.
4. **HV on stands, `MAX_TORQUE_NM = 1.0`** — first motion; verify direction (byte 4).
5. Step torque up gradually; **enable regen last**.

---

## Must-verify against your Cascadia CAN protocol PDF

Firmware-specific assumptions that only you can confirm:

1. Command `byte 5` bit map, and whether your firmware needs a **rolling counter / CRC**
   (classic RMS doesn't; newer revisions do).
2. `0x0AA` enable/lockout bit positions and the inverter/discharge enum values.
3. Scaling (`0.1 V`/`0.1 A`/`0.1 Nm`) and byte order on `0x0A6`/`0x0A7`/`0x0C0`.
4. Pre-charge polarity (`RELAY_ACTIVE_HIGH`), `CONTROL_AIR_MINUS`, `PACK_NOMINAL_V`,
   `PRECHARGE_TIMEOUT_MS`.
5. APPS / brake sensor voltage calibration endpoints (from real measurements).

---

## Key config (top of `InverterControl/InverterControl.ino`)

```cpp
const float MAX_TORQUE_NM   = 1.0;    // START LOW, raise gradually
const float MAX_REGEN_NM    = 50.0;
const bool  REQUIRE_PRECHARGE = true; // false only for LV bench tests
const float PACK_NOMINAL_V  = 300.0;  // set to your accumulator nominal
```
