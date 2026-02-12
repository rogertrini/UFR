// -------- Pins (adjust to your wiring) --------
const int PIN_TSA_OK     = 3;   // HIGH = TSA active (through opto/level shift)
const int PIN_BRAKE      = 4;   // HIGH = brake pressed
const int PIN_START_BTN  = 2;   // Button to GND, INPUT_PULLUP (LOW = pressed)
const int PIN_BUZZER     = 8;   // Active buzzer negative lead (active-low)
const int PIN_READY_LED  = 13;  // Optional: shows Ready latched

// -------- Timing --------
const unsigned long R2D_MS       = 3000;  // Ready-to-Drive sound duration (3 s)
const unsigned long DEBOUNCE_MS  = 30;

// -------- State machine --------
enum State { IDLE_ARMED, R2D_SOUND, READY_LATCHED };
State state = IDLE_ARMED;

unsigned long soundStart = 0;
int lastBtnStable = HIGH;
unsigned long lastBtnChange = 0;

// Debounce just the manual start button (others usually come from clean logic)
int readButtonDebounced() {
  int raw = digitalRead(PIN_START_BTN); b
  if (raw != lastBtnStable) {
    if (millis() - lastBtnChange >= DEBOUNCE_MS) {
      lastBtnStable = raw;
      lastBtnChange = millis();
    }
  } else {
    lastBtnChange = millis();
  }
  return lastBtnStable; // HIGH = released, LOW = pressed
}

inline bool tsaOK()   { return digitalRead(PIN_TSA_OK) == HIGH; }
inline bool brakeOn() { return digitalRead(PIN_BRAKE)  == HIGH; }

// Active buzzer control (active-low)
void buzzerOn()  { digitalWrite(PIN_BUZZER, LOW);  }
void buzzerOff() { digitalWrite(PIN_BUZZER, HIGH); }

void safeResetToArmed() {
  buzzerOff();
  state = IDLE_ARMED;
  digitalWrite(PIN_READY_LED, LOW);
}

void setup() {
  pinMode(PIN_TSA_OK, INPUT);           // through proper interface!
  pinMode(PIN_BRAKE, INPUT);            // through proper interface!
  pinMode(PIN_START_BTN, INPUT_PULLUP); // one side to pin, other to GND

  pinMode(PIN_BUZZER, OUTPUT);
  buzzerOff();                          // off at boot (active-low)

  pinMode(PIN_READY_LED, OUTPUT);
  digitalWrite(PIN_READY_LED, LOW);
}

void loop() {
  // Precondition check (hardware should ALSO enforce this via relay)
  bool preconditions = tsaOK() && brakeOn();

  // If preconditions drop at any time, force safe reset
  if (!preconditions) {
    if (state != IDLE_ARMED) safeResetToArmed();
  }

  int btn = readButtonDebounced(); // LOW = pressed

  switch (state) {
    case IDLE_ARMED:
      // Only accept manual action when BOTH TSA & BRAKE are valid
      if (preconditions && btn == LOW) {
        // Edge detect: require press from released state
        static bool wasReleased = true;
        if (wasReleased) {
          buzzerOn();
          soundStart = millis();
          state = R2D_SOUND;
        }
        wasReleased = false;
      } else if (btn == HIGH) {
        // track release
        static bool wasReleased = true;
        wasReleased = true;
      }
      break;

    case R2D_SOUND:
      if (!preconditions) {
        safeResetToArmed();           // safety first
      } else if (millis() - soundStart >= R2D_MS) {
        buzzerOff();
        state = READY_LATCHED;
        digitalWrite(PIN_READY_LED, HIGH);
      }
      break;

    case READY_LATCHED:
      // Stay READY while preconditions hold; drop immediately if they don’t
      if (!preconditions) {
        safeResetToArmed();
      }
      // Optional: require releasing the button before another cycle after drop
      break;
  }
}
