/*
  Single pot test: reverse + deadzone + torque scaling + CAN bytes (little-endian)

  - Reads pot on A0
  - Optional reverse
  - Deadzone 10-15%
  - Maps to torque (Nm) using MAX_TORQUE_NM
  - Converts to torque command (Nm*10 integer)
  - Prints CAN bytes in LITTLE-ENDIAN: low first, high second
*/

const int POT_PIN = A0;

const int DEADZONE_PCT = 12;     // 10-15% typical
const int REVERSE_INPUT = 1;     // 1 = reverse, 0 = normal

const float MAX_TORQUE_NM = 80.0f;   // <-- set this to your desired max torque at 100% pedal
const float TORQUE_SCALE = 10.0f;    // Nm*10 format

int applyDeadzoneAndRescale(int v)
{
  v = constrain(v, 0, 1023);

  int dz = (DEADZONE_PCT * 1023) / 100;
  if (v <= dz) return 0;

  long num = (long)(v - dz) * 1023L;
  long den = (1023 - dz);
  if (den <= 0) return 0;

  return constrain((int)(num / den), 0, 1023);
}

void setup()
{
  Serial.begin(115200);
  while(!Serial){}
  Serial.println("Pot test: reverse + deadzone + torque + CAN bytes (little-endian)");
}

void loop()
{
  int raw = analogRead(POT_PIN);

  int v = raw;
  if (REVERSE_INPUT) v = 1023 - raw;

  int throttle = applyDeadzoneAndRescale(v);          // 0..1023
  float pct = (throttle / 1023.0f) * 100.0f;          // 0..100

  // torque in Nm based on max torque
  float torqueNm = (pct / 100.0f) * MAX_TORQUE_NM;

  // command integer in Nm*10
  int torqueCmd = (int)(torqueNm * TORQUE_SCALE + 0.5f);

  // LITTLE-ENDIAN bytes
  byte low  = (byte)(torqueCmd & 0xFF);
  byte high = (byte)((torqueCmd >> 8) & 0xFF);

  Serial.print("raw=");
  Serial.print(raw);
  Serial.print(" rev=");
  Serial.print(v);
  Serial.print(" throttle=");
  Serial.print(throttle);
  Serial.print(" pct=");
  Serial.print(pct, 1);
  Serial.print("% torqueNm=");
  Serial.print(torqueNm, 1);
  Serial.print(" torqueCmd=");
  Serial.print(torqueCmd);

  Serial.print(" CAN(LE): [0]=0x");
  if (low < 16) Serial.print("0");
  Serial.print(low, HEX);

  Serial.print(" [1]=0x");
  if (high < 16) Serial.print("0");
  Serial.print(high, HEX);

  Serial.println();
  delay(100);
}
