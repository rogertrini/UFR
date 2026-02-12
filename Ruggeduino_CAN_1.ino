#include <SPI.h>
#include "mcp2515_can.h"

/* ====== USER SETTINGS ====== */
#define SPI_CS_PIN 9
#define CAN_BAUD CAN_500KBPS

#define TORQUE_CAN_ID 0x201     // CHANGE to your inverter torque ID
#define INVERTER_RX_ID 0x181    // CHANGE to inverter feedback ID

#define MAX_TORQUE_NM 50.0      // Adjust to your car
#define MIN_TORQUE_NM 0.0

#define THROTTLE_PIN A0

#define SEND_INTERVAL_MS 10     // 100 Hz command rate
#define INVERTER_TIMEOUT_MS 100 // safety timeout

/* ============================ */

mcp2515_can CAN(SPI_CS_PIN);

unsigned long lastSendTime = 0;
unsigned long lastInverterMsg = 0;

float commandedTorqueNm = 0;
bool inverterAlive = false;

/* ====== SETUP ====== */
void setup()
{
    Serial.begin(115200);

    while (CAN_OK != CAN.begin(CAN_BAUD))
    {
        Serial.println("CAN init fail...");
        delay(100);
    }

    Serial.println("CAN init success!");

    pinMode(THROTTLE_PIN, INPUT);
}

/* ====== MAIN LOOP ====== */
void loop()
{
    readCAN();
    checkTimeout();
    sendTorqueCommand();
}

/* ====== READ CAN MESSAGES ====== */
void readCAN()
{
    if (CAN.checkReceive() == CAN_MSGAVAIL)
    {
        unsigned char len = 0;
        unsigned char buf[8];

        CAN.readMsgBuf(&len, buf);
        long unsigned int canId = CAN.getCanId();

        if (canId == INVERTER_RX_ID)
        {
            lastInverterMsg = millis();
            inverterAlive = true;

            // Example: decode RPM (modify to your inverter spec)
            int rpm = buf[0] | (buf[1] << 8);

            Serial.print("RPM: ");
            Serial.println(rpm);
        }
    }
}

/* ====== CHECK SAFETY TIMEOUT ====== */
void checkTimeout()
{
    if (millis() - lastInverterMsg > INVERTER_TIMEOUT_MS)
    {
        inverterAlive = false;
        commandedTorqueNm = 0;
    }
}

/* ====== SEND TORQUE COMMAND ====== */
void sendTorqueCommand()
{
    if (millis() - lastSendTime < SEND_INTERVAL_MS)
        return;

    lastSendTime = millis();

    /* ----- Read Throttle ----- */
    int throttleRaw = analogRead(THROTTLE_PIN);

    // Calibrate these values!
    int throttleMin = 100;
    int throttleMax = 900;

    throttleRaw = constrain(throttleRaw, throttleMin, throttleMax);

    float percent =
        (float)(throttleRaw - throttleMin) /
        (float)(throttleMax - throttleMin);

    percent = constrain(percent, 0.0, 1.0);

    commandedTorqueNm =
        percent * (MAX_TORQUE_NM - MIN_TORQUE_NM);

    /* ----- Safety: if inverter not alive, zero torque ----- */
    if (!inverterAlive)
        commandedTorqueNm = 0;

    /* ----- Convert to inverter format ----- */
    int torqueScaled = commandedTorqueNm * 10;  // Nm ×10

    byte data[8] = {0};

    data[0] = torqueScaled & 0xFF;          // LSB
    data[1] = (torqueScaled >> 8) & 0xFF;   // MSB

    /* ----- Send CAN ----- */
    CAN.sendMsgBuf(TORQUE_CAN_ID, 0, 8, data);

    /* ----- Debug ----- */
    Serial.print("Torque Nm: ");
    Serial.println(commandedTorqueNm);
}
