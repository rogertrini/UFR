/*
 * demo: CAN-BUS Shield, receive all frames and print all fields id/type/data
 * to receive frame fastly, a poll in loop() is required.
 *
 * Copyright (C) 2020 Seeed Technology Co.,Ltd.
 */
#include <SPI.h>

#define CAN_2515
// #define CAN_2518FD

// Set SPI CS Pin according to your hardware

#if defined(SEEED_WIO_TERMINAL) && defined(CAN_2518FD)
// For Wio Terminal w/ MCP2518FD RPi Hat：
// Channel 0 SPI_CS Pin: BCM 8
// Channel 1 SPI_CS Pin: BCM 7
// Interupt Pin: BCM25
const int SPI_CS_PIN  = BCM8;
const int CAN_INT_PIN = BCM25;
#else

// For Arduino MCP2515 Hat:
// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;
const int CAN_INT_PIN = 2;
#endif


#ifdef CAN_2518FD
#include "mcp2518fd_can.h"
mcp2518fd CAN(SPI_CS_PIN); // Set CS pin

// To TEST MCP2518FD CAN2.0 data transfer
#define MAX_DATA_SIZE 8
// To TEST MCP2518FD CANFD data transfer, uncomment below lines

// #undef  MAX_DATA_SIZE
// #define MAX_DATA_SIZE 64

#endif

#ifdef CAN_2515
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
#define MAX_DATA_SIZE 8
#endif

void setup() {
    SERIAL_PORT_MONITOR.begin(115200);
    while (!SERIAL_PORT_MONITOR) {}

    #if MAX_DATA_SIZE > 8
    /*
     * To compatible with MCP2515 API,
     * default mode is CAN_CLASSIC_MODE
     * Now set to CANFD mode.
     */
    CAN.setMode(CAN_NORMAL_MODE);
    #endif

    while (CAN_OK != CAN.begin(CAN_250KBPS)) {             // init can bus : baudrate = 250k
        SERIAL_PORT_MONITOR.println(F("CAN init fail, retry..."));
        delay(100);
    }
    SERIAL_PORT_MONITOR.println(F("CAN init ok!"));
}

uint32_t id;
uint8_t  type; // bit0: ext, bit1: rtr
uint8_t  len;

byte cdata[MAX_DATA_SIZE] = {0};

// ---------------------------------------------------------------------------
// Decode CAN identifier to human-readable form.  Look up the list of IDs in
// the 0A-0163-04_SW_User_Manual.pdf you supplied and fill in the cases below.
// The values shown are examples; replace them with the real identifiers from
// the manual.
// ---------------------------------------------------------------------------
const char *interpretCanId(uint32_t canId) {
    switch (canId) {

        // receive message IDs A 
        case 0x0A0: return "ID 0x0A0: Temperatures #1 ";
        case 0x0A1: return "ID 0x0A1: Temperatures #2";
        case 0x0A2: return "ID 0x0A2: Temperatures #3 & Torque Shudder";
        case 0x0A3: return "ID 0x0A3: Analog Inputs Voltages";
        case 0x0A4: return "ID 0x0A4: Digital Input Status";
        case 0x0A5: return "ID 0x0A5: Motor Position Information"; // this
        case 0x0A6: return "ID 0x0A6: Current Information";
        case 0x0A7: return "ID 0x0A7: Voltage Information";
        case 0x0A8: return "ID 0x0A8: Flux Information";
        case 0x0A9: return "ID 0x0A9: Internal Voltages";
        
        //receive message IDs A hex
        case 0x0AA: return "ID 0x0AA: Internal States"; // this
        case 0x0AB: return "ID 0x0AB: Fault Codes"; // this
        case 0x0AC: return "ID 0x0AC: Torque & Timer Information"; // this
        case 0x0AD: return "ID 0x0AD: Modulation Index & Flux Weakening Output Info";
        case 0x0AE: return "ID 0x0AE: Firmware Info";
        case 0x0AF: return "ID 0x0AF: Diagnostic Data";// this
        
        //Receive message IDs B
        case 0x0B0: return "ID 0x0B0: High Speed Message (transmitted at 3 ms)";
        case 0x0B1: return "ID 0x0B1: Torque Capability";    
        
        // transmit message IDs
        case 0x0C0: return "ID 0x0C0: yet another field";
        case 0x0C1: return "ID 0x0C1: yet another field";// ... 
        case 0x0C2: return "ID 0x0C2: yet another field";// ... 

        default:    return "Unknown ID";
    }
}


void loop() {
    // check if data coming
    if (CAN_MSGAVAIL != CAN.checkReceive()) {

            SERIAL_PORT_MONITOR.println(F("Not receive!"));

        return;
    }

    char prbuf[32 + MAX_DATA_SIZE * 3]; // 56 byte char buffer
    int i, n;

    unsigned long t = millis();
    // read data, len: data length, buf: data buf
    CAN.readMsgBuf(&len, cdata);

    id = CAN.getCanId();
    type = (CAN.isExtendedFrame() << 0) |
           (CAN.isRemoteRequest() << 1);
    /*
     * MCP2515(or this driver) could not handle properly
     * the data carried by remote frame
     */

    n = sprintf(prbuf, "%04lu.%03d ", t / 1000, int(t % 1000));
    /* Displayed type:
     *
     * 0x00: standard data frame
     * 0x02: extended data frame
     * 0x30: standard remote frame
     * 0x32: extended remote frame
     */
    static const byte type2[] = {0x00, 0x02, 0x30, 0x32};
    n += sprintf(prbuf + n, "RX: [%08lX](%02X) ", (unsigned long)id, type2[type]);
    
    // // ID description from manual
    n += sprintf(prbuf + n, "<%s> ", interpretCanId(id));
    
    n += sprintf(prbuf, "RX: [%08lX](%02X) ", id, type);

    for (i = 0; i < len; i++) {
        n += sprintf(prbuf + n, "%02X ", cdata[i]);
    }
    SERIAL_PORT_MONITOR.println(prbuf);
}

// 0x0AA Internal States 
void decodeInternalStates(uint8_t *data)
{
    // Byte 6
    bool bmsRegenLimit     = data[6] & (1 << 2);
    bool motorTempDerate   = data[6] & (1 << 4);
    bool motorHotSpot      = data[6] & (1 << 5);
    bool keySwitchActive   = data[6] & (1 << 6);
    bool inverterLockout   = data[6] & (1 << 7);

    // Byte 7
    bool directionForward  = data[7] & (1 << 0);
    bool bmsActive         = data[7] & (1 << 1);
    bool bmsMotorLimit     = data[7] & (1 << 2);
    bool maxSpeedLimit     = data[7] & (1 << 3);
    bool inverterHotSpot   = data[7] & (1 << 4);
    bool lowSpeedLimit     = data[7] & (1 << 5);
    bool coolantDerate     = data[7] & (1 << 6);
    bool stallLimit        = data[7] & (1 << 7);

    SERIAL_PORT_MONITOR.println("========== 0x0AA Internal States ==========");

    // Direction
    SERIAL_PORT_MONITOR.print("Direction: ");
    SERIAL_PORT_MONITOR.println(directionForward ? "FORWARD" : "REVERSE");

    // BMS
    SERIAL_PORT_MONITOR.print("BMS Active: ");
    SERIAL_PORT_MONITOR.println(bmsActive ? "YES" : "NO");

    // Enable Status
    SERIAL_PORT_MONITOR.print("Inverter Lockout: ");
    SERIAL_PORT_MONITOR.println(inverterLockout ? "LOCKED" : "UNLOCKED");

    SERIAL_PORT_MONITOR.print("Key Switch Start: ");
    SERIAL_PORT_MONITOR.println(keySwitchActive ? "ACTIVE" : "NOT ACTIVE");

    SERIAL_PORT_MONITOR.println("----------- Torque Limiting Flags ----------");

    if (bmsMotorLimit)    SERIAL_PORT_MONITOR.println("BMS limiting motor torque");
    if (bmsRegenLimit)    SERIAL_PORT_MONITOR.println("BMS limiting regen torque");
    if (motorTempDerate)  SERIAL_PORT_MONITOR.println("Motor temperature derating active");
    if (motorHotSpot)     SERIAL_PORT_MONITOR.println("Motor hot spot limiting");
    if (inverterHotSpot)  SERIAL_PORT_MONITOR.println("Inverter hot spot limiting");
    if (maxSpeedLimit)    SERIAL_PORT_MONITOR.println("Max speed limiting active");
    if (lowSpeedLimit)    SERIAL_PORT_MONITOR.println("Low speed limiting active");
    if (coolantDerate)    SERIAL_PORT_MONITOR.println("Coolant temperature derating active");
    if (stallLimit)       SERIAL_PORT_MONITOR.println("Stall burst limiting active");

    if (!(bmsMotorLimit || bmsRegenLimit || motorTempDerate || motorHotSpot ||
          inverterHotSpot || maxSpeedLimit || lowSpeedLimit || coolantDerate || stallLimit))
    {
        SERIAL_PORT_MONITOR.println("No active torque limits");
    }

    SERIAL_PORT_MONITOR.println("============================================");
}

