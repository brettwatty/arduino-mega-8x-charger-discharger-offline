// Arduino Smart Charger / Discharger - Offline
// Version 1.0.0
// ---------------------------------------------------------------------------
// Created by Brett Watt on 12/08/2019
// Copyright 2018 - Under creative commons license 3.0:
//
// This software is furnished "as is", without technical support, and with no
// warranty, express or implied, as to its usefulness for any purpose.
//
// @brief
// This is the Arduino 8x 18650 Smart Charger / Discharger Code - Offline
//
// @author Email: info@vortexit.co.nz
//       Web: www.vortexit.co.nz

//Include Libraries
#include <Wire.h> // Comes with Arduino IDE
#include <OneWire.h>
#include <LiquidCrystal_I2C.h>
#include <DallasTemperature.h>
#include "Encoder_Polling.h"

#define TEMPERATURE_PRECISION 9
// #define ONE_WIRE_BUS 2 // Pin 2 Temperature Sensors - PCB Version 1.1
#define ONE_WIRE_BUS 4 // Pin 4 Temperature Sensors - PCB Version 2.+

// #define CLK 3 // Rotary Encoder PIN A CLK    - PCB Version 1.1
// #define DT 4  // Rotary Encoder PIN B DT     - PCB Version 1.1
#define CLK 6 // Rotary Encoder PIN A CLK       - PCB Version 2.+
#define DT 7  // Rotary Encoder PIN B DT        - PCB Version 2.+

//Objects
LiquidCrystal_I2C lcd(0x27, 20, 4);  // set the LCD address to 0x27 for a 20 chars and 4 line display
OneWire oneWire(ONE_WIRE_BUS);       // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature.

// Button pin
const byte BTN = 5;

typedef struct
{
    // Pin Definitions
    const uint8_t batteryVolatgePin;
    const uint8_t batteryVolatgeDropPin;
    const uint8_t chargeLedPin;
    const uint8_t chargeMosfetPin;
    const uint8_t dischargeMosfetPin;

    // Timmer
    unsigned long longMilliSecondsCleared;
    byte seconds;
    byte minutes;
    byte hours;

    // Module Cycle
    byte cycleCount;
    byte cycleState;
    byte batteryFaultCode;

    // Voltage Readings
    float batteryInitialVoltage;
    float batteryVoltage;
    float batteryLastVoltage;

    // Temp Readings an Variables
    byte batteryInitialTemp;
    byte batteryHighestTemp;
    byte batteryCurrentTemp;
    byte tempCount;

    // Milli Ohms
    float tempMilliOhmsValue;
    float milliOhmsValue;

    // Discharge Battery
    int intMilliSecondsCount;
    unsigned long longMilliSecondsPreviousCount;
    unsigned long longMilliSecondsPrevious;
    unsigned long longMilliSecondsPassed;
    float dischargeMilliamps;
    float dischargeVoltage;
    float dischargeAmps;
    bool dischargeCompleted;
    int dischargeMinutes;
    bool pendingDischargeRecord;
} Modules;

Modules module[8] =
    {
        {A0, A1, 23, 22, 24},
        {A2, A3, 26, 25, 27},
        {A4, A5, 29, 28, 30},
        {A6, A7, 32, 31, 33},
        {A8, A9, 35, 34, 36},
        {A10, A11, 38, 37, 39},
        {A12, A13, 41, 40, 42},
        {A14, A15, 44, 43, 45}};

typedef struct
{
    const float shuntResistor[8] = {3.3, 3.3, 3.3, 3.3, 3.3, 3.3, 3.3, 3.3};
    const bool useReferenceVoltage = true;         // "true" to use the 5v regulator as the refenence voltage or "false" to use the 1.1V internal voltage reference
    const float referenceVoltage = 4.97;           // 5V output of Arduino
    const float internalReferenceVoltage = 1.1;    // 1.1V internal voltage reference of Arduino
    const float defaultBatteryCutOffVoltage = 2.8; // Voltage that the discharge stops
    const byte restTimeMinutes = 1;                // The time in Minutes to rest the battery after charge. 0-59 are valid
    const int lowMilliamps = 1000;                 // This is the value of Milli Amps that is considered low and does not get recharged because it is considered faulty
    const int highMilliOhms = 500;                 // This is the value of Milli Ohms that is considered high and the battery is considered faulty
    const int offsetMilliOhms = 0;                 // Offset calibration for MilliOhms
    const byte chargingTimeout = 8;                // The timeout in Hours for charging
    const byte tempThreshold = 7;                  // Warning Threshold in degrees above initial Temperature
    const byte tempMaxThreshold = 10;              // Maximum Threshold in degrees above initial Temperature - Considered Faulty
    const float batteryVolatgeLeak = 2.00;         // On the initial screen "BATTERY CHECK" observe the highest voltage of each module and set this value slightly higher
    const byte modules = 8;                        // Number of Modules
    const byte screenTime = 4;                     // Time in Seconds (Cycles) per Active Screen
    const int dischargeReadInterval = 5000;        // Time intervals between Discharge readings. Adjust for mAh +/-
    const bool rechargeCycle = true;               // Run the Recharge Cycle "true" to run and "false" to skip to Completed Cycle
} CustomSettings;

CustomSettings settings;

DeviceAddress tempSensorSerial[9] = {
    {0x28, 0xFF, 0x32, 0xDC, 0x63, 0x16, 0x03, 0xB9},
    {0x28, 0x99, 0xD2, 0x79, 0x97, 0x04, 0x03, 0x54},
    {0x28, 0x33, 0x58, 0x79, 0x97, 0x04, 0x03, 0xB1},
    {0x28, 0xFF, 0x37, 0xA5, 0x31, 0x18, 0x01, 0x04},
    {0x28, 0xFF, 0x5F, 0x9D, 0x31, 0x18, 0x01, 0x7F},
    {0x28, 0xAC, 0xA3, 0x45, 0x92, 0x0C, 0x02, 0xB5},
    {0x28, 0xED, 0xA4, 0x45, 0x92, 0x03, 0x02, 0xBE},
    {0x28, 0x5A, 0xAB, 0x45, 0x92, 0x07, 0x02, 0x68},
    {0x28, 0xFF, 0x01, 0xDC, 0x63, 0x16, 0x03, 0x73}};

int ambientTemperature = 0;
boolean buttonPressed = false;

// Cycle Variables

byte cycleStateLast = 0;
byte rotaryOverride = 0;
boolean rotaryOverrideLock = 0;
boolean buttonState = 0;
boolean lastButtonState = 0;

void setup()
{
    pinMode(BTN, INPUT_PULLUP); // Pin 5 Rotary Encoder Button (BTN)
    encoder_begin(CLK, DT);     // Start the decoder encoder Pin A (CLK) = 3 encoder Pin B (DT) = 4 | Pin 7 (CLK) and Pin 6 (DT) for Version 2.0
    for (byte i = 0; i < 8; i++)
    {
        pinMode(module[i].chargeLedPin, INPUT_PULLUP);
        pinMode(module[i].chargeMosfetPin, OUTPUT);
        pinMode(module[i].dischargeMosfetPin, OUTPUT);
        digitalWrite(module[i].chargeMosfetPin, LOW);
        digitalWrite(module[i].dischargeMosfetPin, LOW);
    }

    char lcdStartup0[25];
    char lcdStartup1[25];
    char lcdStartup2[25];
    char lcdStartup3[25];
    //Startup Screen
    lcd.init();
    lcd.clear();
    lcd.backlight(); // Turn on backlight
    sprintf(lcdStartup0, "%-20s", "ASCD MEGA 8X 2.0.0 ");
    lcd.setCursor(0, 0);
    lcd.print(lcdStartup0);
    sprintf(lcdStartup1, "%-20s", "INITIALIZING TP4056");
    lcd.setCursor(0, 1);
    lcd.print(lcdStartup1);

    for (byte i = 0; i < 8; i++)
    {
        digitalWrite(module[i].chargeMosfetPin, LOW);
        delay(250);
        digitalWrite(module[i].chargeMosfetPin, HIGH);
        delay(250);
        digitalWrite(module[i].chargeMosfetPin, LOW);
        delay(250);
        digitalWrite(module[i].chargeMosfetPin, HIGH);
        delay(250);
        digitalWrite(module[i].chargeMosfetPin, LOW);
    }

    //Initialize Serial
    Serial.begin(115200);
    Serial.setTimeout(5);
    Serial.println("Starting...");
}

void loop()
{
    // Mills - Timmer
    static long rotaryEncoderMillis;
    static long cycleStateValuesMillis;
    long currentMillis = millis();
    if (currentMillis - rotaryEncoderMillis >= 2) // Every 2 millis
    {
        rotaryEncoder();
        rotaryEncoderMillis = currentMillis;
    }
    currentMillis = millis();
    if (currentMillis - cycleStateValuesMillis >= 1000) // Every 1 second
    {
        cycleStateValues();
        cycleStateValuesMillis = currentMillis;
    }
}

void rotaryEncoder()
{
    int rotaryEncoderDirection = encoder_data(); // Check for rotation
    if (rotaryEncoderDirection != 0)             // Rotary Encoder
    {
        if (cycleStateLast - rotaryEncoderDirection >= 0 && cycleStateLast - rotaryEncoderDirection < settings.modules)
        {
            cycleStateLast = cycleStateLast - rotaryEncoderDirection;
        }
        else if (cycleStateLast - rotaryEncoderDirection == settings.modules)
        {
            cycleStateLast = 0;
        }
        else if (cycleStateLast - rotaryEncoderDirection < 0)
        {
            cycleStateLast = settings.modules - 1;
        }
        rotaryOverride = 60; // x (1 min) cycles of cycleStateLCD() decrements - 1 each run until 0
        lcd.clear();
        cycleStateLCD();
    }
    else
    { // Button
        buttonState = digitalRead(BTN);
        if (buttonState != lastButtonState)
        {
            if (buttonState == LOW)
            {
                if (rotaryOverrideLock == 1)
                {
                    rotaryOverrideLock = 0;
                    rotaryOverride = 0;
                }
                else
                {
                    if (rotaryOverride > 0)
                        rotaryOverrideLock = 1;
                }
            }
        }
        lastButtonState = buttonState;
    }
}

void secondsTimer(byte j)
{
    unsigned long longMilliSecondsCount = millis() - module[j].longMilliSecondsCleared;
    module[j].hours = longMilliSecondsCount / (1000L * 60L * 60L);
    module[j].minutes = (longMilliSecondsCount % (1000L * 60L * 60L)) / (1000L * 60L);
    module[j].seconds = (longMilliSecondsCount % (1000L * 60L * 60L) % (1000L * 60L)) / 1000;
}

void clearSecondsTimer(byte j)
{
    module[j].longMilliSecondsCleared = millis();
    module[j].seconds = 0;
    module[j].minutes = 0;
    module[j].hours = 0;
}

void initializeVariables(byte j)
{
    // Initialization
    module[j].tempMilliOhmsValue = 0;
    module[j].milliOhmsValue = 0;
    module[j].intMilliSecondsCount = 0;
    module[j].longMilliSecondsPreviousCount = 0;
    module[j].longMilliSecondsPrevious = 0;
    module[j].longMilliSecondsPassed = 0;
    module[j].dischargeMilliamps = 0.0;
    module[j].dischargeVoltage = 0.00;
    module[j].dischargeAmps = 0.00;
    module[j].dischargeCompleted = false;
    module[j].batteryFaultCode = 0;
}

void cycleStateValues()
{
    getAmbientTemperature();
    for (byte i = 0; i < settings.modules; i++)
    {
        switch (module[i].cycleState)
        {
        case 0: // Check Battery Voltage
            if (batteryCheck(i))
                module[i].cycleCount++;
            if (module[i].cycleCount == 5)
            {
                initializeVariables(i);
                module[i].batteryCurrentTemp = getTemperature(i);
                module[i].batteryInitialTemp = module[i].batteryCurrentTemp;
                module[i].batteryHighestTemp = module[i].batteryCurrentTemp;
                clearSecondsTimer(i);
                module[i].batteryVoltage = readVoltage(module[i].batteryVolatgePin); // Get battery voltage for Charge Cycle
                module[i].batteryInitialVoltage = module[i].batteryVoltage;
                module[i].batteryLastVoltage = module[i].batteryVoltage;
                module[i].cycleState = 2; // Check Battery Voltage Completed set cycleState to Get Charge Battery
                module[i].cycleCount = 0; // Reset cycleCount for use in other Cycles
            }
            break;
        case 2:                                                                  // Charge Battery
            module[i].batteryVoltage = readVoltage(module[i].batteryVolatgePin); // Get battery voltage
            if (processTemperature(i) == 2)
            {
                //Battery Temperature is >= MAX Threshold considered faulty
                digitalWrite(module[i].chargeMosfetPin, 0); // Turn off TP4056
                module[i].batteryFaultCode = 7;             // Set the Battery Fault Code to 7 High Temperature
                clearSecondsTimer(i);
                module[i].cycleState = 7; // Temperature is to high. Battery is considered faulty set cycleState to Completed
                module[i].cycleCount = 0; // Reset cycleCount for use in other Cycles
            }
            else
            {
                digitalWrite(module[i].chargeMosfetPin, 1); // Turn on TP4056
                module[i].cycleCount = module[i].cycleCount + chargeCycle(i);
                if (module[i].cycleCount >= 5)
                {
                    digitalWrite(module[i].chargeMosfetPin, 0); // Turn off TP4056
                    clearSecondsTimer(i);
                    module[i].cycleState = 3; // Charge Battery Completed set cycleState to Check Battery Milli Ohms
                    module[i].cycleCount = 0; // Reset cycleCount for use in other Cycles
                }
            }
            if (module[i].hours == settings.chargingTimeout) // Charging has reached Timeout period. Either battery will not hold charge, has high capacity or the TP4056 is faulty
            {
                digitalWrite(module[i].chargeMosfetPin, 0); //  Turn off TP4056
                module[i].batteryFaultCode = 9;             // Set the Battery Fault Code to 7 Charging Timeout
                clearSecondsTimer(i);
                module[i].cycleState = 7; // Charging Timeout. Battery is considered faulty set cycleState to Completed
                module[i].cycleCount = 0; // Reset cycleCount for use in other Cycles
            }
            break;
        case 3: // Check Battery Milli Ohms
            module[i].cycleCount = module[i].cycleCount + milliOhms(i);
            module[i].tempMilliOhmsValue = module[i].tempMilliOhmsValue + module[i].milliOhmsValue;
            if (module[i].cycleCount == 4)
            {
                module[i].milliOhmsValue = module[i].tempMilliOhmsValue / 4;
                if (module[i].milliOhmsValue > settings.highMilliOhms) // Check if Milli Ohms is greater than the set high Milli Ohms value
                {
                    module[i].batteryFaultCode = 3; // Set the Battery Fault Code to 3 High Milli Ohms
                    module[i].cycleState = 7;       // Milli Ohms is high battery is considered faulty set cycleState to Completed
                    module[i].cycleCount = 0;       // Reset cycleCount for use in other Cycles
                }
                else
                {
                    if (module[i].minutes <= 1) // No need to rest the battery if it is already charged
                    {
                        module[i].cycleState = 5; // Check Battery Milli Ohms Completed set cycleState to Discharge Battery
                        module[i].cycleCount = 0; // Reset cycleCount for use in other Cycles
                    }
                    else
                    {
                        module[i].cycleState = 4; // Check Battery Milli Ohms Completed set cycleState to Rest Battery
                        module[i].cycleCount = 0; // Reset cycleCount for use in other Cycles
                    }
                    clearSecondsTimer(i);
                }
            }
            break;

        case 4:                                                                  // Rest Battery
            module[i].batteryVoltage = readVoltage(module[i].batteryVolatgePin); // Get battery voltage
            module[i].batteryCurrentTemp = getTemperature(i);
            if (module[i].minutes == settings.restTimeMinutes) // Rest time
            {
                module[i].batteryInitialVoltage = module[i].batteryVoltage; // Reset Initial voltage
                clearSecondsTimer(i);
                module[i].cycleState = 5; // Rest Battery Completed set cycleState to Discharge Battery
            }
            break;
        case 5: // Discharge Battery
            if (processTemperature(i) == 2)
            {
                //Battery Temperature is >= MAX Threshold considered faulty
                digitalWrite(module[i].dischargeMosfetPin, 0); // Turn off Discharge Mosfet
                module[i].batteryFaultCode = 7;                // Set the Battery Fault Code to 7 High Temperature
                clearSecondsTimer(i);
                module[i].cycleState = 7; // Temperature is high. Battery is considered faulty set cycleState to Completed
            }
            else
            {
                if (module[i].dischargeCompleted == true)
                {
                    if (module[i].dischargeMilliamps < settings.lowMilliamps) // No need to recharge the battery if it has low Milliamps
                    {
                        module[i].batteryFaultCode = 5; // Set the Battery Fault Code to 5 Low Milliamps
                        clearSecondsTimer(i);
                        module[i].cycleState = 7; // Discharge Battery Completed set cycleState to Completed
                    }
                    else
                    {
                        module[i].batteryVoltage = readVoltage(module[i].batteryVolatgePin); // Get battery voltage for Recharge Cycle
                        module[i].batteryInitialVoltage = module[i].batteryVoltage;          // Reset Initial voltage
                        clearSecondsTimer(i);
                        if (settings.rechargeCycle == true)
                        {
                            module[i].cycleState = 6; // Discharge Battery Completed set cycleState to Recharge Battery
                        }
                        else
                        {
                            module[i].cycleState = 7; // Discharge Battery Completed set cycleState to Completed as skip Recharge Cycle
                        }
                    }
                }
                else
                {
                    if (dischargeCycle(i))
                        module[i].dischargeCompleted = true;
                }
            }
            break;
        case 6:                                                                  // Recharge Battery
            module[i].batteryVoltage = readVoltage(module[i].batteryVolatgePin); // Get battery voltage
            if (processTemperature(i) == 2)
            {
                //Battery Temperature is >= MAX Threshold considered faulty
                digitalWrite(module[i].chargeMosfetPin, 0); // Turn off TP4056
                module[i].batteryFaultCode = 7;             // Set the Battery Fault Code to 7 High Temperature
                clearSecondsTimer(i);
                module[i].cycleState = 7; // Temperature is to high. Battery is considered faulty set cycleState to Completed
                module[i].cycleCount = 0; // Reset cycleCount for use in other Cycles
            }
            else
            {
                digitalWrite(module[i].chargeMosfetPin, 1); // Turn on TP4056
                module[i].cycleCount = module[i].cycleCount + chargeCycle(i);
                if (module[i].cycleCount >= 5)
                {
                    digitalWrite(module[i].chargeMosfetPin, 0); //  Turn off TP4056
                    clearSecondsTimer(i);
                    module[i].cycleState = 7; // Recharge Battery Completed set cycleState to Completed
                    module[i].cycleCount = 0; // Reset cycleCount for use in other Cycles
                }
            }
            if (module[i].hours == settings.chargingTimeout) // Charging has reached Timeout period. Either battery will not hold charge, has high capacity or the TP4056 is faulty
            {
                digitalWrite(module[i].chargeMosfetPin, 0); // Turn off TP4056
                module[i].batteryFaultCode = 9;             // Set the Battery Fault Code to 7 Charging Timeout
                clearSecondsTimer(i);
                module[i].cycleState = 7; // Charging Timeout. Battery is considered faulty set cycleState to Completed
                module[i].cycleCount = 0; // Reset cycleCount for use in other Cycles
            }
            break;
        case 7: // Completed
            if (!batteryCheck(i))
                module[i].cycleCount++;
            if (module[i].cycleCount == 2)
            {
                module[i].cycleState = 0; // Completed and Battery Removed set cycleState to Check Battery Voltage
                module[i].cycleCount = 0; // Reset cycleCount for use in other Cycles
            }
            break;
        }
        secondsTimer(i);
    }

    cycleStateLCD();
}

void cycleStateLCD()
{
    static byte cycleStateCycles;
    if (rotaryOverride > 0)
    {
        cycleStateLCDOutput(cycleStateLast, 0, 1);
        if (rotaryOverrideLock == 0)
            rotaryOverride--;
    }
    else
    {
        cycleStateLCDOutput(cycleStateLast, 0, 1);
        if (cycleStateLast == settings.modules - 1)
        {
            cycleStateLCDOutput(0, 2, 3);
        }
        else
        {
            cycleStateLCDOutput(cycleStateLast + 1, 2, 3);
        }
        if (cycleStateCycles == settings.screenTime)
        {
            if (cycleStateLast >= settings.modules - 2)
            {
                cycleStateLast = 0;
            }
            else
            {
                cycleStateLast += 2;
            }
            cycleStateCycles = 0;
        }
        else
        {
            cycleStateCycles++;
        }
    }
}

void cycleStateLCDOutput(byte j, byte LCDRow0, byte LCDRow1)
{
    char lcdLine0[25];
    char lcdLine1[25];
    char lcdLine2[25];
    char lcdLine3[25];
    if (rotaryOverride > 0)
    {
        sprintf(lcdLine2, "%-20s", " ");
        sprintf(lcdLine3, "%-18s%02d", " ", rotaryOverride);
        lcd.setCursor(0, 2);
        lcd.print(lcdLine2);
        lcd.setCursor(0, 3);
        lcd.print(lcdLine3);
    }
    if (rotaryOverride > 0 && rotaryOverrideLock == 1)
    {
        sprintf(lcdLine3, "%-20s", "   SCREEN LOCKED");
        lcd.setCursor(0, 3);
        lcd.print(lcdLine3);
    }

    switch (module[j].cycleState)
    {
    case 0: // Check Battery Voltage
        sprintf_P(lcdLine0, PSTR("%d%-19S"), j + 1, PSTR("-BATTERY CHECK"));
        sprintf_P(lcdLine1, PSTR("%-15S%d.%02dV"), module[j].cycleCount > 0 ? PSTR("DETECTED") : PSTR("INSERT BATTERY"), (int)module[j].batteryVoltage, (int)(module[j].batteryVoltage * 100) % 100);
        break;
    case 1: // Get Battery Barcode
        sprintf_P(lcdLine0, PSTR("%d%-19S"), j + 1, PSTR("-SCAN BARCODE"));
        sprintf_P(lcdLine1, PSTR("%-15S%d.%02dV"), PSTR(" "), (int)module[j].batteryVoltage, (int)(module[j].batteryVoltage * 100) % 100);
        break;
    case 2: // Charge Battery
        sprintf_P(lcdLine0, PSTR("%d%-11S%02d:%02d:%02d"), j + 1, PSTR("-CHARGING "), module[j].hours, module[j].minutes, module[j].seconds);
        sprintf_P(lcdLine1, PSTR("%d.%02dV %02d%c C%02d%c %d.%02dV"), (int)module[j].batteryInitialVoltage, (int)(module[j].batteryInitialVoltage * 100) % 100, module[j].batteryInitialTemp, 223, module[j].batteryCurrentTemp, 223, (int)module[j].batteryVoltage, (int)(module[j].batteryVoltage * 100) % 100);
        break;
    case 3: // Check Battery Milli Ohms
        sprintf_P(lcdLine0, PSTR("%d%-19S"), j + 1, PSTR("-RESISTANCE"));
        sprintf_P(lcdLine1, PSTR("%-14S%04dm%c"), PSTR("MILLIOHMS"), (int)module[j].milliOhmsValue, 244);
        break;
    case 4: // Rest Battery
        sprintf_P(lcdLine0, PSTR("%d%-10S%02d:%02d:%02d"), j + 1, PSTR("-RESTING"), module[j].hours, module[j].minutes, module[j].seconds);
        sprintf_P(lcdLine1, PSTR("%-15S%d.%02dV"), PSTR(" "), (int)module[j].batteryVoltage, (int)(module[j].batteryVoltage * 100) % 100);
        break;
    case 5: // Discharge Battery
        sprintf_P(lcdLine0, PSTR("%d%-7S%d.%02dA %d.%02dV"), j + 1, PSTR("-DCHG"), (int)module[j].dischargeAmps, (int)(module[j].dischargeAmps * 100) % 100, (int)module[j].dischargeVoltage, (int)(module[j].dischargeVoltage * 100) % 100);
        sprintf_P(lcdLine1, PSTR("%02d:%02d:%02d %02d%c %04dmAh"), module[j].hours, module[j].minutes, module[j].seconds, module[j].batteryCurrentTemp, 223, (int)module[j].dischargeMilliamps);
        break;
    case 6: // Recharge Battery
        sprintf_P(lcdLine0, PSTR("%d%-11S%02d:%02d:%02d"), j + 1, PSTR("-RECHARGE "), module[j].hours, module[j].minutes, module[j].seconds);
        sprintf_P(lcdLine1, PSTR("%d.%02dV %02d%c C%02d%c %d.%02dV"), (int)module[j].batteryInitialVoltage, (int)(module[j].batteryInitialVoltage * 100) % 100, module[j].batteryInitialTemp, 223, module[j].batteryCurrentTemp, 223, (int)module[j].batteryVoltage, (int)(module[j].batteryVoltage * 100) % 100);
        break;
    case 7: // Completed
        switch (module[j].batteryFaultCode)
        {
        case 0: // Finished
            sprintf_P(lcdLine0, PSTR("%d%-19S"), j + 1, PSTR("-FINISHED"));
            break;
        case 3: // High Milli Ohms
            sprintf_P(lcdLine0, PSTR("%d%-19S"), j + 1, PSTR("-FAULT HIGH OHM"));
            break;
        case 5: // Low Milliamps
            sprintf_P(lcdLine0, PSTR("%d%-19S"), j + 1, PSTR("-FAULT LOW mAh"));
            break;
        case 7: // High Temperature
            sprintf_P(lcdLine0, PSTR("%d%-19S"), j + 1, PSTR("-FAULT HIGH TMP"));
            break;
        case 9: // Charge Timeout
            sprintf_P(lcdLine0, PSTR("%d%-19S"), j + 1, PSTR("-FAULT CHG TIME"));
            break;
        }
        sprintf_P(lcdLine1, PSTR("%04dm%c %04dmAh %d.%02dV"), (int)module[j].milliOhmsValue, 244, (int)module[j].dischargeMilliamps, (int)module[j].batteryVoltage, (int)(module[j].batteryVoltage * 100) % 100);
        break;
    }
    lcd.setCursor(0, LCDRow0);
    lcd.print(lcdLine0);
    lcd.setCursor(0, LCDRow1);
    lcd.print(lcdLine1);
}

bool dischargeCycle(byte j)
{
    float batteryShuntVoltage = 0.00;
    module[j].intMilliSecondsCount = module[j].intMilliSecondsCount + (millis() - module[j].longMilliSecondsPreviousCount);
    module[j].longMilliSecondsPreviousCount = millis();
    if (module[j].intMilliSecondsCount >= settings.dischargeReadInterval || module[j].dischargeAmps == 0) // Get reading every 5+ seconds or if dischargeAmps = 0 then it is first run
    {
        module[j].dischargeVoltage = readVoltage(module[j].batteryVolatgePin);
        batteryShuntVoltage = readVoltage(module[j].batteryVolatgeDropPin);
        if (module[j].dischargeVoltage >= settings.defaultBatteryCutOffVoltage)
        {
            digitalWrite(module[j].dischargeMosfetPin, 1); // Turn on Discharge Mosfet
            module[j].dischargeAmps = (module[j].dischargeVoltage - batteryShuntVoltage) / settings.shuntResistor[j];
            module[j].longMilliSecondsPassed = millis() - module[j].longMilliSecondsPrevious;
            module[j].dischargeMilliamps = module[j].dischargeMilliamps + (module[j].dischargeAmps * 1000.0) * (module[j].longMilliSecondsPassed / 3600000.0);
            module[j].longMilliSecondsPrevious = millis();
        }
        module[j].intMilliSecondsCount = 0;
        if (module[j].dischargeVoltage < settings.defaultBatteryCutOffVoltage)
        {
            digitalWrite(module[j].dischargeMosfetPin, 0); // Turn off Discharge Mosfet
            return true;
        }
    }
    return false;
}

float readVoltage(uint8_t analogPin)
{
    float batterySampleVoltage = 0.00;
    for (byte i = 0; i < 5; i++)
    {
        if (settings.useReferenceVoltage == true)
        {
            batterySampleVoltage = batterySampleVoltage + (analogRead(analogPin) * (settings.referenceVoltage) / 1023.0);
        }
        else
        {
            batterySampleVoltage = batterySampleVoltage + (analogRead(analogPin) * (readVcc()) / 1023.0);
        }
    }
    batterySampleVoltage = batterySampleVoltage / 5;

    return batterySampleVoltage; // Calculate and return the Voltage Reading
}

long readVcc()
{
    long result;
    float returnResult = 0.00;
    // Read 1.1V reference against AVcc
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
#else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif
#if defined(__AVR_ATmega2560__)
    ADCSRB &= ~_BV(MUX5); // Without this the function always returns -1 on the ATmega2560
#endif
    delay(2);            // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Convert
    while (bit_is_set(ADCSRA, ADSC))
        ;
    result = ADCL;
    result |= ADCH << 8;
    returnResult = (settings.internalReferenceVoltage * 1024) / result; // Calculate Vcc (in Volts); = (Internal Volatge reference 1.1) * 1024
    return returnResult;
}

byte milliOhms(byte j)
{
    float resistanceAmps = 0.00;
    float voltageDrop = 0.00;
    float batteryVoltageInput = 0.00;
    float batteryShuntVoltage = 0.00;
    digitalWrite(module[j].dischargeMosfetPin, 0); // Turn off the Discharge Mosfet
    batteryVoltageInput = readVoltage(module[j].batteryVolatgePin);
    digitalWrite(module[j].dischargeMosfetPin, 1); // Turn on the Discharge Mosfet
    batteryShuntVoltage = readVoltage(module[j].batteryVolatgePin);
    digitalWrite(module[j].dischargeMosfetPin, 0); // Turn off the Discharge Mosfet
    resistanceAmps = batteryShuntVoltage / settings.shuntResistor[j];
    voltageDrop = batteryVoltageInput - batteryShuntVoltage;
    module[j].milliOhmsValue = ((voltageDrop / resistanceAmps) * 1000) + settings.offsetMilliOhms; // The Drain-Source On-State Resistance of the Discharge Mosfet
    if (module[j].milliOhmsValue > 9999)
        module[j].milliOhmsValue = 9999;
    return 1;
}

bool chargeCycle(byte j)
{
    if (digitalRead(module[j].chargeLedPin) == HIGH)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

byte processTemperature(byte j)
{
    module[j].batteryCurrentTemp = getTemperature(j);
    if (module[j].batteryCurrentTemp > module[j].batteryHighestTemp && module[j].batteryCurrentTemp != 99)
        module[j].batteryHighestTemp = module[j].batteryCurrentTemp; // Set highest temperature if current value is higher
    if ((module[j].batteryCurrentTemp - ambientTemperature) > settings.tempThreshold && module[j].batteryCurrentTemp != 99)
    {
        if ((module[j].batteryCurrentTemp - ambientTemperature) > settings.tempMaxThreshold)
        {
            //Temp higher than Maximum Threshold
            return 2;
        }
        else
        {
            //Temp higher than Threshold <- Does nothing yet need some flag / warning
            return 1;
        }
    }
    else
    {
        //Temp lower than Threshold
        return 0;
    }
}

byte getTemperature(byte j)
{
  if (module[j].tempCount > 16 || module[j].batteryCurrentTemp == 0 || module[j].batteryCurrentTemp == 99) // Read every 16x cycles
  {
    module[j].tempCount = 0;
    sensors.requestTemperaturesByAddress(tempSensorSerial[j]);
    float tempC = sensors.getTempC(tempSensorSerial[j]);
    if (tempC > 99 || tempC < 0)
    {
      tempC = 99;
      if (module[j].batteryCurrentTemp != 99)
        tempC = module[j].batteryCurrentTemp;
    }
    return (int)tempC;
  }
  else
  {
    module[j].tempCount++;
    return module[j].batteryCurrentTemp;
  }
}

void getAmbientTemperature()
{
  static byte ambientTempCount;
  if (ambientTempCount > 16 || ambientTemperature == 0 || ambientTemperature == 99) // Read every 16x cycles
  {
    ambientTempCount = 0;
    sensors.requestTemperaturesByAddress(tempSensorSerial[8]);
    float tempC = sensors.getTempC(tempSensorSerial[8]);
    if (tempC > 99 || tempC < 0)
    {
      tempC = 99;
      if (ambientTemperature != 99)
        tempC = ambientTemperature;
    }
    ambientTemperature = tempC;
  }
  else
  {
    ambientTempCount++;
  }
}

bool batteryCheck(byte j)
{
    static byte countVoltDrop;
    module[j].batteryVoltage = readVoltage(module[j].batteryVolatgePin);
    if (module[j].batteryVoltage <= settings.batteryVolatgeLeak)
    {
        return false;
    }
    else
    {
        if (module[j].batteryLastVoltage - module[j].batteryVoltage >= 0.1)
        {
            if (countVoltDrop > 2)
            {
                digitalWrite(module[j].chargeMosfetPin, LOW);  // Turn off TP4056
                digitalWrite(module[j].chargeMosfetPin, HIGH); // Turn on TP4056
                digitalWrite(module[j].chargeMosfetPin, LOW);  // Turn off TP4056
                module[j].batteryVoltage = readVoltage(module[j].batteryVolatgePin);
                countVoltDrop = 0;
            }
            else
            {
                countVoltDrop++;
            }
        }
        module[j].batteryLastVoltage = module[j].batteryVoltage;
        return true;
    }
}