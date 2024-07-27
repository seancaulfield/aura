
/*
 * aura.ino
 *
 * Atmospheric sensor mash-up, named after the ancient Greek deity (whose name
 * means "breeze").
 *
 * Takes data from a bunch of sensors, averages it over an interval, and dumps
 * it to adafruit.io.
 *
 * Author: Sean Caulfield <sean@yak.net>
 * License: GPL v2.0
 *
 */

#include <Arduino.h>
#include <Wire.h>
#include <AdafruitIO_WiFi.h>
#include <Adafruit_SGP30.h>
#include "config.h"

// Adafruit.IO library objects
AdafruitIO_WiFi io(AIO_USER, AIO_KEY, WIFI_SSID, WIFI_PASS);
AdafruitIO_Feed *feed_co2 = io.feed("co2");

// UART command receive buffer
char uart_buff[64];

// Sensor control objects
Adafruit_SGP30 sgp30;


// Wipe out any calibration data
void clear_sgp30_calibration() {
    // TODO rewrite for ESP32 which doesn't have EEPROM lib
    //EEPROM.write(SGP30_EEPROM_ADDR_ECO2,   0xFF);
    //EEPROM.write(SGP30_EEPROM_ADDR_ECO2+1, 0xFF);
    //EEPROM.write(SGP30_EEPROM_ADDR_TVOC,   0xFF);
    //EEPROM.write(SGP30_EEPROM_ADDR_TVOC+1, 0xFF);
    //Serial.println("SGP30 calibraiton reset");
}

// Fetch calibration data and save to EEPROM
void store_sgp30_calibration() {
    int rc = 0;
    
    rc = sgp30.getBaseline();

    if (rc == SUCCESS) {
        uint8_t msb, lsb;

        Serial.print("eCO2 calibration = ");
        Serial.print(sgp30.baselineCO2, HEX);
        Serial.print("TVOC calibration = ");
        Serial.println(sgp30.baselineTVOC, HEX);

        // TODO rewrite for ESP32 which doesn't have EEPROM lib

        msb = (sgp30.baselineCO2 >> 8) & 0xFF;
        lsb = (sgp30.baselineCO2 >> 0) & 0xFF;
        //EEPROM.write(SGP30_EEPROM_ADDR_ECO2, msb);
        //EEPROM.write(SGP30_EEPROM_ADDR_ECO2+1, lsb);

        msb = (sgp30.baselineTVOC >> 8) & 0xFF;
        lsb = (sgp30.baselineTVOC >> 0) & 0xFF;
        //EEPROM.write(SGP30_EEPROM_ADDR_TVOC, msb);
        //EEPROM.write(SGP30_EEPROM_ADDR_TVOC+1, lsb);

        Serial.println("SGP30 calibraiton saved");

    } else if (rc == ERR_I2C_TIMEOUT) {
        Serial.println("I2C timeout fetching baseline!");
    } else if (rc == ERR_BAD_CRC) {
        Serial.println("Bad CRC fetching baseline!");
    } else {
        Serial.println("Baseline fetch failed!");
    }

}

// Attempt to load calibration data from EEPROM (if it's valid, ie, not all
// 1's).
void load_sgp30_calibration() {
    uint8_t msb = 0xff, lsb = 0xff;
    uint16_t cal_eco2, cal_tvoc;

    // TODO rewrite for ESP32 which doesn't have EEPROM lib

    //msb = EEPROM.read(SGP30_EEPROM_ADDR_ECO2);
    //lsb = EEPROM.read(SGP30_EEPROM_ADDR_ECO2+1);
    cal_eco2 = (msb << 8) | lsb;

    //msb = EEPROM.read(SGP30_EEPROM_ADDR_TVOC);
    //lsb = EEPROM.read(SGP30_EEPROM_ADDR_TVOC+1);
    cal_tvoc = (msb << 8) | lsb;

    // Only use if initialized
    if (cal_eco2 != 0xFFFF && cal_tvoc != 0xFFFF) {
        sgp30.setBaseline(cal_eco2, cal_tvoc);
    }
}
#endif

// This is a terrible way to do this but I need like a single command.
void dispatch(char *cmd, size_t len) {
    if (strstr(cmd, "CALIBRATE") != NULL) {
        //store_sgp30_calibration();
    } else if (strstr(cmd, "WIPE") != NULL) {
        //clear_sgp30_calibration();
    } else {
        Serial.println("?");
    }
}

void processSerialInput() {
    if (Serial && Serial.available()) {
        memset(uart_buff, 0, sizeof(uart_buff));
        size_t rc = Serial.readBytesUntil('\n', uart_buff, sizeof(uart_buff));
        if (rc) {
            size_t terminator = min(rc, sizeof(uart_buff));
            uart_buff[terminator] = '\0';
            dispatch(uart_buff, terminator);
        }
    }
}

void setup() {

    delay(300);

    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    if (Serial) {
        Serial.begin(230400);
        Serial.println("hai");
    }

    Wire.begin();

    // Connect to adafruit.io
    if (Serial) Serial.print("Connecting to adafruit.io");
    io.connect();
    while (io.status() < AIO_CONNECTED) {
        if (Serial) Serial.print(".");
        delay(1000);
    }
    if (Serial) {
        Serial.println();
        Serial.println(io.statusText());
    }

    // Initialize sensors

    digitalWrite(13, LOW);

}

void loop() {
    
    // Handle "background" IO
    io.run();

    // TODO collect sensor data here
    
    // Handle our (trivial) incoming commands
    processSerialInput();

    // Blink LED while farting data up
    digitalWrite(13, HIGH);
    delay(100); // TODO upload data
    digitalWrite(13, LOW);

    // Yield/delay sled, to keep from emitting too much data too fast and
    // locking out my account :P
    yield();
    delay(500);
    yield();
    delay(500);
    yield();

}

// vi: ts=4 sw=4 expandtab
