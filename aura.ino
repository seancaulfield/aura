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
#include <SparkFun_SCD30_Arduino_Library.h>
#include <SparkFun_SGP30_Arduino_Library.h>
#include <SparkFun_MS5637_Arduino_Library.h>
#include <Adafruit_seesaw.h>
#include <AdafruitIO_WiFi.h>
#include "config.h"

// Physical constants (used for relative->abs humidity calc)
const float WATER_G_PER_MOLE = 18.01534; // g/mol
const float GAS_CONSTANT = 8.31447215;   // J/mol/K
const float ABSOLUTE_ZERO = -273.15; // degrees C

// EEPROM addresses for holding calibration information
const size_t SGP30_EEPROM_ADDR_ECO2 = 0x06;
const size_t SGP30_EEPROM_ADDR_TVOC = 0x08;

// Number of readings to average together before posting
const int BATCH_SIZE = 60;

// Initial number of readings to discard (for bogosity)
const int INITIAL_DISCARD = 7;

// Adafruit.IO library objects
AdafruitIO_WiFi io(AIO_USER, AIO_KEY, WIFI_SSID, WIFI_PASS);
AdafruitIO_Feed *feed_co2         = io.feed("co2");
AdafruitIO_Feed *feed_eco2        = io.feed("eco2");
AdafruitIO_Feed *feed_humidity    = io.feed("humidity");
AdafruitIO_Feed *feed_pressure    = io.feed("pressure");
AdafruitIO_Feed *feed_temperature = io.feed("temperature");
AdafruitIO_Feed *feed_voc         = io.feed("voc");

// Sensor library objects
SCD30 co2sensor;
SGP30 gassensor;
MS5637 barometer;
Adafruit_seesaw soilsensor_main;
Adafruit_seesaw soilsensor_babby;

// Sensor readings globals
float curr_co2ppm     = 0.0;
float curr_eco2ppm    = 0.0;
float curr_vocppb     = 0.0;
float curr_rh         = 0.0;
float curr_temp       = 0.0;
float curr_pressure   = 0.0;
float curr_soil_main  = 0.0;
float curr_soil_babby = 0.0;

// Sensor reading moving averages
float agg_co2ppm      = 0.0;
float agg_eco2ppm     = 0.0;
float agg_vocppb      = 0.0;
float agg_rh          = 0.0;
float agg_temp        = 0.0;
float agg_pressure    = 0.0;
float agg_soil_main   = 0.0;
float agg_soil_babby  = 0.0;

// Number of readings aggregated so far
int num_readings = 0;

// Sensor presence globals
bool has_scd30 = true;
bool has_sgp30 = true;
bool has_barometer = true;
bool has_soil_main = true;
bool has_soil_babby = true;

// Discard first INITIAL_DISCARD to eliminate bogus data
bool should_discard = true;

// UART command receive buffer
char uart_buff[64];

//
// Annoyingly, the SGP30 humidity compensation must be specified in terms of
// absolute humidity (g/m^3) instead of relative humidity (%). Conversion using
// ideal gas law + formula for saturation of water vapor based on current
// temperature.
//
// Reference:
// https://carnotcycle.wordpress.com/2012/08/04/how-to-convert-relative-humidity-to-absolute-humidity/
//
uint16_t rel_to_abs_humid(float rh, float tempC) {
  float p;

  // Get maximum saturation pressure of water vapor at current temp
  p = 6.112 * pow(M_E, (17.67 / tempC) / (tempC + 243.5));

  // If we're at x% humidity, the water vapor pressure should be that
  // percentage of the saturation value.
  p = p * (rh / 100.0);

  // Use ideal gas law (PV=nRT) to infer the density of water in a cubic meter
  // of air. Assume v = 1 m^3, solve for n:
  //      P * V     P
  //  n = ----- = -----
  //      R * T   R * T
  float tempK = tempC + -ABSOLUTE_ZERO;
  float n = p / (GAS_CONSTANT * tempK);

  // Finally, given n, calculate density:
  //                          1
  // density = v/m = --------------------
  //                 n * WATER_G_PER_MOLE
  //
  float density = 1 / (n * WATER_G_PER_MOLE);

  // Finally (for real this time), convert the density into the weirdo format
  // the SGP30 expects it in (16bit, fixed precision 8.8)
  double i, f;
  uint8_t msb, lsb;

  f = modf(density, &i);
  msb = lrint(i) % 256;
  lsb = lrint(f * 256) & 0xFF;

  return (msb<<8) | lsb;
}

// Abstraction to handle reading moisture levels
uint16_t get_moist(Adafruit_seesaw *sensor) {
    const int MOISTURE_BUTTON = 0;
    return sensor->touchRead(MOISTURE_BUTTON);
}

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
    
    rc = gassensor.getBaseline();

    if (rc == SUCCESS) {
        uint8_t msb, lsb;

        Serial.print("eCO2 calibration = ");
        Serial.print(gassensor.baselineCO2, HEX);
        Serial.print("TVOC calibration = ");
        Serial.println(gassensor.baselineTVOC, HEX);

        // TODO rewrite for ESP32 which doesn't have EEPROM lib

        msb = (gassensor.baselineCO2 >> 8) & 0xFF;
        lsb = (gassensor.baselineCO2 >> 0) & 0xFF;
        //EEPROM.write(SGP30_EEPROM_ADDR_ECO2, msb);
        //EEPROM.write(SGP30_EEPROM_ADDR_ECO2+1, lsb);

        msb = (gassensor.baselineTVOC >> 8) & 0xFF;
        lsb = (gassensor.baselineTVOC >> 0) & 0xFF;
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
        gassensor.setBaseline(cal_eco2, cal_tvoc);
    }
}

// This is a terrible way to do this but I need like a single command.
void dispatch(char *cmd, size_t len) {
    if (strstr(cmd, "CALIBRATE") != NULL) {
        store_sgp30_calibration();
    } else if (strstr(cmd, "WIPE") != NULL) {
        clear_sgp30_calibration();
    } else {
        Serial.println("?");
    }
}

void postBatch() {
    if (has_scd30) {
        agg_co2ppm /= num_readings;
        agg_rh /= num_readings;
        agg_temp /= num_readings;
        feed_co2->save(agg_co2ppm);
        feed_humidity->save(agg_rh);
        feed_temperature->save(agg_temp);
    }
    if (has_sgp30) {
        agg_eco2ppm /= num_readings;
        agg_vocppb /= num_readings;
        feed_eco2->save(agg_eco2ppm);
        feed_voc->save(agg_vocppb);
    }
    if (has_barometer) {
        agg_pressure /= num_readings;
        feed_pressure->save(agg_pressure);
    }
    if (has_soil_main)  {
        agg_soil_main /= num_readings;
        feed_soil_main->save(agg_soil_main);
    }
    if (has_soil_babby) {
        agg_soil_babby /= num_readings;
        feed_soil_babby->save(agg_soil_babby);
    }
}

void printReadings() {

    if (has_scd30) {
        Serial.print(" co2:");
        Serial.print(agg_co2ppm);
        Serial.print(" rh:");
        Serial.print(agg_rh);
        Serial.print(" temp:");
        Serial.print(agg_temp);
    }

    if (has_sgp30) {
        Serial.print(" eco2:");
        Serial.print(agg_eco2ppm);
        Serial.print(" voc:");
        Serial.print(agg_vocppb);
    }

    if (has_barometer) {
        Serial.print(" pressure:");
        Serial.print(agg_pressure);
    }

    if (has_soil_main) {
        Serial.print(" soil_main:");
        Serial.print(agg_soil_main);
    }

    if (has_soil_babby) {
        Serial.print(" soil_babby:");
        Serial.print(agg_soil_babby);
    }

    Serial.println();

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

    if (!co2sensor.begin()) {
        if (Serial) Serial.println("SCD30 not found!");
        has_scd30 = false;
    }

    if (!barometer.begin()) {
        if (Serial) Serial.println("MS5637 not found!");
        has_barometer = false;
    }

    if (gassensor.begin()) {
        gassensor.initAirQuality();
        load_sgp30_calibration();
    } else {
        if (Serial) Serial.println("SGP30 not found!");
        has_sgp30 = false;
    }

    if (!soilsensor_main.begin(SOIL_SENSOR_ADDR_MAIN)) {
        has_soil_main = false;
        if (Serial) Serial.println("Adafruit soil sensor (main) not found!");
    }

    if (!soilsensor_babby.begin(SOIL_SENSOR_ADDR_BABBY)) {
        has_soil_babby = false;
        if (Serial) Serial.println("Adafruit soil sensor (babby) not found!");
    }

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

    digitalWrite(13, LOW);

}

void loop() {
    
    // Handle "background" IO
    io.run();
    
    // Check atmospheric pressure
    if (has_barometer) {
        curr_pressure = barometer.getPressure();
    }

    // Update CO2 sensor with current (absolute) pressure information (but only
    // if the data is valid(ish).
    if (has_barometer && has_scd30 && curr_pressure > 0.0) {
      co2sensor.setAmbientPressure(curr_pressure);
    }

    // Read out data from SCD30 sensor
    if (has_scd30 && co2sensor.dataAvailable()) {

        curr_co2ppm = co2sensor.getCO2();
        curr_temp = co2sensor.getTemperature();
        curr_rh = co2sensor.getHumidity();

        // Convert RH to annoying absolute humidity for for SGP30 compensation
        // (we don't track this because it's derrived and generally not wanted
        // so we can reconstruct it if needed).
        float abs_humid = rel_to_abs_humid(curr_rh, curr_temp);
        gassensor.setHumidity(abs_humid);

    }

    // Read out data from SGP30 sensor
    if (has_sgp30) {
        gassensor.measureAirQuality();
        curr_eco2ppm = gassensor.CO2;
        curr_vocppb = gassensor.TVOC;
    }

    // Read moisture levels
    if (has_soil_main) {
        curr_soil_main = get_moist(&soilsensor_main);
    }
    if (has_soil_babby) {
        curr_soil_babby = get_moist(&soilsensor_babby);
    }

    // Handle our (trivial) incoming commands
    processSerialInput();

    // Since some initial readings (first for the barometer, first eight
    // for the SGP30) are bogus, just discard until we hit INITIAL_DISCARD,
    // then reset num_readings for normal operation.
    if (should_discard) {

        if (num_readings >= INITIAL_DISCARD) {
            should_discard = false;
            num_readings = 0;
        } else {
            num_readings++;
        }

    // If we have enough readings to send up a batch, do so, then reset aggs
    } else if (num_readings >= BATCH_SIZE) {

        // Blink LED while farting data up
        digitalWrite(13, HIGH);
        postBatch();
        printReadings();
        digitalWrite(13, LOW);

        num_readings = 0;
        if (has_scd30) {
            agg_co2ppm = 0.0;
            agg_rh = 0.0;
            agg_temp = 0.0;
        }
        if (has_sgp30) {
            agg_eco2ppm = 0.0;
            agg_vocppb = 0.0;
        }
        if (has_barometer)  agg_pressure   = 0.0;
        if (has_soil_main)  agg_soil_main  = 0.0;
        if (has_soil_babby) agg_soil_babby = 0.0;

    // Otherwise, add to moving average for each metric
    } else {
        num_readings++;
        if (has_scd30) {
            agg_co2ppm += curr_co2ppm;
            agg_rh     += curr_rh;
            agg_temp   += curr_temp;
        }
        if (has_sgp30) {
            agg_eco2ppm += curr_eco2ppm;
            agg_vocppb  += curr_vocppb;
        }
        if (has_barometer)  agg_pressure   += curr_pressure;
        if (has_soil_main)  agg_soil_main  += curr_soil_main;
        if (has_soil_babby) agg_soil_babby += curr_soil_babby;
    }

    // Yield/delay sled, to keep from emitting too much data too fast and
    // locking out my account :P
    yield();
    delay(101);
    yield();
    delay(102);
    yield();
    delay(103);
    yield();
    delay(104);
    yield();
    delay(105);
    yield();
    delay(106);
    yield();
    delay(107);
    yield();
    delay(108);
    yield();

}

// vi: ts=4 sw=4 expandtab
