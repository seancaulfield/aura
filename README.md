# Aura
--------
Arduino project to mash up a bunch of sensors and log them via MQTT to adafruit.io

Named after the Greek minor goddess whose name translates to "breeze," which
seemed appropriate for an atmospheric sensor system.

# Hardware
----------

All Adafruit but use whatever vendor you prefer:

* (Adafruit ESP32-S2 TFT Feather)[https://www.adafruit.com/product/5300]
* (Adafruit SCD41 CO2 Sensor (Photoacustic))[https://www.adafruit.com/product/5190]
* (Adafruit SCD30 CO2 Sensor (NDIR))[https://www.adafruit.com/product/4867]
* (Adafruit PMSA003I Air Quality Breakout)[https://www.adafruit.com/product/4632]
* (Adafruit SGP30 Mox Gas Sensor)[https://www.adafruit.com/product/3709]
* (Adafruit SGP40 Mox Gas Sensor)[https://www.adafruit.com/product/4829]
* (Adafruit HDC3021 Temperature & Humidity Sensor)[https://www.adafruit.com/product/5989]
* (Adafruit ENS160 Mox Gas Sensor)[https://www.adafruit.com/product/5606]
* (Adafruit TMP117 Temperature Sensor)[https://www.adafruit.com/product/4821]
* (Adafruit BMP390 Pressure Sensor)[https://www.adafruit.com/product/4816]

Measurements collected by each sensor:

* SCD41
  + CO2 (ppm)
* SCD30
  + CO2 (ppm)
* PMSA003I
  + PM1.0 ("CF=1, standard particle", ug/m^3)
  + PM2.5 ("CF=1, standard particle", ug/m^3)
  + PM10.0 ("CF=1, standard particle", ug/m^3)
  + PM1.0 ("under atmospheric environment", ug/m^3)
  + PM2.5 ("under atmospheric environment", ug/m^3)
  + PM10.0 ("under atmospheric environment", ug/m^3)
  + count >=0.3um particles in 0.1L air
  + count >=0.5um particles in 0.1L air
  + count >=1.0um particles in 0.1L air
  + count >=2.5um particles in 0.1L air
  + count >=5.0um particles in 0.1L air
  + count >=10um particles in 0.1L air
* SGP30
  + eCO2 (ppm)
  + TVOC (ppb)
* SGP40
  + "VOC index"
* HDC3021
  + Temperature (C)
  + Relative humidity (%)
* ENS160
  + AQI (0-5?)
  + eCO2 (ppm)
  + TVOC (ppb)
  + Resistance HP0 (ohm)
  + Resistance HP1 (ohm)
  + Resistance HP2 (ohm)
  + Resistance HP3 (ohm)
* TMP117
  + Temperature (C)
* BMP390
  + Temperature (C)
  + Pressure (Pa)
