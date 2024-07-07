MySoilSensorESP32 - Multi-channel soil moisture sensor
====

## Objectives
* get information about soil moisture, to drive decision when to water a plant
* deal with variability in placement of sensor within a pot, 
  resulting in differing range of measured values etc.

## Use Case
* monitor soil moisture of multiple pots using *one* gadget, with a durable sensor stuck in each pot

## Requirements

### Connectivity
- [x] `R001` the device has a wireless connection to the home automation system, i.e. WiFi or MySensors RF24
  - [x] `R001.1` when using Wifi, the SSID and password are configurable in source code
- [x] `R002` the device reports measurements and status via MQTT
  - [x] `R000.1` the MQTT broker name and port are configurable in source code
- [x] `R003` the device supports over-the-air software updates

### Hardware
- [x] `R010` the device runs on batteries
- [x] `R011` if non-rechargeable batteries are used, they must last for at least a year
- [ ] `R012` if rechargeable batteries are used, they must be rechargeable via a solar panel

### Measurements
- [x] `R020` the device connects to at least 4 soil sensors
- [x] `R021` the device is compatible with capacitive soil sensors
- [x] `R022` the device measures the *absolute* voltage output by each sensor, in millivolts
- [x] `R023` when a meaningful range of sensor voltages has been observed, then the device also calculates *relative* moisture
  - [x] `R023.1` *relative* moisture is calculated as a percentage of the observed voltage range, with smaller values indicating dry soil, and vice versa 
- [x] `R024` the device measures the battery voltage, which may be up to 4.7V

### Reports
- [x] `R030` all reports are made as MQTT messages, in a format the can be easily parsed by OpenHAB
- [x] `R031` the device reports absolute voltage measurements
- [x] `R032` the device reports relative moisture measurements
- [x] `R033` the device reports relevant parameters of its internal status
- [x] `R034` the device reports the battery voltage
- [x] `R035` the device reports Wifi connection quality ("RSSI") on a scale of 0 to 100, with 100 being the best quality
- [x] `R036` the device reports the number of times it has woken up from deep sleep
- [x] `R037` the device reports the total time it has been operational since power-up, including sleep times

### Configuration 
- [x] `R040` the interval between measurement reports is configurable in source code, between 1 min and 1 hour
- [ ] `R041` the interval between device status reports is configurable in source code, between the measurements report interval and 12h

### Battery life ###

## Architecture
- [x] `R101` the device uses cheap Aliexpress sourced soil moisture sensors based on a 555 oscillator
- [x] `R102` the dashboard uses a Wifi-capable processor, i.e. ESP32. Rationale: ability to work outside MySensors range
- [x] `R102` the device uses 3 AAA batteriesRationale: only way to satisfy battery life requirements
