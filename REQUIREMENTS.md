MySoilSensorESP32 - Multi-channel soil moisture sensor
====

- [MySoilSensorESP32 - Multi-channel soil moisture sensor](#mysoilsensoresp32---multi-channel-soil-moisture-sensor)
  - [Objectives](#objectives)
  - [Use Case](#use-case)
  - [Requirements](#requirements)
    - [Connectivity](#connectivity)
    - [Hardware](#hardware)
    - [Measurements](#measurements)
    - [Reports](#reports)
    - [Configuration](#configuration)
  - [Architecture](#architecture)
  - [Mechanical design](#mechanical-design)

## Objectives
* get information about soil moisture, to drive decision when to water a plant
* deal with variability in placement of sensor within a pot, 
  resulting in differing range of measured values etc.

## Use Case
* monitor soil moisture of multiple pots using *one* gadget, with a durable sensor stuck in each pot

## Requirements

### Connectivity
- [x] `R001` the device has a wireless connection to the home automation system, 
  via WiFi. <br>*Rationale:* decide against MySensors RF24 because of range concerns, 
  and because it would not support MQTT
  - [x] `R001.1` when using Wifi, the SSID and password are configurable in source code
- [x] `R002` the device communicates via MQTT
  - [x] `R002.1` the MQTT broker name and port are configurable in source code
- [x] `R003` the device supports over-the-air software updates
- [x] `R004' device behavior can be configured over-the-air (details below, in 
  *configuration* section)
- [x] `R005` the device receives runtime configuration instructions via MQTT
  - [x] `R005.1` it is acceptable that the device polls for runtime configuration 
  instructions only when it wakes up for measurements. This means that the 
  configuration instructions must be published as *retained* messages.
  - [x] `R005.2` the device acknowledges receipt of runtime configuration 
  information by deleting the MQTT topic

### Hardware
- [x] `R010` the device runs on batteries
- [x] `R011` if non-rechargeable batteries are used, they must last for at least 
  a year
- [ ] `R012` if rechargeable batteries are used, they must be rechargeable via a 
  solar panel

### Measurements
- [x] `R020` the device connects to at least 4 soil sensors
- [x] `R021` the device is compatible with capacitive soil sensors
- [x] `R022` the device measures the *absolute* voltage output by each sensor, in 
  millivolts
- [x] `R023` the device detects when a sensor is disconnected
  - [x] `R023.1` when a sensor is disconnected, the device resets the range of 
  observed voltages
  - [x] `R023.2` while a sensor is disconnected, measurement values do not 
  contribute to the "observed range 
- [x] `R024` while a meaningful range of sensor voltages has been observed, the 
  device also calculates *relative* moisture. 
  - [x] `R024.1` *Design decision*: "meaningful" range is 5% of the maximum ADC 
  - [x] `R024.2` *relative* moisture is calculated as a fraction of the observed 
  voltage range, with smaller values indicating dry soil, and vice versa 
- [x] `R025` the device measures the battery voltage, which may be up to 4.7V

### Reports
- [x] `R030` all reports are made as MQTT messages, in a format the can be easily 
  parsed by OpenHAB
- [x] `R031` the device reports absolute voltage measurements of sensor outputs, 
  in millivolts
- [x] `R032` the device reports relative moisture measurements (see above), on a 
  scale of 0 to 1000 (i.e.promille of observed range)
- [x] `R033` the device reports relevant parameters of its internal status
- [x] `R034` the device reports the battery voltage
- [x] `R035` the device reports Wifi connection quality ("RSSI") on a scale of 
  0 to 100, with 100 being the best quality
- [x] `R036` the device reports the number of times it has woken up from deep sleep
- [x] `R037` the device reports the total time it has been operational since 
  power-up, including sleep times
- [x] `R038` the device reports the time it was awake to make a measurement, 
  in milliseconds, after each wake period
- [x] `R039` the device reports the time it took to connect to Wifi, 
  in milliseconds, after each wake period

### Configuration 
- [x] `R040` the interval between measurement reports is configurable in source 
  code, between 1 min and 1 hour
- [x] `R041` the interval between device status reports is configurable in source 
  code, between the measurements report interval and 24h
- [x] `R042` the interval between measurement reports can be changed at runtime
- [x] `R043` the interval between device status reports can be changed at runtime
- [ ] `R044` the range for *relative* moisture calculation can be reset at runtime
  - [ ] `R044.1` the range for *relative* moisture calculation can be reset 
  independently for each channel 

## Architecture
- [x] `R101` the device uses cheap Aliexpress sourced capacitive soil moisture 
  sensors based on a 555 oscillator
- [x] `R102` the device uses a Wifi-capable processor, i.e. ESP32. <br>
  *Rationale:* ability to work outside MySensors range
- [x] `R102` the device uses 2 AAA batteries. <br>*Rationale:* only way to 
  satisfy battery life requirements

## Mechanical design
- [x] `R103` the connection from sensor to device can be up to 60cm long. 
  *Rationale*: balcony flower boxes are 60cm wide, and a device can sit between 
  2 flower boxes
- [x] `R104` the connection from sensor to device is easy to disconnect 
  "in the field"
- [x] `R105` the connection from sensor to device uses cheap (<0.5€) connectors. 
  <br>*Solution:* RJ12 6P4C connectors
- [x] `R106` the connector from sensor cable to device need not be waterproof. 
  <br>*Rationale:* rain protection is achieved by other means, protecting the 
  entire device.
