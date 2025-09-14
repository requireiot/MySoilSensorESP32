MySoilSensorESP32 -- a multi-channel soil moisture sensor with long battery life
----
This is part of my home automation setup. For details, see my [blog](https://requireiot.com/my-home-automation-story-part-1/).

<img src="pictures/overview.jpg" width="400" /> 

- [MySoilSensorESP32 -- a multi-channel soil moisture sensor with long battery life](#mysoilsensoresp32----a-multi-channel-soil-moisture-sensor-with-long-battery-life)
- [Objective](#objective)
- [Design decisions](#design-decisions)
  - [How to make it waterproof-ish](#how-to-make-it-waterproof-ish)
  - [How to communicate](#how-to-communicate)
  - [Minimize power consumption](#minimize-power-consumption)
- [Building blocks](#building-blocks)
- [Hardware](#hardware)
- [Batteries](#batteries)
- [Firmware](#firmware)
- [MQTT messages](#mqtt-messages)
- [Over-the-air update](#over-the-air-update)
  - ['Classic' OTA firmware update](#classic-ota-firmware-update)
  - [OTA firmware download from a HTTP server](#ota-firmware-download-from-a-http-server)
- [Debugging support](#debugging-support)
- [Power consumption and battery life](#power-consumption-and-battery-life)
  - [Wake time](#wake-time)
  - [Actual power consumption](#actual-power-consumption)
- [Lessons learned](#lessons-learned)
  - [Analog-to-digital conversion with the ESP32](#analog-to-digital-conversion-with-the-esp32)
  - [Some things are outside of my control](#some-things-are-outside-of-my-control)
  - [An almost easy project](#an-almost-easy-project)
  - [Bad luck with channel 13](#bad-luck-with-channel-13)


## Objective
In summer, I have a lot of potted plants out on the balcony, and I need to know when 
to water them. Indoors, most of my home automation gadgets are based on 
[MySensors](https://www.mysensors.org/), but out on the balcony the signal is 
too weak, so I needed something WiFi-based, but without the high battery 
consumption typical of WiFi solutions.

## Design decisions

### How to make it waterproof-ish

There will be rain out on the balcony, so the modules need to be waterproof-ish. 
Real waterproof sealed cases are expensive, so I opted for a cheap plastic case 
with a Ziploc bag over it, with the opening facing down ... good enough for the 
intended use case.

### How to communicate

To exchange information between the soil sensors and the home automation system 
(soil moisture and performance characteristics from the device, commands to 
change operating parameters or start over-the-air updates to the device), 
I could think of the following technologies:
1. **MQTT**. *Pro*: I already have an MQTT broker running, and this is independent 
   of a specific home automation system. *Con*: Communication is a bit unpredictable, as I found out
2. **HTTP** requests to a REST API, e.g. for OpenHAB. *Pro*: More deterministic, 
   you know exactly when a message has been accepted. *Con*: Specific for one 
   home automation system

I went for (1).

### Minimize power consumption

- a bare ESP32-WROOM module is better than a development module -- probably due 
  to the voltage regulators and USB interfaces on those modules. Power 
  consumption during deep sleep, with a bare ESP32-WROOM, powered from 2x 
  AAA battery, is under **10µA**.
- to avoid any power loss through a voltage regulator, I am running the ESP32 
  directly from a pair of AAA batteries. Newer processors will work down to 2.3V, 
  I just had to make sure the soil moisture sensor also works below 3V (see below)
- the soil sensors are *powered* by GPIO pins, which are turned off during sleep
- to minimize the time the processor is awake, and in particular the time that 
  WiFi is active, I try to reconnect to WiFi using previously established SSID, 
  IP address and channel number, which are cached in RTC memory

## Building blocks

- an ESP32 module with custom Arduino-based software
- two AAA batteries, which should last for more than a year
- up to 4 cheap capacitive soil sensors, of the kind you find on Aliexpress for 
  less than a Euro
- my home network, with a DHCP server assigning pseudo-static IP addresses to 
  everybody, based on MAC address
- my home automation system, with an MQTT broker and OpenHAB for visualization 
  and control

## Hardware

<img src="pictures/open-case.jpg" width="400" /> <img src="pictures/board-bottom.jpg" width="400" />

The hardware is very simple, see the [schematic](hardware/MySoilSensorESP32.pdf) 
and the pictures above. 

For development and debugging, I added pin connectors to connect to FTDI-232 style 
USB-to-serial interfaces, one for the primary UART (for firmware upload) and one 
for UART#2, where the software outputs logging messages. For a production system, 
this is not necessary and can be left out, of course.

## Batteries

One option is to power the ESP32 directly from two AA or AAA batteries. The battery life calculations reported below apply to this configuration.

As an alternative, I have built a few units with a _rechargable_ battery and a solar panel for charging, attached to the outside of the case. This works as well, if the case can be positioned such that the solar panel faces the sun. Here is the bill of materials for this variant:
* a "5V 75mA 80x45mm" solar panel, from Aliexpress, ca. €1
* a TP4056 battery management module, from Aliexpress, ca. €0.20
* a small Li-ion battery, 3.7V at 1000 mA, from Reichelt (p/n AKKU 30389), €1.80 ... this is a noname replacement battery for some GPS receiver. I have also used a replacement battery for a 1st gen Apple iPod nano.
* a low-drop, low quescent current voltage regulator for 3.3V, HT7833

## Firmware

The software was developed using [Platformio](https://platformio.org/). Just 
download the repository contents into an empty folder, and open that folder as 
a Platformio project. 

In folder `src/`, rename file `myauth_sample.h` to `myauth.h` and enter your 
Wifi SSID and password.

In the top folder, rename file `devices.ini.sample` to `devices.ini` and enter the hostnames and IP addresses for your environment. I built multiple units and wanted to update each of them over-the-air, so I needed a separate entry for each, specifying each IP address as assigned by my DHCP server.

Edit file `platformio.ini` and enter the COM port for your 
environment. 

I used a 7x9 cm prototype board for ESP32 and ESP8266 boards, cut to fit inside 
a 100x60x25 mm plastic case. The soil sensors are connected via cables with an 
RJ12 6P4C connecter on the module end, for easy connection "in the field".

The capacitive soil sensors I got from Aliexpress had an NE555 chip that only 
works down to 3.0V, whereas the ESP32 itself will still work when the battery 
voltage is as low as 2.5V, so I replaced the NMOS NE555 with the CMOS version 
ILC555D, which work down to 2.0V. I also bypassed the voltage regulator on the 
sensor board.

## MQTT messages

The device publishes to MQTT topics that include its own hostname:
- to <code>soil/__hostname__/debug</code> it publishes a JSON string with various 
  internal parameters, such as battery voltage, uptime etc., once every 12 hours
- to <code>soil/__hostname__/state</code> it publishes a JSON string with
   measurement results, once every hour 
- to <code>soil/__hostname__/wifi</code> it publishes a JSON string with some 
  performance metrics, such as Wifi quality, the time needed to connect to the 
  Wifi access point, etc., once every hour 
- it subscribes to <code>soil/__hostname__/cmd</code> to enable over-the-air 
  updates (see below)
- it subscribes to <code>soil/__hostname__/config</code> to receive a JSON-format 
  string enable to change some operating parameters (see source code for details)

For example, my device has the hostname `esp32-D80270`, so it will publish to 
`soil/esp32-D80270/state` etc.

## Over-the-air update

The device supports OTA firmware updates, despite the fact that it is in deep 
sleep most of the time. How? When it wakes up, it subscribes to MQTT topic 
<code>soil/_hostname_/cmd</code> (see above).

### 'Classic' OTA firmware update

When the device wakes up, and it finds a retained MQTT message <code>soil/_hostname_/cmd</code> with the payload of `ota`, it enables ArduinoOTA and waits for an upload of new firmware, instead of going to sleep. 
It also deletes the topic to acknowledge that it has received the command. 

To perform an OTA firmware update,
1. compile your code for the target that has "-ota" in its name (look at 
   `platformio.ini` and adjust the IP address or device hostname to match your 
   conguration)
1. publish a retained message for topic <code>soil/_hostname_/cmd</code> with 
   a payload of `ota` For my device `esp32-D80270`, I run the following shell 
   comand on the home automation server: <br> 
 `mosquitto_pub -r -t "soil/esp32-D80270/cmd" -m "ota"`
1. wait for the item to disappear. I watch it with a tool like 
   [MQTT Explorer](https://mqtt-explorer.com/), or run this shell command 
   repeatedly <br/> `mosquitto_sub -t "soil/esp32-D80270/cmd"`  
1. upload your firmware via Platformio

### OTA firmware download from a HTTP server

When the device wakes up, and it finds a retained MQTT message <code>soil/_hostname_/cmd</code> with the payload of `update`, it attempts to download a new firmware file named <code>_hostname_-firmware.bin</code> from a HTTP server on the local network. The server name can be configured in source code. After a successful download, it will install the new firmware and the restart. It also deletes the topic to 
acknowledge that it has received the command. 

To perform this kind of OTA firmware update,
1. compile your code for the `env:http` target 
2. when you "upload", the `firmware.bin` file will just be copied to your HTTP server, and renamed to <code>MySoilSensorESP32.bin</code>. 
3. publish a retained message for topic <code>soil/_hostname_/cmd</code> with 
   the payload of `update` For my device `esp32-D80270`, I run the following shell 
   comand on the home automation server: <br> 
 `mosquitto_pub -r -t "soil/esp32-D80270/cmd" -m "update"`
1. The next time the module wakes up, it will perform the update. You can tell that the update has happened when the MQTT item disappears. I watch it with a tool like [MQTT Explorer](https://mqtt-explorer.com/), or run this shell command 
   repeatedly <br/> `mosquitto_sub -t "soil/esp32-D80270/cmd"`  

The advantage of this type of OTA update is that you can build firmware for multiple devices, publish the <code>soil/_hostname_/cmd</code> topic for each device, and then go away and forget about it. Each device will update itself the next time it wakes up as scheduled.


## Debugging support

The software can print a lot of logging messages to UART#2 (GPIO pins 16 and 17), 
to which I connect an optically isolated FTDI-style USB to serial module. This 
was really helpful for understanding what goes on, and where delays occur.

The logging messages are normally suppressed. To enable logging, edit `platformio.ini`, in the general `[env]` section or an `[env:xyz]` section for a specific device, to contain
```
build_flags = 
  -D CORE_DEBUG_LEVEL=3
```

## Power consumption and battery life

### Wake time

As explained above, the time the module stays awake each measurement cycle is 
important for overall power consumption, so I investigated the wake time in some 
detail.

I built several of these units, some connected directly to the Wifi access point 
(a AVM Fritzbox 7530), and some via a Wifi repeater (a Fritzbox 7490 operating as a mesh repeater), over a distance of 3-5m, through a massive outer brick wall (or maybe through a 
window).

On average, the wake time is about **1000 ms** for all units. Now and then, maybe 
once or twice a day, the wake time is much larger, above **1500 ms**. This appeared
to happen mostly when the Wifi access point had switched to a different Wifi 
channel. When I configured my access point to use a fixed Wifi channel, these 
long wake time events became less frequent.

### Actual power consumption

These factors contribute to power consumption or battery drain, 
in decreasing order of importance:
1. the self-discharge of the AAA batteries, according to [[1]](http://www.gammon.com.au/power), 
about 35 µA or **0.84 mAh** per day 
1. wake time with WiFi ON, 140 mA x ~600ms per wakeup or **0.56 mAh** per day
1. deep sleep, at ~8 µA or **0.2 mAh** per day
1. wake time, with WiFi OFF, 40 mA x ~600ms per wakeup or **0.16 mAh** per day

This adds up to a charge of 106mC per wake period, which closely matches an actual measurement with a [Nordic Power Profiler](https://www.nordicsemi.com/Products/Development-hardware/Power-Profiler-Kit-2).

We have brought the design to the point where overall power consumption is 
dominated by the self-dischange of the Alkaline batteries, so there is no point 
in attempting to further optimize the power consumption of the processor module.

## Lessons learned

### Analog-to-digital conversion with the ESP32

The ADC on the ESP32 module has a bad reputation, there are many websites and 
forum posts that document its bad linearity and high noise level. For this 
application, the inferior performance is acceptable, because we are not attempting 
to make high precision measurements, just give an indication of "soil is ok" vs 
"soil is too dry".

### Some things are outside of my control

The performance of the device, in terms of power consumption or battery life, 
depends of course on the design of the device itself and the design decisions I 
made (processor, program flow, etc.), but also on external factors that I cannot 
control. For example, power consumption depends on wake time, which is the 
aggregate of
* the time between turning on sensor power, and measuring the sensor voltage -- 
  I chose *150ms*, which appears to be enough for the sensor voltage to settle
* the time to re-connect to the Wifi access point, using stored information 
  about WiFi channel, SSID etc -- this takes about *350-550* most of the time, 
  but sometimes can take as much as 2-3s
* the time to look up the IP address for the MQTT broker, usually *20-40ms*
* the time to connect to the MQTT broker, mostly *30ms*, but sometimes up to 
  *500ms*, which I can't explain
* the time to send MQTT messages -- I can influence this, by deciding how much 
  information to send
* the time to disconnect from the Wifi access point -- mostly about *20ms*, 
  but sometimes more than *100ms*, which I can't explain

### An almost easy project

I found it fairly easy to create a first version of the code that sort of worked 
most of the time. Then I began to notice the rough edges, and it took a lot of 
time to get those fixed:
* *easy*: making measurements and reporting them via MQTT. *Harder*: get the 
  measurements to be less jittery, and less affected by varying current 
  consumption due to WiFi activity 
* allow configuration changes via MQTT, and acknowledge the command by deleting 
  the topic. *Easy*: get it to work most of the time, with occasional loss of a 
  message. *Harder*: make it work every time, by inserting delays here and there, 
  without a massive increase in the overall wake time

### Bad luck with channel 13

When the WiFi router is set to channel 12 or 13, connection time is much higher 
(>2000ms) than on WiFi channel 1-11 (300-500ms) ... didn't expect that, should 
have read [this](https://olimex.wordpress.com/2021/12/10/avoid-wifi-channel-12-13-14-when-working-with-esp-devices/) before!
