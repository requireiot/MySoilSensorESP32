/**
 * @file 		  MySoilSensorESP32.cpp
 *
 * Author		: Bernd Waldmann
 * Created		: 14-May-2024
 * Tabsize		: 4
 * 
 * This Revision: $Id: main.cpp 1860 2025-09-26 20:05:18Z  $
 */

/*
   Copyright (C) 2024,2025 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public License, 
   v. 2.0. If a copy of the MPL was not distributed with this file, You can 
   obtain one at http://mozilla.org/MPL/2.0/

   SPDX-License-Identifier: MPL-2.0
*/

/**
 * @brief Multi-channel soil moisture sensor, reporting via MQTT,
 * using an ESP32 module
 */

//==============================================================================

//----- C standard headers
#include <stddef.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <time.h>
#include <stdarg.h>

//----- Espressif libraries
#include <rom/rtc.h>            // Apache-2.0 license
#include <soc/rtc_cntl_reg.h>   // Apache-2.0 license
#include <driver/rtc_io.h>      // Apache-2.0 license
#include <esp_wifi.h>           // Apache-2.0 license

//----- Arduino libraries
#include <Arduino.h>            // LGPLv2.1+ license
#include <SPI.h>                // LGPLv2.1+ license
#include <WiFi.h>               // LGPLv2.1+ license
#include <ArduinoOTA.h>         // LGPLv2.1+ license, https://github.com/jandrassy/ArduinoOTA
#include <ArduinoJSON.h>        // MIT license, https://github.com/bblanchon/ArduinoJson
#include <PubSubClient.h>       // MIT license, https://github.com/knolleary/pubsubclient
#include <HTTPUpdate.h>         // LGPLv2.1+ license

//----- my headers
#include "ansi.h"
#include "myauth.h"
#include "MyWifi.h"
#include "SimpleHttpUpdate.h"
#include "SimpleOTA.h"
#include "SimpleReports.h"
#include "SimpleMqttClient.h"


/// version string published at startup.
const char VERSION[] = "$Id: main.cpp 1860 2025-09-26 20:05:18Z  $ built " __DATE__ " " __TIME__;

#define DebugSerial Serial1

//==============================================================================
#pragma region Preferences

#define PIN_AWAKE 23    // show that we are awake, e.g. for oscilloscope trigger

//----- battery voltage measurement

#define BAT_R1 470      // from Vbat to ADC
#define BAT_R2 470      // from ADC to GND
#define PIN_BATTERY 36  // ADC used to measure battery voltage. Arduino name: A0

//----- serial output

#define MY_BAUD_RATE 115200uL

//----- MQTT settings

#define MQTT_BROKER "ha-server"
#define MQTT_TOPIC_BASE "soil/${HOSTNAME}/"
#define MQTT_PREFIX_CMND "cmd"
#define MQTT_TOPIC_SUBSCRIBE MQTT_TOPIC_BASE MQTT_PREFIX_CMND
// subtopics that we publish under
#define SUBTOPIC_DATA   "state"   // topic is soil/hostname/state
#define SUBTOPIC_INFO   "debug"
#define SUBTOPIC_DEVICE "device"
// subtopics where we receive commands from outside
#define SUBTOPIC_CONFIG "config"
#define SUBTOPIC_CMD    "cmd"
// payloads for the soil/hostname/cmd topic
#define MQTT_CMD_RESET   "reset"   // reset min/max sensor range
#define MQTT_CMD_UPDATE  "update"  // perform HTTP update
#define MQTT_CMD_OTA     "ota"     // wait for OTA update

//----- HTTP server for JSON updates
#ifndef FIRMWARE_NAME
 #define FIRMWARE_NAME "MySoilSensorESP32.bin"
#endif
#define UPDATE_SERVER "http://file-server/ota/"
#define FIRMWARE_PATH UPDATE_SERVER FIRMWARE_NAME

//----- sensor pin connections

/// connect these GPIO to sensor outputs
const int pins_sensor[] = { 
    39,     // GPIO 39 = ADC1_3 = Arduino A3
    34,     // GPIO 34 = ADC1_6 = Arduino A6
    35,     // GPIO 35 = ADC1_7 = Arduino A7
    32      // GPIO 32 = ADC1_4 = Arduino A4
    };   

/// connect these GPIO to sensor VCC
const int pins_power[]  = { 
    25,     // GPIO 25 = ADC2_8
    26,     // GPIO 26 = ADC2_9
    27,     // GPIO 27 = ADC2_7
    4       // GPIO 4  = ADC2_0
    };    

/// no of sensor channels
const size_t NCHANNELS = sizeof pins_sensor / sizeof pins_sensor[0];

/// max expected ADC value in millivolts
const unsigned ADC_RANGE_MV = 3000;
/// max voltage from an unconnected input with pulldown
const unsigned ALMOST_ZERO_MV = 400;
/// must have seen this much swing to consider measurement valid, in milllivolts
const unsigned MIN_SENSOR_DYNAMIC_RANGE = 100;

//------------------------------------------------------------------------------
#pragma endregion
//==============================================================================
#pragma region Configuration

//----- Timing phrases
#define SECONDS		
#define MINUTES 	* 60uL SECONDS
#define HOURS 		* 60uL MINUTES
#define DAYS		* 24uL HOURS

#define SIGNATURE 0xDEADBEEF

struct Configuration {
    unsigned signature;
    unsigned meas_interval;     // interval between measurements [s]
    unsigned info_interval;     // interval between debug reports [s]
    unsigned powerup;           // time to power up sensors [ms]
};

const Configuration defaultConfiguration = {
    .signature = SIGNATURE,
    .meas_interval = 30 MINUTES,
    .info_interval = 6 HOURS,
    .powerup = 150
};

RTC_DATA_ATTR Configuration config;

bool config_changed = false;

void setConfiguration( const char* json )
{
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, json);
    if (error) return;
    config.info_interval = doc["info"] | config.info_interval;
    config.meas_interval = doc["meas"] | config.meas_interval;
    config.powerup = doc["powerup"] | config.powerup;
    config_changed = true;
}


//------------------------------------------------------------------------------
#pragma endregion
//==============================================================================
#pragma region Timing

/*
    Delays and measurement interval are specified in milliseconds
    an int32_t can represent up to 25 days in milliseconds

    Uptime and longer intervals are specified in seconds
    an int32_t can represent up to 68 years in seconds
*/
//#define QUICK

#ifdef QUICK    // shortened timing, for debugging only, may be defined in myauth.h
 // minimum time between refreshes
 const unsigned long SOIL_REPORT_INTERVAL_S = 30 SECONDS;  
 const unsigned long INFO_REPORT_INTERVAL_S = 60 SECONDS;
  // ignore the first few minutes, before th user places the sensor in the soil
  const unsigned long IGNORE_FIRST           = 30 SECONDS;
#else
 // minimum time between refreshes
 #define SOIL_REPORT_INTERVAL_S config.meas_interval
 #define INFO_REPORT_INTERVAL_S config.info_interval
  // ignore the first few minutes, before the user places the sensor in the soil
  const unsigned long IGNORE_FIRST = 5 MINUTES;
#endif
#define CYCLES_PER_FRESH_CONNECT (1 DAYS / SOIL_REPORT_INTERVAL_S)

#define SENSOR_RAMPUP_MS config.powerup

#define DNS_WAIT_MS   100   // how long to wait for DNS response [ms]
#define DNS_RETRY       3   // how many times to retry DNS request

#define MQTT_WAIT_MS  100   // how long to wait for connect to MQTT broker [ms]
#define MQTT_RETRY      3   // how many times to retry connect to MQTT broker

//------------------------------------------------------------------------------
#pragma endregion
//==============================================================================
#pragma region Global variables

bool request_ota = false;       // flag, enable OTA and don't go to sleep
bool request_update = false;    // flag, perform HTTP update
bool request_reset = false;     // flag, reset measurement range
bool firstRun = false;          // flag, this is not waking up from deep sleep

//----- timing measurements to be reported
/// took this long to connect to MQTT broker [ms]
uint32_t dur_mqtt;                
/// took this long to look up MQTT broker hostname [ms]
uint32_t dur_dns;                 
/// how long were we awake? Remember in RTC memory and report next time [ms]
RTC_DATA_ATTR unsigned dur_wake;
/// how long did it take to disconnect from WiFi AP? Remember in RTC memory and report next time [ms]
RTC_DATA_ATTR unsigned dur_disconnect;
/// how many times did we fail to connect, since last successful connection?
RTC_DATA_ATTR unsigned lastCycleFailed;

#ifdef PIN_BATTERY
 unsigned battery_mV;                ///< measured battery voltage, in millivolts
#endif

String Uptime;                  ///< total uptime, including periods of sleep

/// count # of times we have woken up from sleep
RTC_DATA_ATTR unsigned bootCount;       
/// last time battery etc was reported
RTC_DATA_ATTR time_t lastDebugReport_s; 

//----- soil moisture measurements
/// absolute measurements of sensor output [mV]
int sensor_abs[NCHANNELS];  
/// sensor output relative to observed range [promille]
int sensor_rel[NCHANNELS];  

// remember min/max values beyond sleep
struct SensorRange { 
    int vmin; 
    int vmax; 
    bool valid; 
    void reset() { vmin=ADC_RANGE_MV; vmax=0; valid=false; }
};
RTC_DATA_ATTR SensorRange ranges[NCHANNELS];

PubSubClient mqttClient(wifiClient);
SimpleMqttClient simpleMqttClient(mqttClient,MQTT_BROKER);

//------------------------------------------------------------------------------
#pragma endregion
//==============================================================================
#pragma region Little helpers

String formatUptime( time_t now )
{
    char buf[20];

    long seconds = now; 
    long days = seconds / (24 * 60 * 60L);
    seconds -= days * (24 * 60 * 60L);
    long hours = seconds / (60 * 60L);
    seconds -= hours * (60 * 60L);
    long minutes = seconds / 60;
    // like "999d 23:59"
    snprintf( buf, sizeof buf, "%ldd %ld:%02ld", days, hours, minutes );
    return String(buf);
}


/**
 * @brief get IP address from server name, via DNS
 * 
 * @param servername    name to be looked up
 * @param ip            returns IP address here
 * @return true         if lookup successful
 * @return false        if failed
 * 
 * As a side effect update `dur_dns` global variable
 */
bool dns_lookup( const char* servername, IPAddress& ip )
{
    uint32_t t_begin, t_end;
    int ret;

    t_begin = millis();
    for (int i=0; i<DNS_RETRY; i++) {
        ret=WiFi.hostByName(servername, ip); 
        if (1 == ret) {
            t_end = millis();
            dur_dns = t_end - t_begin;       
            log_i("'%s' is %s (" ANSI_BRIGHT_MAGENTA "%u" ANSI_RESET " ms)",
                servername, ip.toString().c_str(), dur_dns );
            return true;
        } else {
            log_e("failed DNS lookup, error %d",ret);
            delay(DNS_WAIT_MS);   
        }
    }
    return false;
}

//------------------------------------------------------------------------------
#pragma endregion
//==============================================================================
#pragma region JSON reports


char msgbuf[256];

/**
 * @brief Create JSON record with debug information for publishing via MQTT
 * 
 * @return const char*  Pointer to static buffer with JSON string
 */
 const char* debug_to_json()
{
    JsonDocument doc;

    doc["Boot"] = bootCount;
#ifdef PIN_BATTERY
    doc["Battery"] = battery_mV;
#endif
    doc["Uptime"] = Uptime.c_str();

    serializeJson(doc,msgbuf);
    return msgbuf;
}


/**
 * @brief Create JSON record with measured parameters for publishing via MQTT
 * 
 * @return const char*  Pointer to static buffer with JSON string
 */
const char* state_to_json()
{
    JsonDocument doc;
    JsonArray vabs = doc["abs"].to<JsonArray>();
    JsonArray vrel = doc["rel"].to<JsonArray>();
    for (int i=0; i<NCHANNELS; i++) {
        vabs.add(sensor_abs[i]);
        vrel.add(sensor_rel[i]);
    }
    if (dur_wake)
        doc["wake"] = dur_wake;

    if (dur_disconnect)
        doc["dis"] = dur_disconnect;

    serializeJson(doc,msgbuf);
    return msgbuf;
}


/**
 * @brief Create JSON record with information about WiFi timing, for publishing via MQTT
 * 
 * @return const char*  Pointer to static buffer with JSON string
 */
const char* wifi_to_json()
{
    JsonDocument doc;
   
    reportWifi( doc );
    simpleMqttClient.report(doc);
    doc["fail"] = lastCycleFailed;

    serializeJson(doc,msgbuf);
    return msgbuf;
}


/**
 * @brief Create debug record for publishing via MQTT,
 * with static information about device
 * 
 * @return const char*  Pointer to static buffer with JSON string
 */
const char* device_to_json()
{
    JsonDocument doc;
    unsigned stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

    doc["SDK"] = ESP.getSdkVersion();
    //doc["Core"] = ESP.getCoreVersion();
    doc["Heap"] = ESP.getFreeHeap();   
    doc["Stack"] = stackHighWaterMark;

    serializeJson(doc,msgbuf);
    return msgbuf;
}


//------------------------------------------------------------------------------
#pragma endregion
//==============================================================================
#pragma region MQTT stuff

/*
    publish to haus/HOSTNAME/ 
    path                  | content                         |  publish interval
    ----------------------+---------------------------------+------------------
    haus/HOSTNAME/version | version string from SVN         |  once
    haus/HOSTNAME/device  | static info about CPU, Code etc |  once
    haus/HOSTNAME/wifi    | performance parameters          |  every cycle
    haus/HOSTNAME/debug   | device status                   |  every 6h

    subscribe to haus/HOSTNAME/cmd
    payload                 effect
    ---------------------------------------------------------------------------
    reset                   reset ADC ranges
    ota                     do not go to sleep, wait for OTA update
    update                  perform firmware update from HTTP server
    {"key":"val"}           change configuration option `key` to `val`
*/

void active_wait( unsigned ms )
{
    while (ms--) {
        delay(1);
        if (mqttClient.connected()) mqttClient.loop();
    }
}


void mqttPublish( const char* subtopic, const char* message, bool retain=false )
{
    simpleMqttClient.publish(subtopic, message, retain);
}


void onReceive( const char* subtopic, const char* message )
{
    log_i( "MQTT: received '" ANSI_BLUE "%s" ANSI_RESET "' = '" ANSI_BOLD "%s" ANSI_RESET "'",
        subtopic ? subtopic : "(blank)", 
        message ? message : "(blank)" );

    // is it a empty payload? ... just deleting a retained topic, ignore
    if (message==NULL || *message==0) return;

    if (message[0]=='{') {  // looks like a JSON string, must be a config message
        setConfiguration( message );
    } else if (!strcmp(message,MQTT_CMD_OTA)) {
        request_ota = true;
    } else if (!strcmp(message,MQTT_CMD_UPDATE)) {
        request_update = true;
    } else if (!strcmp(message,MQTT_CMD_RESET)) {
        request_reset = true;
    }
    // acknowledge by deleting retained message
    mqttPublish( MQTT_PREFIX_CMND, NULL, true ); 
}


bool mqttSetup()
{
    bool ok = simpleMqttClient.begin( MQTT_TOPIC_SUBSCRIBE, MQTT_TOPIC_BASE );
    if (ok) {
        simpleMqttClient.setCallback(onReceive);
        active_wait(10);
    }
    return ok;
}

//------------------------------------------------------------------------------
#pragma endregion
//==============================================================================
#pragma region ADC measurement


// qsort requires you to create a sort function
int sorter(const void *cmp1, const void *cmp2)
{
  // Need to cast the void * to int *
  unsigned a = *((unsigned *)cmp1);
  unsigned b = *((unsigned *)cmp2);
  // The comparison
  return a > b ? -1 : (a < b ? 1 : 0);
}


/**
 * @brief Read ADC, and do some primitive noise reduction by averaging multiple
 * samples
 * 
 * @param pin   ESP32 pin number
 * @return unsigned   measured voltage in millivolts
 */
unsigned readSensor( int pin )
{
    unsigned voltage;
    const unsigned NSAMPLES = 20;
    unsigned samples[NSAMPLES];

    // ignore 1st measurement
    voltage = analogReadMilliVolts(pin);
    // take multiple samples
    for (int i=0; i<NSAMPLES; i++)
        samples[i] = analogReadMilliVolts(pin);
    // sort
    qsort( samples, NSAMPLES, sizeof(samples[0]), sorter );
    // average all measurements except for highest and lowest
    unsigned sum=0;
    for (int i=1; i<NSAMPLES-1; i++)
        sum += samples[i];
    voltage = sum / (NSAMPLES-2);

    return voltage;
}


/**
 * @brief take measurement for a single sensor, store result in global vars
 * 
 * @param ch logical channel no [0..3]
 */
void measureSensor( int ch )
{
    unsigned voltage = readSensor(pins_sensor[ch]);
    sensor_abs[ch] = voltage;
    log_i("ch %d = %u mV [range %d .. %d] ",
        pins_sensor[ch], voltage, ranges[ch].vmin, ranges[ch].vmax );

    SensorRange& r = ranges[ch];
    if (voltage < ALMOST_ZERO_MV) {
        r.reset();
    } else {
        if (voltage > r.vmax)
            r.vmax = voltage;
        if (voltage < r.vmin)
            r.vmin = voltage;
        r.valid = 
            ((r.vmax - r.vmin) > MIN_SENSOR_DYNAMIC_RANGE)
            && (r.vmin < ADC_RANGE_MV)
            && (r.vmax > 0)
            ;
    }

    sensor_rel[ch] = (r.valid) ? 1000 - (1000 * (voltage-r.vmin) / (r.vmax-r.vmin)) : -1;
}


/**
 * @brief Turn ON power for all 4 sensors
 * 
 */
void powerSensors()
{
    for (auto pin : pins_power) {
        pinMode( pin, OUTPUT );
        digitalWrite( pin, HIGH );
    }
}


/**
 * @brief Turn OFF power for all 4 sensors
 * 
 */
void poweroffSensors()
{
    for (auto pin : pins_power) {
        digitalWrite( pin, LOW );
    }
    delay(10);
    for (auto pin : pins_power) {
        pinMode(pin,INPUT);
    }
}

//------------------------------------------------------------------------------
#pragma endregion
//==============================================================================
#pragma region Arduino standard functions

/**
 * @brief Standard Arduino framework function, called after power-on 
 * or after waking up from deep sleep.
 * 
 */
void setup()
{
    uint32_t t_begin, t_end;

//----- signal start of wake phase (oscilloscope trigger)
#ifdef PIN_AWAKE
    pinMode( PIN_AWAKE, OUTPUT );
    digitalWrite( PIN_AWAKE, HIGH );
#endif

    Serial1.begin(MY_BAUD_RATE,SERIAL_8N1,16,17);
    Serial1.setDebugOutput(true);  

  	int rtc_reset_reason = rtc_get_reset_reason(0); 

    if ((config.signature != SIGNATURE) || (rtc_reset_reason != DEEPSLEEP_RESET))
        memcpy( &config, &defaultConfiguration, sizeof(Configuration) );
  
    if (rtc_reset_reason != DEEPSLEEP_RESET) { // we did not wake up from deep sleep
        lastCycleFailed = 0;
        firstRun = true;
        memset( ranges, 0, sizeof ranges );
        for (int i=0; i<NCHANNELS; i++) ranges[i].vmin = ADC_RANGE_MV;

        struct timeval tv;
        tv.tv_sec =  0;
        settimeofday(&tv, NULL);        
    }

    request_ota = false;
    uint32_t t_setup_start = millis();  // starts with 0 every time we wake up
    time_t now_s = time(NULL);          // persists across sleep episodes
    if (bootCount==0) now_s=0;
    Uptime = formatUptime(now_s);
    bootCount++;
    // ignore the first 5min or so of sensor readings, to give the op time to place the sensor
    bool ignoreSensors = (now_s < (IGNORE_FIRST / 1000uL) );

    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG,0); // disable brownout detection

    DebugSerial.println("====================================================");

//----- measure battery voltage in mV, for later reporting via MQTT

#ifdef PIN_BATTERY
    battery_mV = analogReadMilliVolts( PIN_BATTERY ) * (BAT_R1 + BAT_R2) / BAT_R2;
#endif

//----- power up the sensors ... they need some time to warm up, so do this before Wifi

    unsigned long t_power = millis();
    powerSensors();

//----- report environment -----------------------------------------------------

    log_i( 
        "Chip:" ANSI_BOLD "%s" ANSI_RESET
        " at " ANSI_BOLD "%u" ANSI_RESET " MHz"
        "  Flash:" ANSI_BOLD "%u" ANSI_RESET " K"
        , 
        ESP.getChipModel(),
        (unsigned)ESP.getCpuFreqMHz(),
        (unsigned)(ESP.getFlashChipSize() / 1024)
    );
    log_i(
        "SDK:" ANSI_BOLD "%s" ANSI_RESET
        "  Arduino " ANSI_BOLD "%d.%d.%d" ANSI_RESET
        ,
        ESP.getSdkVersion(),
        ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH
    );
    log_i(
        "Reset:" ANSI_RED ANSI_BOLD "%d" ANSI_RESET
        "  Heap:" ANSI_BOLD "%d" ANSI_RESET
        ,
        rtc_reset_reason,
        ESP.getFreeHeap()
    );
#ifdef PIN_BATTERY
    log_i( "Battery " ANSI_BOLD "%d" ANSI_RESET " mV",
        battery_mV );
#endif

//----- measure sensors

    log_i("SENSOR: wait for power-up (%u ms)", SENSOR_RAMPUP_MS);
    while ( (unsigned long)(millis()-t_power) < SENSOR_RAMPUP_MS ) {
        delay(10);
    }

    if (!ignoreSensors) {   // ignore sensor readings for the first few minutes?
        for (int i=0; i<NCHANNELS; i++) {
            measureSensor(i);
        }
    } 

    poweroffSensors();

//----- turn on Wifi -----------------------------------------------------------

    bool allow_reconnect = true;
    if ((bootCount % CYCLES_PER_FRESH_CONNECT) == 0)  // once every 24h, do a fresh connect
        allow_reconnect = false;
    if (firstRun)
        allow_reconnect = false;
    if (lastCycleFailed)    
        allow_reconnect = false;

    int wifiOk = setupWifi( WIFI_SSID, WIFI_PASSWORD, allow_reconnect );

//----- do MQTT stuff ----------------------------------------------------------

    if (wifiOk) {
        if (mqttSetup()) {

            if (firstRun) mqttPublish("version",VERSION, true);
            
                // report bat voltage etc at first round, and then every 12h or so
            if (firstRun || 
                ( (time_t)(now_s - lastDebugReport_s) > INFO_REPORT_INTERVAL_S) 
            ) {
                lastDebugReport_s = now_s;
                mqttPublish( SUBTOPIC_INFO, debug_to_json(), true );
            }

            mqttPublish( "wifi", wifi_to_json() );        

            if (!ignoreSensors) {   // ignore sensor readings for the first few minutes?
                mqttPublish( SUBTOPIC_DATA, state_to_json() );
            }

            if (bootCount==2) {
                mqttPublish(SUBTOPIC_DEVICE,device_to_json(),true);
            }

            if (request_ota) 
                setupOTA(DebugSerial); 

            mqttClient.disconnect();
            active_wait(10);

            lastCycleFailed = 0;
        } else {
            lastCycleFailed++;
            log_e("MQTT failed");
        }
    } else {
        lastCycleFailed++;
    }

//----- wrap up basic setup ----------------------------------------------------

    uint32_t t_basic_setup_end = millis();
    log_i("setup(): " ANSI_BOLD "%u" ANSI_RESET " ms", 
        (unsigned)(t_basic_setup_end-t_setup_start)
    );

    if (request_ota) {
        log_i("OTA enabled. ");
        return;
    }
    if (request_reset) {
        log_i("Resetting sensor ranges");
        request_reset = false;
        for (auto r: ranges) r.reset();
    }
    if (request_update) {
        request_update = false;
        do_httpUpdate(wifiClient, FIRMWARE_PATH, DebugSerial);
    }

//----- now enter deep sleep ---------------------------------------------------

    // turn off Wifi 
    t_begin = millis();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    yield();
    t_end = millis();
    dur_disconnect = t_end - t_begin;
    log_i("Wifi disconnect (" ANSI_BRIGHT_MAGENTA "%u" ANSI_RESET " ms)", 
        dur_disconnect );

    uint32_t t_setup_end = millis();
    dur_wake = t_setup_end - t_setup_start;
    DebugSerial.printf("Ran for " ANSI_BOLD "%u" ANSI_RESET " ms, going to sleep now.\n", 
        dur_wake );
    DebugSerial.flush();

    unsigned sleeptime_s = lastCycleFailed ? 300 : SOIL_REPORT_INTERVAL_S;
    esp_sleep_enable_timer_wakeup( sleeptime_s * 1'000'000uLL );

    // signal end of wake phase (oscilloscope trigger)
#ifdef PIN_AWAKE
    digitalWrite( PIN_AWAKE, LOW );
    pinMode( PIN_AWAKE, INPUT );
#endif

    esp_deep_sleep_start();
    // module goes into deep sleep here
}


void loop()
{
    if (!request_ota) return;
    if (loopWifi()) return;
	ArduinoOTA.handle();
    DebugSerial.print('.');

    delay(1000);
}

//---------------------------------------------------------------------
#pragma endregion
