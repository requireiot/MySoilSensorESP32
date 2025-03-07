/**
 * @file 		  MySoilSensorESP32.cpp
 *
 * Author		: Bernd Waldmann
 * Created		: 14-May-2024
 * Tabsize		: 4
 * 
 * This Revision: $Id: main.cpp 1725 2025-03-07 11:10:39Z  $
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
#include <esp32/clk.h>          // Apache-2.0 license
#include <esp_wifi.h>           // Apache-2.0 license

//----- Arduino libraries
#include <Arduino.h>            // LGPLv2.1+ license
#include <SPI.h>                // LGPLv2.1+ license
#include <WiFi.h>               // LGPLv2.1+ license
#include <ArduinoOTA.h>         // LGPLv2.1+ license, https://github.com/jandrassy/ArduinoOTA
#include <ArduinoJSON.h>        // MIT license, https://github.com/bblanchon/ArduinoJson
#include <PubSubClient.h>       // MIT license, https://github.com/knolleary/pubsubclient

//----- my headers
#include <ansi.h>
#include "myauth.h"
#include "MyWifi.h"
#include "ota.h"

/// version string published at startup.
const char VERSION[] = "$Id: main.cpp 1725 2025-03-07 11:10:39Z  $ " __DATE__ " " __TIME__;
/*                      1...5...10....5...20....5...30....5...40...5...50....5...60 */
//==============================================================================
#pragma region Preferences


#define BAT_R1 470  // from Vbat to ADC
#define BAT_R2 470  // from ADC to GND

//----- serial output
#define MY_BAUD_RATE 230400uL // 115200uL

//----- MQTT settings

#define MQTT_BROKER "ha-server"

#define SUBTOPIC_DATA "state"   // topic is soil/hostname/state
#define SUBTOPIC_INFO "debug"
#define SUBTOPIC_OTA "ota"
#define SUBTOPIC_CONFIG "config"

// measure voltage on which GPIO pins?

 #define PIN_AWAKE 23    // show that we are awake, e.g. for oscilloscope trigger
 #define PIN_BATTERY 36  // ADC used to measure battery voltage. Arduino name: A0

 /// connect these GPIO to sensor outputs
 const int pins_sensor[] = { 
    39,     // GPIO 39 = ADC1_3 = Arduino A3
    34,     // GPIO 34 = ADC1_6 = Arduino A6
    35,     // GPIO 35 = ADC1_7 = Arduino A7
    32      // GPIO 32 = ADC1_4 = Arduino A4
 //  33      // GPIO 33 = ADC1_5 = Arduino A5
    };   

 /// connect these GPIO to sensor VCC
 const int pins_power[]  = { 
    25,     // GPIO 25=ADC2_8
    26,     // GPIO 26=ADC2_9
    27,     // GPIO 27=ADC2_7
    4       // GPIO 4=ADC2_0
 //  23,     //     
    };    

const size_t NCHANNELS = sizeof pins_sensor / sizeof pins_sensor[0];    ///< no of sensor channels

/// max expected ADC value in millivolts
const unsigned ADC_RANGE_MV = 3000;
/// unconncted input with pulldown
const unsigned ALMOST_ZERO_MV = 400;
/// must have seen this much swing to consider measurement valid
const unsigned MIN_SENSOR_DYNAMIC_RANGE = ADC_RANGE_MV / 20;

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
    unsigned meas_interval;     //< interval between measurements, in seconds    
    unsigned info_interval;     //< interval between debug reports, in seconds
    unsigned powerup;           //< time to power up sensors, in milliseconds
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

#define SENSOR_RAMPUP_MS config.powerup

//------------------------------------------------------------------------------
#pragma endregion
//==============================================================================
#pragma region Global variables

bool request_ota = false;       ///< flag, enable OTA and don't go to sleep
bool firstRun = false;          ///< flag, this is not waking up from deep sleep

String hostname;
String mqttbase;
String topic_ota;
String topic_config;

//----- timing measurements to be reported
/// took this long to connect to MQTT broker [ms]
uint32_t t_mqtt;                
/// took this long to look up MQTT broker hostname
uint32_t t_dns;                 
/// how long were we awake? Remember in RTC memory and report next time
RTC_DATA_ATTR unsigned wake_duration;
/// how long did it take to disconnect from WiFi AP? Remember in RTC memory and report next time
RTC_DATA_ATTR unsigned disconnect_duration;

#ifdef PIN_BATTERY
 int battery_mV;                ///< measured battery voltage, in millivolts
#endif

String Uptime;                  ///< total uptime, including periods of sleep

/// count # of times we have woken up from sleep
RTC_DATA_ATTR uint64_t bootCount;       
/// last time battery etc was reported
RTC_DATA_ATTR time_t lastDebugReport_s; 

//----- soil moisture measurements
/// absolute measurements of sensor output, in mV
int sensor_abs[NCHANNELS];  
/// sensor output relative to observed range, in promille
int sensor_rel[NCHANNELS];  

// remember min/max values beyond sleep
struct SensorRange { int vmin; int vmax; bool valid; };
RTC_DATA_ATTR SensorRange ranges[NCHANNELS];

PubSubClient mqttClient(wifiClient);

HardwareSerial DebugSerial(2);     // on ESP32, we use UART#2 (gpio 16,17) for debug outputs
#undef Serial
#define Serial DebugSerial

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
    seconds -= minutes * 60;
    // like "999d 23:59:59"
    snprintf( buf, sizeof buf, "%ldd %ld:%02ld:%02ld", days, hours, minutes, seconds );
    return String(buf);
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

    doc["FreeHeap"] = ESP.getFreeHeap();   
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
    if (wake_duration)
        doc["wake"] = wake_duration;

    if (disconnect_duration)
        doc["dis"] = disconnect_duration;

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

    doc["mqtt"] = t_mqtt;
    doc["dns"] = t_dns;

    serializeJson(doc,msgbuf);
    return msgbuf;
}


//------------------------------------------------------------------------------
#pragma endregion
//==============================================================================
#pragma region MQTT stuff

void active_wait( unsigned ms )
{
    while (ms--) {
        delay(1);
        if (mqttClient.connected()) mqttClient.loop();
        yield();
    }
}


/**
 * @brief Callback function called by PubSubClient library when MQTT message received
 * 
 * @param topic  MQTT topic (string)
 * @param payload  MQTT payload (array of bytes)
 * @param length   length of payload
 */
void subscribeCallback(char* topic, byte* payload, unsigned length) 
{
    String spay(payload,length);

    log_i( "MQTT: received '" ANSI_BLUE "%s" ANSI_RESET "' =\n '" ANSI_BOLD "%s" ANSI_RESET "'",
        topic, spay.c_str() );

    if (0==strcmp( topic, topic_ota.c_str())) {
        if ( (spay=="on") || (spay=="ON") || (spay=="1") ) {
            request_ota = true;
        }
    } else if (0==strcmp( topic, topic_config.c_str())) {
        setConfiguration( spay.c_str() );
    }
}


/**
 * @brief Connect to MQTT broker
 * 
 * @returns true if connected
 */
bool mqttConnect() 
{
    unsigned nAttempts=0;
    uint32_t t_begin, t_end;
    const char* clientName = hostname.c_str();

    log_i("connecting as '%s' to broker '%s'", clientName, MQTT_BROKER );

    topic_ota = mqttbase + SUBTOPIC_OTA;
    topic_config = mqttbase + SUBTOPIC_CONFIG;

    t_begin = millis();
    int waitms = 100;
    for (int i=0; i<10; i++) {
        mqttClient.loop(); yield();
    	if (mqttClient.connect(clientName)) {
            t_end = millis();
            t_mqtt = t_end - t_begin;

    		log_i("connected after %u ms", t_mqtt );

            mqttClient.subscribe( topic_ota.c_str() );
            mqttClient.subscribe( topic_config.c_str() );

            return true;
    	} else {
            int st = mqttClient.state();
    		log_e("failed %d, rc=%d", i, st);
            delay(waitms);
            //waitms *= 2;
    	}
    }
    log_e( ANSI_RED "Could not connect to MQTT" ANSI_RESET );
	return false;
}


/** Try to connect or re-connect to MQTT broker, non-blocking.
	@returns true if connected
 */
bool mqttReconnect()
{
	if (mqttClient.connected()) return true;
    return mqttConnect();
}


/** Publish message to MQTT.
	@param subtopic  path appended to PUB_TOPIC_BASE to form topic
	@param message   MQTT payload, 0-terminated
    @param retain    mark as retained if `true`
 */
void mqttPublish( const char* subtopic, const char* message, bool retain=false )
{
	if (!mqttClient.connected()) return;
    yield(); mqttClient.loop(); yield();

    String topic = mqttbase + subtopic;
    if (!mqttReconnect()) return;
	if (mqttClient.publish(topic.c_str(), message, (boolean)retain)) {
        log_i("publish '%s' =\n '%s'", topic.c_str(), message ? message : "NULL" );
	} else {
		log_e("publish " ANSI_BRIGHT_RED "fail" ANSI_RESET);
	}
    active_wait(10);
}


/**
 * Publish via MQTT a JSON string with debug information: uptime, free heap etc.
 */
void mqttPublishDebug()
{
    const char* json = debug_to_json();
    mqttPublish(SUBTOPIC_INFO,json,true);
}


/**
 * Publish via MQTT a JSON string with measured values
 * 
 */
void mqttPublishState()
{
    const char* json = state_to_json();
    mqttPublish(SUBTOPIC_DATA,json);
}


/**
 * Publish via MQTT a JSON string with Wifi status
 * 
 */
void mqttPublishWifi()
{
    const char* json = wifi_to_json();
    mqttPublish("wifi",json);
}


bool dns_lookup( const char* servername, IPAddress& ip )
{
    uint32_t t_begin, t_end;
    int ret;

    t_begin = millis();
    // try DNS multiple times
    int waitms = 100;
    // try DNS request 10 times
    for (int i=0; i<10; i++) {
        ret=WiFi.hostByName(servername, ip); 
        if (1 == ret) {
            t_end = millis();
            t_dns = t_end - t_begin;       
            log_i("DNS: '%s' is %s, took " ANSI_BRIGHT_MAGENTA "%u" ANSI_RESET " ms",
                servername, ip.toString().c_str(), t_dns );
            return true;
        } else {
            log_e("failed DNS lookup, error %d",ret);
            delay(waitms);
            //waitms *= 2;            
        }
    }
    return false;
}


/**
 * Initialize and connect to MQTT server
 */
bool mqttSetup()
{
    uint32_t t_begin, t_end;

    // had some issues with slow DNS response. now doing DNS lookup as a separate step
    IPAddress ip;
    t_begin = millis();
    if (!dns_lookup(MQTT_BROKER,ip)) return false;
    t_end = millis();
    t_dns = t_end - t_begin;
    log_i("DNS: broker %s is %s (" ANSI_BRIGHT_MAGENTA "%u" ANSI_RESET " ms)", 
        MQTT_BROKER, ip.toString().c_str(), t_dns );
    mqttClient.setServer(ip, 1883);
    mqttClient.setCallback(subscribeCallback);
    mqttReconnect();
    return true;
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
    log_i("SENSOR: measure %d ",pins_sensor[ch]);

    SensorRange& r = ranges[ch];
    if (voltage < ALMOST_ZERO_MV) {
        r.valid = false;
        r.vmin = ADC_RANGE_MV;
        r.vmax = 0;
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

  	int rtc_reset_reason = rtc_get_reset_reason(0); 

    if ((config.signature != SIGNATURE) || (rtc_reset_reason != DEEPSLEEP_RESET))
        memcpy( &config, &defaultConfiguration, sizeof(Configuration) );

    if (rtc_reset_reason != DEEPSLEEP_RESET) { // we did not wake up from deep sleep
        firstRun = true;
        memset( ranges, 0, sizeof ranges );
        for (int i=0; i<NCHANNELS; i++) ranges[i].vmin = ADC_RANGE_MV;

        struct timeval tv;
        tv.tv_sec =  0;
        settimeofday(&tv, NULL);        
    }

    request_ota = false;
    bootCount++;
    uint32_t t_setup_start = millis();  // starts with 0 every time we wake up
    time_t now_s = time(NULL);          // persists across sleep episodes
    Uptime = formatUptime(now_s);
    // ignore the first 5min or so of sensor readings, to give the op time to place the sensor
    bool ignoreSensors = (now_s < (IGNORE_FIRST / 1000uL) );

    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG,0); // disable brownout detection

    DebugSerial.setPins(16,17);
    DebugSerial.begin(MY_BAUD_RATE);
    DebugSerial.setDebugOutput(true);  

    Serial.println("========================================================");

//----- measure battery voltage in mV, for later reporting via MQTT

#ifdef PIN_BATTERY
    battery_mV = analogReadMilliVolts( PIN_BATTERY ) * (BAT_R1 + BAT_R2) / BAT_R2;
#endif

//----- power up the sensors ... they need some time to warm up, so do this before Wifi

    unsigned long t_power = millis();
    powerSensors();

//----- report environment -----------------------------------------------------

    Serial.printf( "Chip:" ANSI_BOLD "%s" ANSI_RESET, 
        ESP.getChipModel() );
    Serial.printf( " at " ANSI_BOLD "%u" ANSI_RESET " MHz",
        (unsigned)ESP.getCpuFreqMHz() );
    Serial.printf( "  Flash:" ANSI_BOLD "%u" ANSI_RESET " K", 
        (unsigned)(ESP.getFlashChipSize() / 1024));
    Serial.printf( "  Core:" ANSI_BOLD "%s" ANSI_RESET, 
        esp_get_idf_version() );
    Serial.println();

    Serial.printf( "Reset:" ANSI_RED ANSI_BOLD "%d" ANSI_RESET, 
        rtc_reset_reason );
    Serial.printf( "  Heap:" ANSI_BOLD "%d" ANSI_RESET, 
        ESP.getFreeHeap() );
#ifdef PIN_BATTERY
    Serial.printf( "  Battery " ANSI_BOLD "%d" ANSI_RESET " mV",
        battery_mV );
#endif
    Serial.printf( "  Time " ANSI_BRIGHT_BLUE "%ld" ANSI_RESET "s  Uptime %s", 
        now_s, Uptime.c_str() );
    Serial.println();

//----- measure sensors

    // wait for sensor power supply
    log_i("SENSOR: wait for power-up (%u ms)", SENSOR_RAMPUP_MS);
    while ( (unsigned long)(millis()-t_power) < SENSOR_RAMPUP_MS ) {
        delay(10);
    }

    if (!ignoreSensors) {   // ignore sensor readings for the first few minutes?
        for (auto r : ranges ) 
            log_i("SENSOR: range %d:%d",r.vmin,r.vmax);
        for (int i=0; i<NCHANNELS; i++) {
            measureSensor(i);
        }
    } // if (!ignoreSensors)

    poweroffSensors();

//----- turn on Wifi -----------------------------------------------------------

    int wifiOk = setupWifi( !firstRun );
#ifdef WIFI_DBM
    esp_wifi_set_max_tx_power( WIFI_DBM * 4 );
#else
    esp_wifi_set_max_tx_power(40); // 10 dBm
#endif

//----- do MQTT stuff ----------------------------------------------------------

    if (wifiOk) {

        hostname = WiFi.getHostname();
        mqttbase = "soil/" + hostname + "/";

        if (mqttSetup()) {

            if (firstRun)
                mqttPublish("version",VERSION, true);
            
            // report bat voltage etc at first round, and then every 12h or so
            if (firstRun || 
                ( (time_t)(now_s - lastDebugReport_s) > INFO_REPORT_INTERVAL_S) 
            ) {
                lastDebugReport_s = now_s;
                mqttPublishDebug();
            } else {
                yield(); mqttClient.loop(); yield();
            }

            mqttPublishWifi();

            if (!ignoreSensors) {   // ignore sensor readings for the first few minutes?
                mqttPublishState();
            }

            if (config_changed) {
                mqttPublish( SUBTOPIC_CONFIG, NULL, true );
                active_wait(100);
            }

            if (request_ota) {
                mqttPublish( SUBTOPIC_OTA, NULL, true );
                active_wait(100);
                setupOTA(); 
            }

            mqttClient.disconnect();
            active_wait(10);
        } else {
            log_e("MQTT failed");
        }
    }

//----- wrap up basic setup ----------------------------------------------------

    uint32_t t_basic_setup_end = millis();
    log_i("setup(): " ANSI_BOLD "%u" ANSI_RESET " ms", 
        (unsigned)(t_basic_setup_end-t_setup_start)
    );

    if (request_ota) {
        Serial.print("OTA enabled. ");
        return;
    }

//----- now enter deep sleep ---------------------------------------------------

    // turn off Wifi 
    t_begin = millis();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    yield();
    t_end = millis();
    disconnect_duration = t_end - t_begin;
    log_i("Wifi disconnect took " ANSI_BRIGHT_MAGENTA "%u" ANSI_RESET " ms", 
        disconnect_duration );

    uint32_t t_setup_end = millis();
    wake_duration = t_setup_end - t_setup_start;
    log_i("Ran for " ANSI_BOLD "%u" ANSI_RESET " ms, going to sleep now.", 
        wake_duration );
    Serial.flush();

    esp_sleep_enable_timer_wakeup( SOIL_REPORT_INTERVAL_S * 1000000ull );

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
    Serial.print('.');

    delay(1000);
}

//---------------------------------------------------------------------
#pragma endregion
