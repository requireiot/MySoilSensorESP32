/**
 * @file 		  MySoilSensorESP32.cpp
 *
 * Author		: Bernd Waldmann
 * Created		: 14-May-2024
 * Tabsize		: 4
 * 
 * This Revision: $Id: MySoilSensorESP32.cpp 1620 2024-08-05 14:46:56Z  $
 */

/*
   Copyright (C) 2024 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public License, 
   v. 2.0. If a copy of the MPL was not distributed with this file, You can 
   obtain one at http://mozilla.org/MPL/2.0/

   SPDX-License-Identifier: MPL-2.0
*/

/**
 * @brief Multi-channel soil moisture sensor, reporting via MQTT,
 * using an ESP32 module
 * 
 */

//==============================================================================

//----- C standard headers
#include <stddef.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <time.h>
#include <stdarg.h>

//----- Arduino and libraries
#include <Arduino.h>            // LGPLv2.1+ license
#include <SPI.h>                // LGPLv2.1+ license
#include <WiFi.h>               // LGPLv2.1+ license

#include <rom/rtc.h>            // Apache-2.0 license
#include <soc/rtc_cntl_reg.h>   // Apache-2.0 license
#include <driver/rtc_io.h>      // Apache-2.0 license
#include <esp32/clk.h>          // Apache-2.0 license
#include <esp_wifi.h>           // Apache-2.0 license

#include <ArduinoOTA.h>         // LGPLv2.1+ license, https://github.com/jandrassy/ArduinoOTA
#include <ArduinoJSON.h>        // MIT license, https://github.com/bblanchon/ArduinoJson
#include <PubSubClient.h>       // MIT license, https://github.com/knolleary/pubsubclient

//----- my headers
#include <ansi.h>
#include "myauth.h"
#include "MyWifi.h"
#include "ota.h"

//==============================================================================
#pragma region Preferences

/*
    This supports ESP32 and ESP32C3 based boards
    - tested with ESP32-WROOM-32E
        platformio: board=esp32dev
        conditional #ifdef ARDUINO_ESP32_DEV 
        conditional #if CONFIG_IDF_TARGET_ESP32 
        conditional #ifdef __XTENSA__
    - WIP with ESP32C3 supermini, 
        platformio: board=lolin_c3_mini
        conditional #ifdef ARDUINO_LOLIN_C3_MINI
        conditional #if CONFIG_IDF_TARGET_ESP32C3 
        conditional #ifdef __riscv
*/

#define BAT_R1 470  // from Vbat to ADC
#define BAT_R2 470  // from ADC to GND

//----- serial output
#define MY_BAUD_RATE 230400uL // 115200uL

//----- MQTT settings

#define MQTT_BROKER "ha-server"

/// version string published at startup.
const char VERSION[] = "$Id: MySoilSensorESP32.cpp 1620 2024-08-05 14:46:56Z  $";

/*                      1...5...10....5...20....5...30....5...40...5...50 */

// measure voltage on which GPIO pins?

#if CONFIG_IDF_TARGET_ESP32

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
#endif // CONFIG_IDF_TARGET_ESP32

#if CONFIG_IDF_TARGET_ESP32C3

 #define PIN_AWAKE 7    // show that we are awake, e.g. for oscilloscope trigger
 #define PIN_BATTERY 0  // ADC used to measure battery voltage. Arduino name: A0

/// connect these GPIO to sensor outputs
 const int pins_sensor[] = { 1,2,3,4 }; // opt 5
/// connect these GPIO to sensor VCC
 const int pins_power[]  = { 10 }; 
#endif // CONFIG_IDF_TARGET_ESP32C3

const size_t NCHANNELS = sizeof pins_sensor / sizeof pins_sensor[0];    ///< no of sensor channels

/// max expected ADC value in millivolts
const unsigned ADC_RANGE_MV = 3000;
/// unconncted input with pulldown
const unsigned ALMOST_ZERO_MV = 200;
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
    unsigned nsamples;          //< how many samples to average per ADC channel
    unsigned powerup;           //< time to power up sensors, in milliseconds
};

const Configuration defaultConfiguration = {
    .signature = SIGNATURE,
    .meas_interval = 60 MINUTES,
    .info_interval = 12 HOURS,
    .nsamples = 20,
    .powerup = 200
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
    config.nsamples = doc["samples"] | config.nsamples;
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

bool request_ota = false;   ///< flag, enable OTA and don't go to sleep
bool firstRun = false;      ///< flag, this is not waking up from deep sleep

String hostname;
String mqttbase;
String topic_ota;
String topic_config;

#ifdef PIN_BATTERY
 int battery_mV;            ///< measured battery voltage, in millivolts
#endif
String Uptime;              ///< total uptime, including periods of sleep

RTC_DATA_ATTR uint64_t bootCount;       ///< count # of times we have woken up from sleep
RTC_DATA_ATTR time_t lastDebugReport_s; ///< time is s of last time battery etc was reported
// how long were we awake? Remember in RTC memory and report next time
RTC_DATA_ATTR unsigned wake_duration;

//----- measurements
int sensor_abs[NCHANNELS];  ///< absolute measurements of sensor output, in mV
int sensor_rel[NCHANNELS];  ///< sensor output relative to observed range, in promille

// remember min/max values beyond sleep
struct SensorRange { int vmin; int vmax; bool valid; };
RTC_DATA_ATTR SensorRange ranges[NCHANNELS];

PubSubClient mqttClient(wifiClient);

#if CONFIG_IDF_TARGET_ESP32
 HardwareSerial DebugSerial(2);     // on ESP32, we use UART#2 for debug outputs
#endif // CONFIG_IDF_TARGET_ESP32

#if CONFIG_IDF_TARGET_ESP32C3
 HardwareSerial DebugSerial(0);     /// on ESP32-C3, we use hardware UART#0 for debug outputs
#endif // CONFIG_IDF_TARGET_ESP32C3

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
 * @brief Create debug record for publishing via MQTT
 * 
 * @return const char*  Pointer to static buffer with JSON string
 * @code
    {"FreeHeap":246456,"Boot":1,"Battery":3034,"Uptime":"999d 0:00:00","RSSI":90}
    1...5...10...15...20...25...30...35...40...45...50...55...60...65...70...75...80...85...90
   @endcode
 */
const char* debug_to_json()
{
    JsonDocument doc;

    int dBm = WiFi.RSSI();        // in dBm
    int quality;
    if (dBm <= -100) {
        quality = 0;
    } else if (dBm >= -50) {
        quality = 100;
    } else {
        quality = 2 * (dBm + 100);    
    }

    doc["FreeHeap"] = ESP.getFreeHeap();   
    doc["Boot"] = bootCount;
#ifdef PIN_BATTERY
    doc["Battery"] = battery_mV;
#endif
    doc["Uptime"] = Uptime.c_str();
    doc["RSSI"] = quality;

    serializeJson(doc,msgbuf);
    return msgbuf;
}


/**
 * @brief Each cycle, prepare JSON report on measured parameters
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

    serializeJson(doc,msgbuf);
    return msgbuf;
}


const char* wifi_to_json()
{
    reportWifi( msgbuf, sizeof msgbuf );
    return msgbuf;
}


//------------------------------------------------------------------------------
#pragma endregion
//==============================================================================
#pragma region MQTT stuff

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

    Serial.printf(
        "\n%5ld MQTT: "
        "received '" ANSI_BLUE "%s" ANSI_RESET "'"
        " = '" ANSI_BOLD "%s" ANSI_RESET "'\n",
        millis(), topic, spay.c_str() );
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

    const char* clientName = hostname.c_str();
    Serial.printf(
        "%5ld MQTT: connecting as '" ANSI_BLUE "%s" ANSI_RESET 
        "' to broker '" ANSI_BLUE "%s" ANSI_RESET "'\n", 
        millis(), clientName, MQTT_BROKER );

    topic_ota = mqttbase + "ota";
    topic_config = mqttbase + "config";

    int waitms = 100;
    for (int i=0; i<10; i++) {
        mqttClient.loop(); yield();
    	if (mqttClient.connect(clientName)) {
    		Serial.printf( "%5ld MQTT: " ANSI_BRIGHT_GREEN "connected\n" ANSI_RESET, millis());

            mqttClient.subscribe( topic_ota.c_str() );
            Serial.printf(
                "%5ld MQTT: subscribed to " ANSI_BLUE "%s" ANSI_RESET "\n", 
                millis(), topic_ota.c_str() );

            mqttClient.subscribe( topic_config.c_str() );
            Serial.printf(
                "%5ld MQTT: subscribed to " ANSI_BLUE "%s" ANSI_RESET "\n", 
                millis(), topic_config.c_str() );

            return true;
    	} else {
            int st = mqttClient.state();
    		Serial.printf("%5ld MQTT: failed %d, rc=%d\n", millis(), i, st);
            delay(waitms);
            waitms *= 2;
    	}
    }
    Serial.println( ANSI_RED "Could not connect to MQTT" ANSI_RESET );
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
    mqttReconnect();
	if (mqttClient.publish(topic.c_str(), message, (boolean)retain)) {
        Serial.printf(
            "%5ld MQTT: publish '" ANSI_BLUE "%s" ANSI_RESET "' = \n"
            "'" ANSI_BLUE "%s" ANSI_RESET "'\r\n", 
            millis(), topic.c_str(), message);
	} else {
		Serial.print("MQTT: publish " ANSI_BRIGHT_RED "fail" ANSI_RESET "\n");
	}
    yield(); mqttClient.loop(); yield();
}


/**
 * Publish via MQTT a JSON string with debug information: uptime, free heap etc.
 */
void mqttPublishDebug()
{
    const char* json = debug_to_json();
    mqttPublish("debug",json,true);
}


/**
 * Publish via MQTT a JSON string with measured values
 * 
 */
void mqttPublishState()
{
    const char* json = state_to_json();
    mqttPublish("state",json);
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


/**
 * Initialize and connect to MQTT server
 */
void mqttSetup()
{
    // had some issues with slow DNS response. now doing DNS lookup as a separate step
    Serial.printf("%5ld MQTT: DNS lookup\n",millis());
    IPAddress ip;
    int ret = WiFi.hostByName(MQTT_BROKER, ip);
    while (1 != ret) {
        Serial.printf("%5ld failed DNS lookup, error %d\n",millis(),ret);
        delay(100);
        ret = WiFi.hostByName(MQTT_BROKER, ip);
    }
    Serial.printf("%5ld MQTT: broker " 
        ANSI_BLUE "%s" ANSI_RESET " is " 
        ANSI_BOLD "%s" ANSI_RESET "\n",
        millis(), MQTT_BROKER, ip.toString().c_str());
    mqttClient.setServer(ip, 1883);
    mqttClient.setCallback(subscribeCallback);
    mqttReconnect();
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
unsigned measureSensor( int pin )
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
    voltage = sum/ (NSAMPLES-2);

    return voltage;
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

#if CONFIG_IDF_TARGET_ESP32
    DebugSerial.setPins(16,17);
#endif // CONFIG_IDF_TARGET_ESP32

#if CONFIG_IDF_TARGET_ESP32C3
 	DebugSerial.setPins(20,21);
#endif // CONFIG_IDF_TARGET_ESP32C3

    DebugSerial.begin(MY_BAUD_RATE);
    Serial.setDebugOutput(true);  

    Serial.println("========================================================");

//----- measure battery voltage in mV, for later rerporting via MQTT

#ifdef PIN_BATTERY
    battery_mV = analogReadMilliVolts( PIN_BATTERY ) * (BAT_R1 + BAT_R2) / BAT_R2;
#endif

//----- report environment -----------------------------------------------------

    Serial.printf( "Reset: RTC=" ANSI_RED ANSI_BOLD "%d" ANSI_RESET, 
        rtc_reset_reason );
    Serial.printf( "  Heap:" ANSI_BOLD "%d" ANSI_RESET, 
        ESP.getFreeHeap() );
#ifdef PIN_BATTERY
    Serial.printf( "  Battery " ANSI_BOLD "%d" ANSI_RESET "mV",
        battery_mV );
#endif
    Serial.printf( "  Time " ANSI_BRIGHT_BLUE "%ld" ANSI_RESET "s  Uptime %s", 
        now_s, Uptime.c_str() );
    Serial.println();

    Serial.printf( " Chip:" ANSI_BOLD "%s" ANSI_RESET, 
        ESP.getChipModel() );
    Serial.printf( " at " ANSI_BOLD "%d" ANSI_RESET "MHz",
        (int)(esp_clk_cpu_freq()/1000000));
    Serial.printf( "  Flash:" ANSI_BOLD "%d" ANSI_RESET "K", 
        (int)(ESP.getFlashChipSize() / 1024));
    Serial.printf( "  Core:" ANSI_BOLD "%s" ANSI_RESET, 
        esp_get_idf_version() );
    Serial.println();

    yield();

//----- power up the sensors ... they need some time to warm up, so do this before Wifi

    unsigned long t_power = millis();
    //Serial.printf("SENSOR: at %ld power up ",t_power);
    Serial.printf("%5ld SENSOR: power-up, ",t_power);
    for (auto pin : pins_power) {
        pinMode( pin, OUTPUT );
        digitalWrite( pin, HIGH );
        Serial.printf("%d ",pin);
    }
    Serial.println();

//----- turn on Wifi -----------------------------------------------------------

    int wifiOk = setupWifi( !firstRun );
    esp_wifi_set_max_tx_power(40); // 10 dBm

//----- do MQTT stuff ----------------------------------------------------------

    if (wifiOk) {

        hostname = WiFi.getHostname();
        mqttbase = "soil/" + hostname + "/";

        mqttSetup();
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

//----- measure sensors

        if (!ignoreSensors) {   // ignore sensor readins for the first few minutes?
            Serial.printf("%5ld SENSOR: wait for power-up, ",millis());

            // wait for sensor power supply
            mqttClient.loop(); yield();
            while ( (unsigned long)(millis()-t_power) < SENSOR_RAMPUP_MS ) {
                yield(); mqttClient.loop(); yield();
                delay(50);
            }
            Serial.print("range:"); 
            for (auto r : ranges ) Serial.printf(" %d:%d",r.vmin,r.vmax);
            Serial.println();

            Serial.printf("%5d SENSOR: measure ",millis());
            for (int i=0; i<NCHANNELS; i++) {
                yield(); mqttClient.loop(); yield();  

                //unsigned voltage = analogReadMilliVolts(pins_sensor[i]);
                unsigned voltage = measureSensor(pins_sensor[i]);

                sensor_abs[i] = voltage;
                Serial.printf("%d ",pins_sensor[i]);
                SensorRange& r = ranges[i];
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
                if (r.valid) {
                    sensor_rel[i] = 1000 - (1000 * (voltage - r.vmin) / (r.vmax - r.vmin));
                } else {
                    sensor_rel[i] = -1;
                }
                yield(); mqttClient.loop(); yield();
            }
        } // if (!ignoreSensors)
        
        //----- turn off power to sensors

        Serial.printf("\n%5ld SENSOR: power down sensors ",millis());
        for (int pin : pins_power) {
            digitalWrite( pin, LOW );
        }
        delay(10);
        for (int pin : pins_power) {
            pinMode(pin,INPUT);
            Serial.printf("%d ",pin);
        }
        Serial.println();

        mqttPublishState();

        delay(100); yield(); mqttClient.loop(); yield();

        if (config_changed) {
            mqttClient.publish( topic_config.c_str(), NULL, (boolean)true );
            yield(); mqttClient.loop(); yield();
        }

        if (request_ota) {
            mqttPublish("ota","OFF",true);
            setupOTA(); 
        }
        mqttClient.disconnect();
    }

//----- wrap up basic setup ----------------------------------------------------

    uint32_t t_basic_setup_end = millis();
    Serial.printf(
        "setup(): " ANSI_BOLD "%u" ANSI_RESET " ms. ", 
        (unsigned)(t_basic_setup_end-t_setup_start)
    );

    if (request_ota) {
        Serial.print("OTA enabled. ");
        return;
    }

    Serial.flush();
    yield();

//----- now enter deep sleep ---------------------------------------------------

    // turn off Wifi 
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    yield();

    uint32_t t_setup_end = millis();
    wake_duration = t_setup_end - t_setup_start;
    Serial.printf(
        "\nRan for " ANSI_BOLD "%u" ANSI_RESET " ms, going to sleep now.\n", 
        wake_duration
    );
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
