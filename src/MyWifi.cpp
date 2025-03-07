/**
 * @file        wifi.cpp
 * Author		: Bernd Waldmann
 * Created		: 9-Feb-2020
 * Tabsize		: 4
 * 
 * This Revision: $Id: MyWifi.cpp 1724 2025-03-05 09:28:44Z  $
 */

/*
   Copyright (C) 2022,2024 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public 
   License, v. 2.0. If a copy of the MPL was not distributed with this 
   file, You can obtain one at http://mozilla.org/MPL/2.0/

   SPDX-License-Identifier: MPL-2.0
*/

/**
 * @brief  Connect or re-connect to Wifi AP, supports fast re-connection
 * using previously obtained parameters (after waking up from deep sleep).
 * Wifi connection parameters can be made persistent via RTC RAM.
 * 
 * Just call setupWifi(), it will handle 3 cases:
 * 1. Wifi was automatically reconnected by Espressif firmware
 * 2. re-connect to same Wifi network as last time, and re-use IP address etc
 *    received from DHCP server last time
 * 3. if (1) or (2) failed, connect to a Wifi network for the first time,
 *    and save connection parameters for next time
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJSON.h>  // MIT license, https://github.com/bblanchon/ArduinoJson
#include <esp_wifi.h>

#include "MyWifi.h"
#include "myauth.h" // defines WIFI_SSID, WIFI_PASSWORD
#include "ansi.h"

//----- constants and preferences

/*
    in my setup (Fritzbox 7390 + Mesh Repeater Fritzbox 7490), re-connecting
    to the same WiFi channel and BSSID as during the previous wake cycle takes 
    much LONGER (>1000ms) that a fresh connection using SSID, username
    and password (400 ms). 
    For some reason, the DNS lookup for the MQTT broker also takes longer 
    (>1000ms vs 20ms), as does connecting to the MQTT broker (>1000ms vs 30ms)
*/
/// allow automatic connection to Wifi AP on power up ?
const bool ALLOW_AUTO_CONNECT_ON_START = false;
/// allow automatic re-connection to Wifi AP using channed and BSSID from NVM?
const bool ALLOW_AUTO_RECONNECT = false;

//----- external references

extern HardwareSerial DebugSerial;
#undef Serial
#define Serial DebugSerial

//----- local variables

WiFiClient wifiClient;

RTC_DATA_ATTR WifiState wifiState;

static unsigned t_wifi_setup = 0;    // filled by setupWifi()

enum connect_t { error=0, autoconnect, reconnect, freshconnect };
static connect_t connectMode = connect_t::error;       // filled by setupWifi()

//----------------------------------------------------------------------------

const char* iptoa( const IPAddress& ip )
{
    static char s[16];
    snprintf( s, sizeof(s), "%hd.%hd.%hd.%hd", ip[0], ip[1], ip[2], ip[3] );
    return s;
}

//----------------------------------------------------------------------------

/**
 * @brief Fill WiFi config struct (for RTC RAM) with currect WiFi parameters.
 */
static void _fillConfig()
{
    wifiState.channel = WiFi.channel();           
    memcpy( wifiState.bssid, WiFi.BSSID(), sizeof(wifiState.bssid) );
    wifiState.ip = WiFi.localIP();
    wifiState.gateway = WiFi.gatewayIP();
    wifiState.subnet = WiFi.subnetMask();
    wifiState.dns = WiFi.dnsIP();
    wifiState.make_valid();
}

//----------------------------------------------------------------------------

/**
 * @brief Print current Wifi connection parameters to debug console
 */
static void _printConfig() 
{
    Serial.print( "\n" ANSI_WHITE );
    Serial.print("  BSSID   : ");
    for (int i=0; i<6; i++) Serial.printf("%02X ", wifiState.bssid[i]);
    Serial.printf("\tChannel : %d\n", wifiState.channel ); 
    Serial.printf("  IP      : %s", iptoa(wifiState.ip) ); 
    Serial.printf("\tGateway : %s\n", iptoa(wifiState.gateway) ); 
    Serial.printf("  Subnet  : %s", iptoa(wifiState.subnet) ); 
    Serial.printf("\t\tDNS     : %s\n", iptoa(wifiState.dns) ); 
    Serial.printf("  Hostname: %s",WiFi.getHostname());
    uint8_t mac[6];
    WiFi.macAddress(mac);
    Serial.printf("  MAC %02X:%02X:%02X:%02X:%02X:%02X\n",
        mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);

    Serial.print(ANSI_RESET);
}

//----------------------------------------------------------------------------

/**
 * @brief Create JSON report with Wifi performance parameters
 * 
 * @param doc  reference to JSON object to add to
 */
void reportWifi( JsonDocument& doc )
{
    int dBm = WiFi.RSSI();        // in dBm
    int quality;
    if (dBm <= -100) {
        quality = 0;
    } else if (dBm >= -50) {
        quality = 100;
    } else {
        quality = 2 * (dBm + 100);    
    }

    doc["ch"] = wifiState.channel;   
    doc["RSSI"] = quality;
    doc["ttc"] = t_wifi_setup;      // time to connect [ms]
    doc["mode"] = (int)connectMode;
}

//----------------------------------------------------------------------------


void onWiFiEvent(WiFiEvent_t event) 
{
    switch(event) {
    	
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            log_i("got IP ");
            break;
            
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            log_i("lost connection ");
            break;
            
        case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE:
            log_i("authmode change ");
            break;
            
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            log_i("connected ");
            break;
            
        case ARDUINO_EVENT_WIFI_STA_STOP:
            log_i("stop ");
            break;
    }    
}

//----------------------------------------------------------------------------

/**
 * @brief Try to re-connect to same WiFi AP as before.
 * 
 * @return  true if connection was successful, else app should try _freshConnect
 */
static bool _reconnectWifi()
{
    const uint32_t timeout = 5000;      // 5 seconds
    uint32_t t1, t2;
    int wfs;

    log_i("try to re-connect ch=%d, ", wifiState.channel);
    uint32_t t_start = millis();

    WiFi.config( wifiState.ip, wifiState.gateway, wifiState.subnet, wifiState.dns, wifiState.dns );
    WiFi.begin( WIFI_SSID, WIFI_PASSWORD, wifiState.channel, wifiState.bssid, true );

    t1 = millis();

    while( (wfs=WiFi.status()) != WL_CONNECTED ) {
    
        if (wfs == WL_CONNECT_FAILED) {
            log_e( ANSI_BRIGHT_RED ANSI_BOLD "Failed to connect" ANSI_RESET);
            return false;
        }

        if ((uint32_t)(millis() - t_start) > timeout) {
            log_e( ANSI_BRIGHT_RED ANSI_BOLD "timeout ERROR  " ANSI_RESET );
            WiFi.disconnect();
            yield();
            return false;
        }

        delay(50);
    }
    
    t2 = millis();
    log_i(" wait: %u ms ", (unsigned)(t2-t1));

    return true;
}

//----------------------------------------------------------------------------

/**
 * @brief  Try to connect to WiFi AP
 * 
 * @return true  if connection was successful
 * @return false  if not
 */
static bool _freshConnectWifi()
{
    const uint32_t timeout = 10000; // 20 seconds
    wl_status_t ws;
    unsigned t;

    log_i("fresh connect, SSID='%s'",WIFI_SSID);
    uint32_t t_start = millis();

    WiFi.begin( WIFI_SSID, WIFI_PASSWORD );
    yield();

    while (!WiFi.isConnected()) {
        ws = WiFi.status();
        t = (unsigned)(millis() - t_start);
        Serial.printf(" %3d\r",(int)(ws));
        if (t > timeout) {
            Serial.print("ERROR  ");
            WiFi.disconnect();
            delay( 1 );
            WiFi.mode( WIFI_OFF );
            return false;
        }
        delay(50);  // was 200
    }
    yield();
    if (WiFi.isConnected()) {
        log_i(" ok ");
        return true;
    } else {
        log_e("Wifi connect failed, status=%d ",WiFi.status());
        return false;
    }
}

//----------------------------------------------------------------------------

/**
 * @brief Connect or reconnect to WiFi. WiFi parameters (channel, bssid etc.)
 * taken from/stored to a struct which can be held in RTC RAM.
 * 
 * @param allow_reconnect   if true, first try to reconnect using info in 
 *                          saved settings
 * @return 1  if automatic reconnection was successful
 * @return 2  if reconnection was successful
 * @return 3  if fresh connection was successful
  * @return 0  if reconnect and fresh connect failed
 */
int setupWifi( bool allow_reconnect )
{
    connectMode = connect_t::error;
    bool isValid = wifiState.is_valid();
    allow_reconnect = isValid && allow_reconnect && ALLOW_AUTO_RECONNECT;

    log_i("Wifi: config is%svalid, try to connect", 
        isValid ? " " : ANSI_RED " not " ANSI_RESET );

    uint32_t t_start = millis();

    WiFi.onEvent(onWiFiEvent);
    WiFi.persistent( ALLOW_AUTO_CONNECT_ON_START );
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(allow_reconnect);
    yield();

    if (allow_reconnect) {
//----- 1. check if Wifi has already been connected in the background
        if (ALLOW_AUTO_CONNECT_ON_START && (WiFi.waitForConnectResult(300) == WL_CONNECTED)) {
            log_i("already connected ");
            connectMode = connect_t::autoconnect;
        } else if (isValid) {
//----- 2. if not, then try to re-connect using saved parameters, if available
            connectMode = _reconnectWifi() ? connect_t::reconnect : connect_t::error;
        }
    }
    if (!connectMode) {
//----- 3. if all else has failed, let's try a fresh connection using SSID and password
        connectMode = _freshConnectWifi() ? connect_t::freshconnect : connect_t::error;
    }

    uint32_t t_stop = millis();
    t_wifi_setup = t_stop - t_start;

    if (connectMode != connect_t::error) {
        log_i("mode %d, took " ANSI_BRIGHT_MAGENTA "%u" ANSI_RESET " ms", 
            (int)connectMode, t_wifi_setup );
    } else {
        log_e( ANSI_BRIGHT_RED "Wifi is NOT connected." ANSI_RESET );
        return connect_t::error;
    }

    // default hostname is esp32-ABCDEF, where AB CD EF are last 3 bytes of MAC address
    // if you want a different hostname, define MY_HOSTNAME in platformio.ini
#ifdef MY_HOSTNAME
    log_i("hostname='%s'",MY_HOSTNAME);
    WiFi.setHostname(MY_HOSTNAME);
#endif

    _fillConfig();  // remember successful connection parameters
    //_printConfig();
    return int(connectMode);
}

//----------------------------------------------------------------------------

/**
 * @brief Refresh WiFi connection if necessary
 * 
 * @return true  had to refresh WiFi connection
 * @return false  WiFi was still connected
 */
bool loopWifi()
{
    if (WiFi.status() != WL_CONNECTED) {
        delay(1);
        setupWifi();
        return true;
    }
    return false;
}

//----------------------------------------------------------------------------

static uint32_t _crc32( uint32_t crc, uint8_t by )
{
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
        bool bit = crc & 0x80000000;
        if (by & i) {
            bit = !bit;
        }
        crc <<= 1;
        if (bit) {
            crc ^= 0x04c11db7;
        }
    }
    return crc;
}


/**
 * @brief Calculate CRC-32 for a byte array in RAM
 * 
 * @param data       byte array
 * @param length     length in bytes
 * @return uint32_t  CRC-32 value
 */
uint32_t WifiState::calculateCRC32(const uint8_t* data, size_t length) 
{
    uint32_t crc = 0xffffffff;
    while (length--) 
        crc = _crc32( crc, *data++ );
    return crc;
}
