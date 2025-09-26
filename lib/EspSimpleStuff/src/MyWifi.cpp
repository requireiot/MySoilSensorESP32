/**
 * @file        MyWifi.cpp
 * Author		: Bernd Waldmann
 * Created		: 9-Feb-2020
 * Tabsize		: 4
 * 
 * This Revision: $Id: MyWifi.cpp 1850 2025-09-26 16:09:10Z  $
 */

/*
   Copyright (C) 2022,2025 Bernd Waldmann

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
//#include "myauth.h" // defines WIFI_SSID, WIFI_PASSWORD
#include "ansi.h"

//----- types ------------------------------------------------------------------

/// @brief Wifi connection information that can be stored in RTC RAM or FFS
struct WifiState {
   uint32_t crc32;
   uint32_t ip;
   uint32_t gateway;
   uint32_t subnet;
   uint32_t dns;
   uint32_t channel;
   uint8_t bssid[6];

   bool operator == (const WifiState& other) {
      return (ip == other.ip)
         &&  (gateway == other.gateway)
         &&  (subnet == other.subnet)
         &&  (dns == other.dns)
         &&  (channel == other.channel)
         ;
   }

   bool is_valid() {
        return crc32 == calculateCRC32( 
            ((uint8_t *)&(this->ip)), sizeof(*this)-sizeof(crc32)
        );
   }

   void make_valid() {
        crc32 = calculateCRC32( 
            ((uint8_t *)&(this->ip)), sizeof(*this)-sizeof(crc32)
        );
   }
};

//----- constants and preferences ----------------------------------------------

/// allow automatic connection to Wifi AP on power up ?
const bool ALLOW_AUTO_CONNECT_ON_START = false;
/// allow automatic re-connection to Wifi AP using channed and BSSID from NVM?
const bool ALLOW_AUTO_RECONNECT = true;

#define WIFI_TIMEOUT_MS 5000uL

#define IF_NAME ANSI_MAGENTA "WiFi" ANSI_RESET

//----- local variables --------------------------------------------------------

/// singleton WiFi client, accessible from other modules
WiFiClient wifiClient;
/// parameters of last WiFi connection, stored in RTC RAM
RTC_DATA_ATTR WifiState wifiState;
/// statistics about how long it takes to connect etc
static unsigned t_wifi_setup = 0;    // filled by setupWifi()
static uint32_t t_start, t_connected, t_gotIP;

enum connect_t { error=0, autoconnect, reconnect, freshconnect };
static connect_t connectMode = connect_t::error;       // filled by setupWifi()

static const char* _wifi_SSID;
static const char* _wifi_PASS;

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
 * @brief Add Wifi performance parameters to JSON report
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
    doc["dhcp"] = t_gotIP - t_connected;  // time to get IP address [ms]
    doc["mode"] = (int)connectMode;
    doc["RSSI"] = quality;
    doc["ttc"] = t_wifi_setup;            // time to connect to AP [ms]
}

//----------------------------------------------------------------------------

static void onWiFiEvent(WiFiEvent_t event) 
{
    switch(event) {

        case ARDUINO_EVENT_WIFI_READY:
            log_i(IF_NAME " ready");
            break;

        case ARDUINO_EVENT_WIFI_STA_START:
            delay(10);
			log_i(IF_NAME ": STA start, hostname is '" ANSI_BOLD "%s" ANSI_RESET "'",
	            WiFi.getHostname());
			break;

        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            t_connected = millis();
            log_i(IF_NAME " connected after %u ms", t_connected-t_start);
            break;

        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            t_gotIP = millis();
            log_i(IF_NAME ": got IP " ANSI_BOLD "%s" ANSI_RESET " after %u ms", 
            	WiFi.localIP().toString().c_str(),
            	t_gotIP-t_start
            );
            break;

        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
			log_e( IF_NAME " " ANSI_BRIGHT_RED "Disconnected" ANSI_RESET);
            break;

        case ARDUINO_EVENT_WIFI_STA_STOP:
            log_i(IF_NAME ": STA stop");
            break;

        case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE:
            log_i(IF_NAME " authmode change ");
            break;
            
        default:
            log_i(IF_NAME " event %d",(int)event);
            break;
    }    
}

//----------------------------------------------------------------------------

/**
 * @brief Try to re-connect to same WiFi AP as before. This does _not_ check if
 * the `wifiState` struct is valid, that must be done by the caller.
 * 
 * @return  true if connection was successful, else app should try _freshConnect
 */
static bool _reconnectWifi( const char* ssid, const char* pass )
{
    int wfs;

    log_i(IF_NAME " try to re-connect ch=%d, ", wifiState.channel);
    t_start = millis();
    WiFi.config( wifiState.ip, wifiState.gateway, wifiState.subnet, wifiState.dns, wifiState.dns );
    WiFi.begin( ssid, pass, wifiState.channel, wifiState.bssid, true );

    while( (wfs=WiFi.status()) != WL_CONNECTED ) {
    
        if (wfs == WL_CONNECT_FAILED) {
            log_e( ANSI_BRIGHT_RED "Failed to connect" ANSI_RESET);
            return false;
        }

        if ((uint32_t)(millis() - t_start) > WIFI_TIMEOUT_MS) {
            log_e( ANSI_BRIGHT_RED "timeout ERROR  " ANSI_RESET );
            WiFi.disconnect();
            return false;
        }

        delay(50);
    }    
    return true;
}

//----------------------------------------------------------------------------

/**
 * @brief  Try to connect to WiFi AP using SSID and PASSWORD
 * 
 * @return true  if connection was successful
 * @return false  if not
 */
static bool _freshConnectWifi( const char* ssid, const char* pass )
{
    log_i(IF_NAME " fresh connect to '" ANSI_BOLD "%s" ANSI_RESET "'",ssid);
    t_start = millis();
    WiFi.begin( ssid, pass );
    uint8_t wf = WiFi.waitForConnectResult(WIFI_TIMEOUT_MS);
    if (wf==WL_CONNECTED) {
        log_i(IF_NAME " " ANSI_GREEN "connected" ANSI_RESET);
        return true;
    } else {
        log_e( IF_NAME " " ANSI_RED "connect failed, status=%d" ANSI_RESET, (int)wf);
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
int setupWifi( const char* ssid, const char* pass, bool allow_reconnect )
{
	if (ssid==NULL || pass==NULL) {
		log_e("WiFi: must specify SSID and password");
		return connect_t::error;
	}
	_wifi_SSID = ssid;
	_wifi_PASS = pass;

    connectMode = connect_t::error;
    bool isValid = wifiState.is_valid();
    allow_reconnect = isValid && allow_reconnect && ALLOW_AUTO_RECONNECT;

    log_i(IF_NAME ": config is%svalid, try to connect", 
        isValid ? " " : ANSI_RED " not " ANSI_RESET );

    uint32_t t_start = millis();

    WiFi.onEvent(onWiFiEvent);
    WiFi.persistent( ALLOW_AUTO_CONNECT_ON_START );
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(allow_reconnect);

    if (allow_reconnect) {
//----- 1. check if Wifi has already been connected in the background
        if (ALLOW_AUTO_CONNECT_ON_START && (WiFi.waitForConnectResult(300) == WL_CONNECTED)) {
            if (t_connected==0) t_connected = millis();
            log_i("already connected ");
            connectMode = connect_t::autoconnect;
        } else if (isValid) {
//----- 2. if not, then try to re-connect using saved parameters, if available
            connectMode = _reconnectWifi(ssid,pass) ? connect_t::reconnect : connect_t::error;
        }
    }
    if (connectMode==connect_t::error) {
//----- 3. if all else has failed, let's try a fresh connection using SSID and password
        connectMode = _freshConnectWifi(ssid,pass) ? connect_t::freshconnect : connect_t::error;
    }

    uint32_t t_stop = millis();
    t_wifi_setup = t_stop - t_start;

    if (connectMode != connect_t::error) {
        log_i(IF_NAME ": mode %d, took " ANSI_BRIGHT_MAGENTA "%u" ANSI_RESET " ms", 
            (int)connectMode, t_wifi_setup );
    } else {
        log_e(IF_NAME " " ANSI_BRIGHT_RED "is NOT connected." ANSI_RESET );
        return connect_t::error;
    }

    // default hostname is esp32-ABCDEF, where AB CD EF are last 3 bytes of MAC address
    // if you want a different hostname, define MY_HOSTNAME in platformio.ini
#ifdef MY_HOSTNAME
    log_i("hostname='%s'",MY_HOSTNAME);
    WiFi.setHostname(MY_HOSTNAME);
#endif

    _fillConfig();  // remember successful connection parameters
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
        setupWifi( _wifi_SSID, _wifi_PASS );
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
uint32_t calculateCRC32(const uint8_t* data, size_t length) 
{
    uint32_t crc = 0xffffffff;
    while (length--) 
        crc = _crc32( crc, *data++ );
    return crc;
}
