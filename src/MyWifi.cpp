/**
 * @file        wifi.cpp
 * Author		: Bernd Waldmann
 * Created		: 9-Feb-2020
 * Tabsize		: 4
 * 
 * This Revision: $Id: MyWifi.cpp 1608 2024-06-24 16:58:53Z  $
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
#include "MyWifi.h"
#include "myauth.h" // defines WIFI_SSID, WIFI_PASSWORD
#include "ansi.h"

//#undef Serial
//#define Serial Serial0

extern HardwareSerial DebugSerial;
#undef Serial
#define Serial DebugSerial

#define WIFI_CONFIG_VERSION 100

WiFiClient wifiClient;

// keep in ESP32-C3 "RTC memory"
RTC_DATA_ATTR WifiState wifiState;

//#define USE_BUILTIN_LED

#ifdef USE_BUILTIN_LED
 #define WIFI_LED_ON   digitalWrite(LED_BUILTIN,LOW)
 #define WIFI_LED_OFF  digitalWrite(LED_BUILTIN,HIGH)
#else
 #define WIFI_LED_ON
 #define WIFI_LED_OFF
#endif

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
 * 
 * @param pState  points to WiFi parameters struct to be filled
 */
static void _fillConfig()
{
    wifiState.channel = WiFi.channel();           
    memcpy( wifiState.bssid, WiFi.BSSID(), 6 );
    wifiState.ip = WiFi.localIP();
    wifiState.gateway = WiFi.gatewayIP();
    wifiState.subnet = WiFi.subnetMask();
    wifiState.dns = WiFi.dnsIP();
    wifiState.make_valid();
    yield();
}

//----------------------------------------------------------------------------

static void _printConfig() 
{
    Serial.print( ANSI_WHITE );
    Serial.print("  BSSID   : ");
    for (int i=0; i<6; i++) Serial.printf("%02X ", wifiState.bssid[i]);
    Serial.printf("\tChannel : %d\n", wifiState.channel ); 
    Serial.printf("  IP      : %s", iptoa(wifiState.ip) ); 
    Serial.printf("\tGateway : %s\n", iptoa(wifiState.gateway) ); 
    Serial.printf("  Subnet  : %s", iptoa(wifiState.subnet) ); 
    Serial.printf("\t\tDNS     : %s\n", iptoa(wifiState.dns) ); 
    Serial.print(ANSI_RESET);
}

//----------------------------------------------------------------------------

void onWiFiEvent(WiFiEvent_t event) {
    Serial.printf(" [WiFi:%d] ", event);

    switch(event) {
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            Serial.print("got IP ");
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            Serial.print("lost connection ");
            break;
        case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE:
            Serial.print("authmode change ");
            break;
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            Serial.print("connected ");
            break;
        case ARDUINO_EVENT_WIFI_STA_STOP:
            Serial.print("stop ");
            break;

    }    
}

/**
 * @brief Try to re-connect to same WiFi AP as before.
 * 
 * @param   pState  points to struct with WiFi parameters
 * @return  true if connection was successful, else app should try _freshConnect
 */
static bool _reconnectWifi( void (*cb)(void) )
{
    const uint32_t timeout = 5000;      // 5 seconds
    bool blink = false;
    uint32_t t1, t2;

    Serial.printf("try to re-connect ch=%d ... ", wifiState.channel);
    uint32_t t_start = millis();

    WiFi.onEvent(onWiFiEvent);
    WiFi.config( wifiState.ip, wifiState.gateway, wifiState.subnet, wifiState.dns, wifiState.dns );
    WiFi.begin( WIFI_SSID, WIFI_PASSWORD, wifiState.channel, wifiState.bssid, true );

    delay(200);
    if (cb) cb();

    t1 = millis();
    int wfs;
    
    while( (wfs=WiFi.status()) != WL_CONNECTED ) {
        Serial.print(".");
        if (blink) { WIFI_LED_ON; } else { WIFI_LED_OFF; } blink = !blink;   
    
        if (wfs == WL_CONNECT_FAILED) {
            Serial.println("Failed to connect");
            return false;
        }

        if ((uint32_t)(millis() - t_start) > timeout) {
            WIFI_LED_OFF;
            Serial.print( ANSI_BRIGHT_RED ANSI_BOLD "ERROR  " ANSI_RESET );
            WiFi.disconnect();
            yield();
            return false;
        }

        delay(50);
    }
    
    t2 = millis();
    Serial.printf(" wait:%u ms ", (unsigned)(t2-t1));

    WIFI_LED_OFF;
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
    bool blink = false;
    wl_status_t ws;
    unsigned t;

    Serial.printf("\nfresh connect, SSID='%s' pw='****'\n",WIFI_SSID);
    uint32_t t_start = millis();

//    WiFi.config( IPADDR_NONE, IPADDR_NONE, IPADDR_NONE, IPADDR_NONE );
    WiFi.begin( WIFI_SSID, WIFI_PASSWORD );
    yield();

    while (!WiFi.isConnected()) {
        ws = WiFi.status();
        t = (unsigned)(millis() - t_start);
        Serial.printf(" %3d\r",(int)(ws));
        if (blink) { WIFI_LED_ON; } else { WIFI_LED_OFF; } blink = !blink;
        delay(200);
        if (t > timeout) {
            WIFI_LED_OFF;
            Serial.print("ERROR  ");
            WiFi.disconnect();
            delay( 1 );
            WiFi.mode( WIFI_OFF );
            return false;
        }
    }
    WIFI_LED_OFF;
    yield();
    if (WiFi.isConnected()) {
        Serial.print(" ok ...");
        return true;
    } else {
        Serial.printf("Wifi connect failed, status=%d ... ",WiFi.status());
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
 * @param cb                callback function, called while waiting for WiFi to 
 *                          connect. This function is guaranteed to be called 
 *                          exactly once, independent of the success of the WiFi 
 *                          connection
 * @return true  if connection was successful
 * @return false  if reconnect and fresh connect failed
 */
bool setupWifi( bool allow_reconnect, void (*cb)(void) )
{
    bool ok = false;
    bool isValid = wifiState.is_valid();
    bool reconnect = isValid && allow_reconnect;

    Serial.printf("Wifi: config is %s valid, try to connect ... ", 
        isValid ? "" : 
        ANSI_RED "not" ANSI_RESET 
        );

    uint32_t t_start = millis();

    WiFi.persistent(- false );
    WiFi.mode(WIFI_STA);
    WiFi.setAutoConnect(reconnect);
    WiFi.setAutoReconnect(reconnect);
    yield();

    if (reconnect) {
        if (WiFi.waitForConnectResult(100) == WL_CONNECTED) {
            Serial.print("already connected ... ");
            ok = true;
            if (cb) cb();
        } else if (isValid) {
            ok = _reconnectWifi( cb );
        }
    }
    if (!ok) {
        ok = _freshConnectWifi();
        if (cb) cb();
        if (ok) _fillConfig();
    }
    if (ok)
        Serial.print( ANSI_BRIGHT_GREEN "\nWifi is connected." ANSI_RESET );
    else 
        Serial.print( ANSI_BRIGHT_RED "\nWifi is NOT connected." ANSI_RESET );

    uint32_t t_stop = millis();
    Serial.printf(" total WiFi setup: %u ms\n", t_stop-t_start );

    _printConfig();

    if (!ok) 
        return false;

    Serial.print( ANSI_WHITE );
#ifdef MY_HOSTNAME
    Serial.printf("  hostname='%s'  ",MY_HOSTNAME);
    WiFi.setHostname(MY_HOSTNAME);
#endif

    //----- report MAC address
    char msgbuf[80];
    uint8_t mac[6];
    WiFi.macAddress(mac);
    snprintf(msgbuf,sizeof(msgbuf),"MAC %02X:%02X:%02X:%02X:%02X:%02X  ",
        mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
	Serial.print(msgbuf);

    //----- report IP address
    IPAddress ip;
    ip=WiFi.localIP();
    snprintf(msgbuf,sizeof(msgbuf),"IP %d.%d.%d.%d", ip[0],ip[1],ip[2],ip[3] );
	Serial.println(msgbuf);
    Serial.print( ANSI_RESET );

    _fillConfig();
    return true;
}


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
