/**
 * @file 		  SimpleReports.cpp
 *
 * Author		: Bernd Waldmann
 * Created		: 21-Sep-2025
 * Tabsize		: 4
 * 
 * This Revision: $Id: SimpleReports.cpp 1840 2025-09-24 14:33:02Z  $
 */

/*
   Copyright (C) 2025 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public License, 
   v. 2.0. If a copy of the MPL was not distributed with this file, You can 
   obtain one at http://mozilla.org/MPL/2.0/

   SPDX-License-Identifier: MPL-2.0
*/

/**
 * @brief gather information about CPU, memory, SDK version etc, and either print
 * to serial port or provide as JSON-formatted string, for publishing via MQTT or syslog
 */

#include <Arduino.h>
#include <WiFi.h>           // LGPLv2.1+ license
#include <ArduinoJson.h>    // MIT license, https://github.com/bblanchon/ArduinoJson

#include "ansi.h"
#include "SimpleReports.h"


static char msgbuf[256];  // buffer for JSON string


/**
 * @brief print information about chip, SDK version, etc. to a serial port
 * 
 * @param serial    Serial port to print to
 */
void printEnvironment( Print& serial )
{
    // static information about hardware
    serial.printf( " Chip:" ANSI_BOLD "%s" ANSI_RESET, 
        ESP.getChipModel() );
    serial.printf( " at " ANSI_BOLD "%d" ANSI_RESET "MHz",
        (int)ESP.getCpuFreqMHz());
    serial.printf("  Flash:" ANSI_BOLD "%d" ANSI_RESET "K", 
        (int)(ESP.getFlashChipSize() / 1024));
    serial.println();

    // static information about build environment
    serial.printf(" Core:" ANSI_BOLD "%s" ANSI_RESET, 
        ESP.getSdkVersion() );
    serial.printf("  Arduino " ANSI_BOLD "%d.%d.%d" ANSI_RESET,
            ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH);
    serial.printf("  C++" ANSI_BOLD "%d" ANSI_RESET, (int)(__cplusplus/100)-2000 );
    serial.println();
}


/**
 * @brief print information about memory and stack usage to a serial port
 * 
 * @param serial    Serial port to print to 
 */
void printMemoryInfo( Print& serial )
{
    serial.printf(" Heap:" ANSI_BOLD "%d" ANSI_RESET, 
        ESP.getFreeHeap() );
    serial.printf("  AllocHeap:" ANSI_BOLD "%u" ANSI_RESET, 
        ESP.getMaxAllocHeap() );
    serial.printf("  Stack watermark:" ANSI_BOLD "%u" ANSI_RESET, 
        (unsigned)uxTaskGetStackHighWaterMark(NULL) );
    serial.println();
}


/**
 * @brief add information about chip, memory size, SDK version, etc. to a JSON document.
 * Caller may add additional information to the document and then publish via MQTT
 * 
 * @param doc  The JSON structure to add items to
 */
void reportEnvironmentJSON( JsonDocument& doc )
{
    doc["Chip"] = ESP.getChipModel();
    doc["MHz"] = ESP.getCpuFreqMHz();
    doc["FlashK"] = ESP.getFlashChipSize() / 1024;
    doc["Core"] = ESP.getSdkVersion();
    char buf[10];
    snprintf(buf,sizeof buf,"%d.%d.%d",
        ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH);
    doc["Arduino"] = buf;
    doc["C++"] = __cplusplus;
    doc["Heap"] = ESP.getFreeHeap();
}


/**
 * @brief return a JSON-formatted string with information about chip, memory size, 
 * etc.
 * 
 * @return String 
 */
const char* reportEnvironmentString()
{
    JsonDocument doc;
    reportEnvironmentJSON(doc);
    serializeJson(doc,msgbuf,sizeof(msgbuf));
    return msgbuf;
}


/**
 * @brief print static information about network connection to a serial port
 * 
 * @param serial  the serial port to print to
 */
void printNetworkInfo( Print& serial )
{
    serial.printf(" MAC:" ANSI_BOLD "%s" ANSI_RESET "  ",
        WiFi.macAddress().c_str());
    serial.printf("IP:" ANSI_BOLD "%s" ANSI_RESET "  ",
        WiFi.localIP().toString().c_str());
    serial.printf("Hostname:'" ANSI_BLUE "%s" ANSI_RESET "'",
        WiFi.getHostname());
    serial.println();
}


/**
 * @brief add information about network connection to a JSON document
 * 
 * @param doc  The JSON structure to add items to
 */
void reportNetworkInfoJSON( JsonDocument& doc )
{
    doc["SSID"] = WiFi.SSID();
    doc["BSSID"] = WiFi.BSSIDstr();
    doc["MAC"] = WiFi.macAddress();
    doc["IP"] = WiFi.localIP().toString();
    doc["Hostname"] = WiFi.getHostname();
}


/**
 * @brief return a JSON-formatted string with information about network connection
 * 
 * @return String 
 */
const char* reportNetworkInfoString()
{
    JsonDocument doc;
    reportNetworkInfoJSON(doc);
    serializeJson(doc,msgbuf,sizeof(msgbuf));
    return msgbuf;
}
