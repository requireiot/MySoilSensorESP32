/**
 * @file        wifi.h
 * Project		: Home automation, MySoilSensorESP32
 * Author		: Bernd Waldmann
 * Created		: 9-Feb-2020
 * Tabsize		: 4
 * 
 * This Revision: $Id: MyWifi.h 1850 2025-09-26 16:09:10Z  $
 */

/*
   Copyright (C) 2020,2021 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0. 
   If a copy of the MPL was not distributed with this file, You can obtain one at http://mozilla.org/MPL/2.0/

   SPDX-License-Identifier: MPL-2.0
*/

#ifndef _WIFI_H
#define _WIFI_H

extern WiFiClient wifiClient;

int setupWifi( const char* ssid, const char* pass, bool allow_reconnect=true );
bool loopWifi();
void reportWifi( JsonDocument& doc );

uint32_t calculateCRC32(const uint8_t* data, size_t length);

#endif // _WIFI_H