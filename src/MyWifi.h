/**
 * @file        wifi.h
 * Project		: Home automation, MyLaundromatESP
 * Author		: Bernd Waldmann
 * Created		: 9-Feb-2020
 * Tabsize		: 4
 * 
 * This Revision: $Id: MyWifi.h 1616 2024-07-31 09:22:01Z  $
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

/**
 * @brief Wifi connection information that can be stored in RTC RAM or FFS
 * 
 */
struct WifiState {
   uint32_t crc32;
   uint32_t version;
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
         &&  (version == other.version)
         ;
   }

   static uint32_t calculateCRC32( const uint8_t *data, size_t length );

   bool is_valid() {
       return crc32 == calculateCRC32( ((uint8_t *)&(this->version)), sizeof(*this)-sizeof(crc32)) ;
   }

   void make_valid() {
       crc32 = calculateCRC32( ((uint8_t *)&(this->version)), sizeof(*this)-sizeof(crc32)) ;
   }
};


bool setupWifi( bool allow_reconnect=true, void (*cb)(void)=NULL );
bool loopWifi();
void reportWifi( char* buf, size_t buflen );

#endif // _WIFI_H