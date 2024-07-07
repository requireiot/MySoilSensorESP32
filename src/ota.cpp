/**
 * @file        ota.cpp
 * Project		: Home automation, MyLaundromatESP
 * Author		: Bernd Waldmann
 * Created		: 9-Feb-2020
 * Tabsize		: 4
 * 
 * This Revision: $Id: ota.cpp 1595 2024-05-21 21:06:20Z  $
 */

/*
   Copyright (C) 2020,2021 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0. 
   If a copy of the MPL was not distributed with this file, You can obtain one at http://mozilla.org/MPL/2.0/

   SPDX-License-Identifier: MPL-2.0
*/

#include <ArduinoOTA.h>
#include "ota.h"

//----- OTA
const char* OTA_PASSWORD  = "123";  //the WIFI_PASSWORD you will need to enter to upload remotely via the ArduinoIDE
const int   OTA_PORT      = 3232;

extern HardwareSerial DebugSerial;

#undef Serial
#define Serial DebugSerial


void setupOTA()
{
//----- configure Over-The-Air updates
	ArduinoOTA.setPort( OTA_PORT );
	ArduinoOTA.setPassword( (const char *)OTA_PASSWORD );
	Serial.println("setting up OTA");
    
	ArduinoOTA.onStart([]() {
		Serial.println("ArduinoOTA start");
	});
	ArduinoOTA.onEnd([]() {
		Serial.println("\nArduinoOTA end");
	});
	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
		Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
	});
	ArduinoOTA.onError([](ota_error_t error) {
		Serial.printf("Error[%u]: ", error);
		if (error == OTA_AUTH_ERROR) {
			Serial.println("Auth Failed");
		} else if (error == OTA_BEGIN_ERROR) {
			Serial.println("Begin Failed");
		} else if (error == OTA_CONNECT_ERROR) {
			Serial.println("Connect Failed");
		} else if (error == OTA_RECEIVE_ERROR) {
			Serial.println("Receive Failed");
		} else if (error == OTA_END_ERROR) {
			Serial.println("End Failed");
		}
	});
	ArduinoOTA.begin();
}
