/**
 * @file        MyOTA.cpp
 * Project		: Home automation, MyLaundromatESP
 * Author		: Bernd Waldmann
 * Created		: 9-Feb-2020
 * Tabsize		: 4
 * 
 * This Revision: $Id: MyOTA.cpp 1827 2025-09-14 20:54:44Z  $
 */

/*
   Copyright (C) 2020,2025 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0. 
   If a copy of the MPL was not distributed with this file, You can obtain one at http://mozilla.org/MPL/2.0/

   SPDX-License-Identifier: MPL-2.0
*/

/**
 * @brief Convenience function to setup up OTA and define callbacks
 */

#include <ArduinoOTA.h>
#include "MyOTA.h"

//----- OTA
const char* OTA_PASSWORD  = "123";  //the WIFI_PASSWORD you will need to enter to upload remotely via the ArduinoIDE
const int   OTA_PORT      = 3232;

extern HardwareSerial DebugSerial;


void setupOTA()
{
//----- configure Over-The-Air updates
	ArduinoOTA.setPort( OTA_PORT );
	ArduinoOTA.setPassword( (const char *)OTA_PASSWORD );
	DebugSerial.println("setting up OTA");
    
	ArduinoOTA.onStart([]() {
		DebugSerial.println("ArduinoOTA start");
	});
	ArduinoOTA.onEnd([]() {
		DebugSerial.println("\nArduinoOTA end");
	});
	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
		DebugSerial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
	});
	ArduinoOTA.onError([](ota_error_t error) {
		DebugSerial.printf("Error[%u]: ", error);
		if (error == OTA_AUTH_ERROR) {
			DebugSerial.println("Auth Failed");
		} else if (error == OTA_BEGIN_ERROR) {
			DebugSerial.println("Begin Failed");
		} else if (error == OTA_CONNECT_ERROR) {
			DebugSerial.println("Connect Failed");
		} else if (error == OTA_RECEIVE_ERROR) {
			DebugSerial.println("Receive Failed");
		} else if (error == OTA_END_ERROR) {
			DebugSerial.println("End Failed");
		}
	});
	ArduinoOTA.begin();
}
