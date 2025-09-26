/**
 * @file        SimpleOTA.cpp
 * Project		: Home automation
 * Author		: Bernd Waldmann
 * Created		: 9-Feb-2020
 * Tabsize		: 4
 * 
 * This Revision: $Id: SimpleOTA.cpp 1850 2025-09-26 16:09:10Z  $
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
#include "ansi.h"
#include "SimpleOTA.h"

//----- OTA
const char* OTA_PASSWORD  = "123";  //the WIFI_PASSWORD you will need to enter to upload remotely via the ArduinoIDE
const int   OTA_PORT      = 3232;

#define TAG ANSI_BRIGHT_MAGENTA "OTA Update" ANSI_RESET

void setupOTA( Print& serialdevice )
{
	static Print& serial = serialdevice;

//----- configure Over-The-Air updates
	ArduinoOTA.setPort( OTA_PORT );
	ArduinoOTA.setPassword( (const char *)OTA_PASSWORD );

	ArduinoOTA.onStart([]() {
		serial.println( TAG " start");
	});
	ArduinoOTA.onEnd([]() {
		serial.println("\n" TAG " end");
	});
	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
		serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
	});
	ArduinoOTA.onError([](ota_error_t error) {
		serial.printf( TAG " Error [%u]: ", error);
		if (error == OTA_AUTH_ERROR) {
			serial.println("Auth Failed");
		} else if (error == OTA_BEGIN_ERROR) {
			serial.println("Begin Failed");
		} else if (error == OTA_CONNECT_ERROR) {
			serial.println("Connect Failed");
		} else if (error == OTA_RECEIVE_ERROR) {
			serial.println("Receive Failed");
		} else if (error == OTA_END_ERROR) {
			serial.println("End Failed");
		}
	});
	ArduinoOTA.begin();
	log_i( TAG " ready");
}
