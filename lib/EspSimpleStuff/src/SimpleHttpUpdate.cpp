/**
 * @file        MyUpdate.cpp
 * Author		: Bernd Waldmann
 * Created		: 14-Sep-2025
 * Tabsize		: 4
 * 
 * This Revision: $Id: SimpleHttpUpdate.cpp 1850 2025-09-26 16:09:10Z  $
 */

/*
   Copyright (C) 2025 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public 
   License, v. 2.0. If a copy of the MPL was not distributed with this 
   file, You can obtain one at http://mozilla.org/MPL/2.0/

   SPDX-License-Identifier: MPL-2.0
*/

/**
 * @brief Convenience function to execute update-over-HTTP
 */

#include <WiFi.h>
#include <HTTPUpdate.h>
#include "ansi.h"
#include "SimpleHttpUpdate.h"


#define TAG ANSI_BRIGHT_MAGENTA "HTTP Update" ANSI_RESET

/**
 * @brief Download new firmware from HTTP server at `firmware_url` and install
 * 
 * @param firmware_url 
 */
void do_httpUpdate( WiFiClient& client, const char* firmware_url, Print& aSerial )
{
    static Print& serial = aSerial;

    httpUpdate.onStart([]() {
        serial.printf(TAG " started\n");
    });
	httpUpdate.onEnd([]() {
		serial.println("\n" TAG " end\n");
	});
    httpUpdate.onError([](int err) {
        serial.printf(TAG " fatal error code %d", err);
    });
	httpUpdate.onProgress([](unsigned int progress, unsigned int total) {
		serial.printf("\r" TAG " progress: %u%%", (100 * progress / total));
	});

    log_i("HTTP update from \n  '" ANSI_BLUE "%s" ANSI_RESET "'", firmware_url);
    httpUpdate.rebootOnUpdate(true);
    HTTPUpdateResult ret = httpUpdate.update(client, firmware_url);    
    switch (ret) {
      case HTTP_UPDATE_FAILED: 
        serial.printf(ANSI_BRIGHT_RED "HTTP_UPDATE_FAILED Error (%d): %s" ANSI_RESET "\n", 
            httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str()); 
        break;
      case HTTP_UPDATE_NO_UPDATES: 
        serial.println( TAG " no updates"); 
        break;
      case HTTP_UPDATE_OK: 
        serial.println( TAG " update ok"); 
        break;
    }
}
