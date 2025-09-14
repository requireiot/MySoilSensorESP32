/**
 * @file        MyUpdate.cpp
 * Author		: Bernd Waldmann
 * Created		: 14-Sep-2025
 * Tabsize		: 4
 * 
 * This Revision: $Id: $
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

#include <HTTPUpdate.h>
#include "ansi.h"
#include "MyUpdate.h"

extern HardwareSerial DebugSerial;


#define HTTP_UPDATE ANSI_BRIGHT_MAGENTA "HTTP Update" ANSI_RESET

/**
 * @brief Download new firmware from HTTP server at `firmware_url` and install
 * 
 * @param firmware_url 
 */
void do_httpUpdate( const char* firmware_url )
{
    WiFiClient client;

    httpUpdate.onStart([]() {
        DebugSerial.printf(HTTP_UPDATE " started\n");
    });
	httpUpdate.onEnd([]() {
		DebugSerial.println("\n" HTTP_UPDATE " end\n");
	});
    httpUpdate.onError([](int err) {
        DebugSerial.printf(HTTP_UPDATE " fatal error code %d", err);
    });
	httpUpdate.onProgress([](unsigned int progress, unsigned int total) {
		DebugSerial.printf("\r" HTTP_UPDATE " progress: %u%%", (100 * progress / total));
	});

    log_i("HTTP update from \n  '" ANSI_BLUE "%s" ANSI_RESET "'", firmware_url);
    HTTPUpdateResult ret = httpUpdate.update(client, firmware_url);    
    switch (ret) {
      case HTTP_UPDATE_FAILED: 
        DebugSerial.printf(ANSI_BRIGHT_RED "HTTP_UPDATE_FAILED Error (%d): %s" ANSI_RESET "\n", 
            httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str()); 
        break;
      case HTTP_UPDATE_NO_UPDATES: 
        DebugSerial.println("HTTP_UPDATE_NO_UPDATES"); 
        break;
      case HTTP_UPDATE_OK: 
        DebugSerial.println("HTTP_UPDATE_OK"); 
        break;
    }
}
