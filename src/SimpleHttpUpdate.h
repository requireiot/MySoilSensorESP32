/**
 * @file        MyUpdate.h
 * Author		: Bernd Waldmann
 * Created		: 14-Sep-2025
 * Tabsize		: 4
 * 
 * This Revision: $Id: SimpleHttpUpdate.h 1846 2025-09-26 10:12:45Z  $
 */

/*
   Copyright (C) 2025 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public 
   License, v. 2.0. If a copy of the MPL was not distributed with this 
   file, You can obtain one at http://mozilla.org/MPL/2.0/

   SPDX-License-Identifier: MPL-2.0
*/

#ifndef __MYUPDATE_H
#define __MYUPDATE_H

void do_httpUpdate( WiFiClient& client, const char* firmware_url, Print& serial );

#endif
