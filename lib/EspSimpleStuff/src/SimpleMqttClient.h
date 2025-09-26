/**
 * @file 		  SimpleMqttClient.h
 *
 * Author		: Bernd Waldmann
 * Created		: 23-Sep-2025
 * Tabsize		: 4
 * 
 * This Revision: $Id: SimpleMqttClient.h 1850 2025-09-26 16:09:10Z  $
 */

/*
   Copyright (C) 2025 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public License, 
   v. 2.0. If a copy of the MPL was not distributed with this file, You can 
   obtain one at http://mozilla.org/MPL/2.0/

   SPDX-License-Identifier: MPL-2.0
*/


#include <PubSubClient.h>       // MIT license, https://github.com/knolleary/pubsubclient

#include <functional>
#define SIMPLE_MQTT_CALLBACK std::function<void(char*, char*)>

class SimpleMqttClient {
protected:
    PubSubClient* _psc;
    unsigned long _lastReconnectAttempt = 0;
    const char* _broker;
    String _publishTopic;
    String _subscribeTopic;
    SIMPLE_MQTT_CALLBACK _callback;
    static SimpleMqttClient* _instance;
    static void subscribeCallback( char* topic, byte* payload, unsigned length );
    unsigned _t_mqtt_connect=0;
public:
    SimpleMqttClient( PubSubClient& psc, const char* broker ) 
        : _psc(&psc), _broker(broker), _callback(nullptr) { _instance=this; }

    bool begin( const char* subscribeTopic=nullptr, const char* publishTopic=nullptr );
    bool reconnect();
    bool loop();
    void publish(const char* subtopic,const char* payload, bool retain=false);
    void setCallback( SIMPLE_MQTT_CALLBACK cb ) { _callback = cb; };

    virtual void onConnect() {};
    virtual void onReceive( char* topic, char* payload ) 
        { if (_callback) _callback(topic, payload); };

    void report( JsonDocument& doc );
};