/**
 * @file 		  SimpleMqttClient.cpp
 *
 * Author		: Bernd Waldmann
 * Created		: 23-Sep-2025
 * Tabsize		: 4
 * 
 * This Revision: $Id: SimpleMQTTClient.cpp 1840 2025-09-24 14:33:02Z  $
 */

/*
   Copyright (C) 2025 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public License, 
   v. 2.0. If a copy of the MPL was not distributed with this file, You can 
   obtain one at http://mozilla.org/MPL/2.0/

   SPDX-License-Identifier: MPL-2.0
*/

/**
 * @brief Simple MQTT client for IoT stuff.
 * Base topic for publishing and base topic for subscribing defined in call to begin(),
 *  with optional ${HOSTNAME} placeholder replaced by WiFi hostname.
 * Behavior when subscribed topic is received can be defined by overriding onSubscribe() method.
 * Publishing to a subtopic of defined base topic is done by calling publish() method.
 * 
 * Usage example:
 * ```
 *  PubSubClient mqttClient(wifiClient);
 *  SimpleMqttClient myMqttClient(mqttClient,"ha-server.local");
 * void setup() {
 *  myMqttClient.begin("home/${HOSTNAME}/set/","home/${HOSTNAME}/get/");
 *  // publish to "home/esp32c3-123456/get/status"
 *  myMqttClient.publish("status","ON");
 * }
 * void loop() {
 *  myMqttClient.loop();
 * }
 * ```
 */


#include <WiFi.h>
#include <PubSubClient.h>       
#include <ArduinoJSON.h>        // MIT license, https://github.com/bblanchon/ArduinoJson

#include "SimpleMqttClient.h"
#include "ansi.h"

#define T_MQTT_RECONNECT    30'000uL   /// try to reconnect to MQTT broker every X s
#define MQTT_WAIT_MS        100uL      /// wait between connect attempts
#define MQTT_RETRIES        3          /// number of connect attempts

SimpleMqttClient* SimpleMqttClient::_instance = nullptr;

/**
 * @brief start with MQTT client, connect to broker and define topics
 * 
 * @param subscribeTopic  topic to subscribe to, or NULL if none. This can be
 *                        a full topic like "home/status", or a base topic ending
 *                        with "/", in which case all subtopics are subscribed to.
 * @param publishTopic    top-level topic to publish to, or NULL if none
 * @return true           if connection succeeded
 * @return false          if connection failed
 */
bool SimpleMqttClient::begin( const char* subscribeTopic, const char* publishTopic )
{
    if (subscribeTopic) {
        _subscribeTopic = subscribeTopic;
        _subscribeTopic.replace("${HOSTNAME}",WiFi.getHostname());
    }
    if (publishTopic) {
        _publishTopic = publishTopic;
        _publishTopic.replace("${HOSTNAME}",WiFi.getHostname());
    }
    _psc->setServer(_broker, 1883);
    _psc->setCallback(subscribeCallback);
    _psc->setKeepAlive(30).setSocketTimeout(4);
    return reconnect();
}


/**
 * @brief internal helper for MQTT subscribe callback, a static method. 
 * Calls onReceive() method of the instance.
 * If a subscribe topic was defined in begin(), it is stripped off the topic 
 * parameter before calling onReceive().
 * 
 * @param topic     full topic of received message, 0-terminated
 * @param payload   message as a byte string, not 0-terminated
 * @param length    number of bytes in payload
 */
void SimpleMqttClient::subscribeCallback( char* topic, byte* payload, unsigned length )
{
    if (_instance) {
        char* subtopic = topic;
        char msg[length+1];
        memcpy(msg,payload,length);
        msg[length] = 0;
        if (_instance->_subscribeTopic.length()>0) {
            // we have a pre-defined subscribe topic, strip it off
            if (!strncmp(
                topic,
                _instance->_subscribeTopic.c_str(),
                _instance->_subscribeTopic.length())
                ) {
                subtopic += _instance->_subscribeTopic.length();
                if (*subtopic=='/') subtopic++;
            } else {
                log_w("Received message for unknown topic '%s'",topic);
                return;
            }
        }
        log_i(
            "MQTT receive\n  topic='" ANSI_GREEN "%s" ANSI_RESET 
            "'\n  payload='" ANSI_BLUE "%s" ANSI_RESET "'",
            topic, 
            msg ? msg : "(null)"
        );
        _instance->onReceive(subtopic,msg);
    }
}


/**
 * @brief periodic processing of MQTT stuff, and try to reconnect if needed. Call this from loop()
 * 
 * @return true    if connected
 * @return false   if not connected
 */
bool SimpleMqttClient::loop() 
{
    if (!_psc->connected()) {
        log_e("Reconnecting to MQTT server, state was %d", _psc->state());
        if (reconnect()) {
            log_i("Reconnected to MQTT server");
            _psc->loop();
        }
        return _psc->connected();
    } else {
        _psc->loop();
        return true;
    }
}


/**
 * @brief Connect or reconnect (if needed) to MQTT broker. 
 * If a subscribe topic was defined in begin(), subscribe to it.
 * 
 * To limit the rate of access to the MQTT broker, while catering for missed 
 * connections, this method will
 * - make up to `MQTT_RETRIES` attempts to connect, with `MQTT_WAIT_MS` ms wait 
 *   between attempts
 * - if still unsuccessful, during the next call to the method, at least 
 *   `T_MQTT_RECONNECT` ms must have passed before anotehr attempt is made
 * 
 * @return true    if connected
 * @return false   if not connected
 */
bool SimpleMqttClient::reconnect()
{
	if (_psc->connected()) return true;
    uint32_t t_begin, t_end;

    unsigned long t_now = millis();
    // if this is the first attempt, or we have waited 30s, then try again
    if (
        _lastReconnectAttempt==0 
        || (unsigned long)(t_now - _lastReconnectAttempt) > T_MQTT_RECONNECT
    ) {
        _lastReconnectAttempt = t_now;
        log_i("Connecting to MQTT broker '" ANSI_BLUE "%s" ANSI_RESET "'"
            " as '" ANSI_BLUE "%s" ANSI_RESET "'",
            _broker, WiFi.getHostname());
        t_begin = millis();
        for (int i=0; i<MQTT_RETRIES; i++) {
            if (_psc->connect(WiFi.getHostname())) {
                // successful connection
                t_end = millis();
                _t_mqtt_connect = t_end - t_begin;                
                log_i( ANSI_GREEN "connected" ANSI_RESET " in %u ms",
                    _t_mqtt_connect );
                onConnect();
                if (_subscribeTopic.length()>0) {
                    String subscribeTopic = _subscribeTopic;
                    if (subscribeTopic.endsWith("/")) subscribeTopic += "#";
                    log_i("Subscribing to '" ANSI_GREEN "%s" ANSI_RESET "'",
                        subscribeTopic.c_str());
                    _psc->subscribe(subscribeTopic.c_str());
                }
                return _psc->connected();            
            } else {
                int st = _psc->state();
                log_e(ANSI_RED "failed, rc=%d" ANSI_RESET, st);
                delay(MQTT_WAIT_MS);
            }
        }
        return false;
    }
	return _psc->connected();
}


/**
 * @brief Publish a message to MQTT broker
 * 
 * @param subtopic    topic or subtopic. If _publishTopic was defined in begin(), it is prepended
 * @param payload     message payload, 0-terminated
 * @param retain      if true, message is retained by broker 
 */
void SimpleMqttClient::publish(const char* subtopic,const char* payload, bool retain)
{
	if (!_psc->connected()) return;
    char topic[256];

    topic[0] = 0;
    if (_publishTopic.length()>0) strncat(topic,_publishTopic.c_str(),sizeof topic-1);
	strncat(topic,subtopic,sizeof topic-1);
    reconnect();
	if (_psc->publish(topic, payload, (boolean)retain)) {
        log_i(
            "MQTT publish\n  topic='" ANSI_GREEN "%s" ANSI_RESET 
            "'\n  payload='" ANSI_BLUE "%s" ANSI_RESET "'", 
            topic, 
            payload ? payload : "(null)"
        );
	} else {
		log_e("MQTT publish " ANSI_RED "fail !" ANSI_RESET);
	}
    _psc->loop();
}


/**
 * @brief add to JSON report with MQTT performance parameters
 * 
 * @param doc  reference to JSON object to add to
 */
void SimpleMqttClient::report( JsonDocument& doc )
{
    doc["mqtt"] = _t_mqtt_connect;   
}
