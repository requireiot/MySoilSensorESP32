# EspSimpleStuff 
#### a collection of modules to simplify WiFi, MQTT, OTA and firmware updates
----
## SimpleMqttClient

1. create an instance of `PubSubClient` from the [PubSubClient](https://registry.platformio.org/libraries/knolleary/PubSubClient) library
2. create an instance of `SimpleMqttClient` using the constructor `SimpleMqttClient( PubSubClient& psc, const char* broker )`
3. in `setup()`, call `SimpleMqttClient::begin( const char* subscribeTopic, const char* publishTopic )`, 
4. optionally, you can supply a topic to subscribe to, and a base topic for publishing. Each of those may contain the placeholder `${HOSTNAME}`, which will be replaced with the WiFi hostname. If `subscribeTopic` ends with a '/', then the wildcard '#' will be appended.
5. alternatively, create a class derived from `SimpleMqttClient` and override the `on Connect()` method, which will be called when the connection to the MQTT broker has been established. you can then do your own subscribing
6. optionally, call `SimpleMqttClient::setCallback( void (*cb)(const char*, const char* )` to specify a callback function that will be called when a subscribed topic is published.
7. alternatively, create a class derived from `SimpleMqttClient` and override the `onReceive(char* topic, char* payload)` method, which will be called when a message is received. Parameter `topic` is the MQTT topic, without the prefix defined in teh call to `begin()`. Parameter `payload`is the payload converted to a 0-terminated string.
8. optionally, publish a message by calling `SimpleMqttClient::publish(const char* subtopic,const char* payload, bool retain=false)`. If a base topic for publishing has been defined in the call to `begin()`, then that will be prepended to the topic.

### Usage example:

```
wifiClient WiFiClient;
PubSubClient mqttClient(wifiClient);
SimpleMqttClient myMqttClient(mqttClient,"ha-server.local");
void setup() {
  myMqttClient.begin("home/${HOSTNAME}/set/","home/${HOSTNAME}/get/");
  // publish to "home/esp32c3-123456/get/status"
  myMqttClient.publish("status","ON");
 }
 void loop() {
  myMqttClient.loop();
}
```

## SimpleOTA

1. From `setup()`, call `setupOTA(Print& serialdevice)` to initialize the [ArduinoOTA](https://registry.platformio.org/libraries/jandrassy/ArduinoOTA) system, specifying a device where logging output goes 
2. From `loop()`, call `ArduinoOTA.handle()` frequently

## SimpleHttpUpdate

You need to have a HTTP server that can serve firmware update (.bin) files.

When you want to trigger a firmware update, call `do_httpUpdate( WiFiClient& client, const char* firmware_url, Print& serial )`, specifying the WiFi client, the URL of the firmware update file, such as `myserver.local/ota/firmware.bin`, and a device where logging output goes.

## SimpleReports

This enables sharing information about the environment (CPU type, CPU frequency, flash size, Arduino version etc.) and the network (MAC and IP address, hostname), either by printing that information to logging output, or by sending a JSON-formatted string to MQTT or Syslog.

* `printEnvironment( Print& serial )` outputs information about CPU and development environment to logging output device `serial`
* `printMemoryInfo( Print& serial )` outpus information about free heap size and stack size to logging output device `serial`
* `const char* reportEnvironmentString()` returns the same information (environment and memory) as a string taht can be published to MQTT or Syslog
* `reportEnvironmentJSON( JsonDocument& doc )` adds the same information (environment and memory) to an existing JSON document provided by you, so you can add some further custom information
* `printNetworkInfo( Print& serial )` outputs information about the network connection to logging output device `serial`
* `reportNetworkInfoString()` returns the same information as a string taht can be published to MQTT or Syslog
* `reportNetworkInfoJSON( JsonDocument& doc )` adds the same information  to an existing JSON document provided by you, so you can add some further custom information

## MyWiFi

Connect or re-connect to Wifi AP, supports fast re-connection using previously obtained parameters (after waking up from deep sleep). Wifi connection parameters can be made persistent via RTC RAM.

From `setup()` call `setupWifi( const char* ssid, const char* pass, bool allow_reconnect )`, it will handle 3 cases:
1. Wifi was automatically reconnected by Espressif firmware (if enabled in source code)
2. re-connect to same Wifi network as last time, and re-use IP address etc
   received from DHCP server last time (if allowed)
3. if (1) or (2) failed, connect to a Wifi network for the first time,
   and save connection parameters for next time

From `loop()` you can call `bool loopWifi()`, which will check if the WiFi connection is still live, and reconnect if necessary.

