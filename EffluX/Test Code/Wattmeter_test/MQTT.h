#ifndef MQTT_H
#define MQTT_H

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

extern String MACID;

void setup_mqtt();

void send_data(String attribute, float value, String unit, char timestamp[]);

#endif