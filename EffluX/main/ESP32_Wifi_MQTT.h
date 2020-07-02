//2019-2020 firmware contributors: Mugared Khalifa, Kejia Qiang
#ifndef ESP32_WIFI_MQTT_H
#define ESP32_WIFI_MQTT_H
#pragma once
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiManager1.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
// #include "lcd.h"

extern String MACID;
extern char mqttServer[25];
extern char mqttPort[10];
extern char mqttUser[25];
extern char mqttPassword[25];
extern long lastMsg;

extern WiFiClient espClient;
extern PubSubClient client;
extern WiFiManager wifiManager;
extern WiFiServer server;

void start_mqtt();
void setup_wifi();
void wifi_Connector();
void reconnect();
void callback(char* topic, byte* message, unsigned int length);
void configModeCallback(WiFiManager *myWiFiManager);
void pub_msg(char * msg);
void addCustomParameters();
void enterWiFiManager();
void connectToWifi();
void setupWifiClient();
//void EEPROMResetButtonPressISR();
//void EEPROMResetButtonAction();
void wifi_Connector(void *pvParameters);
void load_eeprom_data();
void copy_data_to_field(char* this_data, char* field, int address_of_prev_seperator,int end_address);
void eeprom_save_data(String this_data);
void esp_wifi_mqtt_setup();
void eeprom_data_setup(char* data);
void reset_wifi_credentails();
void send_data(String attribute, float value, String unit, char timestamp[]);
#endif
