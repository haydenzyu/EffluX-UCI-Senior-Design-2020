#ifndef MQTT_H
#define MQTT_H

extern String MACID;

void setup_mqtt();

void send_data(String attribute, float value, String unit, char timestamp[]);

#endif