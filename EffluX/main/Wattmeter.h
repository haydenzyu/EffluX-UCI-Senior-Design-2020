#ifndef Wattmeter
#define Wattmeter

#include "ADE7953_I2C.h"
#include "RTC.h"
#include "RTClib.h"
#include "ESP32_Wifi_MQTT.h"

extern int Current;
extern int Voltage;

extern ADE7953 myADE7953;

void WattmeterTest();

#endif