#ifndef SETUP_H
#define SETUP_H
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "leds.h"
#include "RTC.h"
#include "buttons.h"
#include "lcd.h"
#include "relay.h"
#include "ESP32_Wifi_MQTT.h" 
#include "carafe.h"

void system_setup();
void I2Cscanner();

#endif