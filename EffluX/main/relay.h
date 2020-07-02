#ifndef RELAY_H
#define RELAY_H

#include <Arduino.h>
#include <Wire.h>
#include "constants.h"
#include "leds.h"

void setup_relay();
void turn_on_boiler();
void turn_off_boiler();
void turn_on_top();
void turn_off_top();
void turn_on_bot();
void turn_off_bot();

#endif