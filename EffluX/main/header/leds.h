#ifndef LEDS_H
#define LEDS_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_TLC59711.h> 
#include "header/constants.h"

extern Adafruit_TLC59711 tlc;

void set_led(uint16_t r, uint16_t g, uint16_t b, uint8_t wait, uint16_t led_num);
void colorWipe(uint16_t r, uint16_t g, uint16_t b, uint8_t wait);
void led_setup();


#endif