#ifndef LCD_H
#define LCD_H

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h> //the library is in the Firmware/lib folder
#include <RTClib.h>
#include <RTC.h>
#include "constants.h"
#include "buttons.h"

extern char top_line[30];
extern char bottom_line[30];
extern LiquidCrystal_I2C lcd; //if this doens't work replace 0x27 with 0x3F or just use an i2c scanner


void lcd_setup();
void lcd_default();

#endif
