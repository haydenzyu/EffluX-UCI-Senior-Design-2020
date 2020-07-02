#ifndef LCD_H
#define LCD_H

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h> //the library is in the Firmware/lib folder
#include <RTClib.h>
#include <RTC.h>
#include "constants.h"

extern LiquidCrystal_I2C lcd(0x27,16,2); //if this doens't work replace 0x27 with 0x3F or just use an i2c scanner

void LCD_refresh();
void lcd_setup();
void lcd_default();

#endif
