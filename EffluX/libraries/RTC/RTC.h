// // ---- RTC ---- //
#pragma once 

#include <Wire.h>
#include "RTClib.h"
// #include <Wire.h>


extern RTC_DS3231 RTC;

extern uint8_t currentHour;
extern uint8_t currentMinute;
extern uint8_t currentSecond;




void printDate();

void printTime();

void getCurrentTime();
void getDateTimeNow(char* outStr);
void RTCSetup();
