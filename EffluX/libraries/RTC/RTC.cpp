#include "RTC.h"

#include <Wire.h>
#include "RTClib.h"
#include <stdio.h>

#include "esp_system.h"
#include <time.h>
#include <Arduino.h>

RTC_DS3231 RTC;
uint8_t currentHour;
uint8_t currentMinute;
uint8_t currentSecond;


void printDate() {
	DateTime now = RTC.now(); 
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.println();
    delay(1000);	// Should remove but remember to set delays when called in loops elsewhere
}

void printTime() {
	DateTime now = RTC.now(); 
	Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println(); 
    delay(1000);
}

void getCurrentTime(){
    DateTime now = RTC.now();
    currentHour = now.hour();
    currentMinute = now.minute();
    currentSecond = now.second();

}

void getDateTimeNow(char* outStr){ //returns a char[] with date and time
    DateTime now = RTC.now();
    char str[50];
    sprintf(str, "%d/%d/%d %d:%d:%d ", now.month(), now.day(), now.year(), now.hour(), now.minute(), now.second() );
    for(int i=0; i<50; ++i){
        outStr[i] = str[i];
    }
}


void RTCSetup() {
  // I2C Setup must be called for RTC to work
  RTC.begin();
  if (RTC.lostPower()) {
      Serial.println("RTC is NOT running!");
      // following line sets the RTC to the date & time this sketch was compiled
      RTC.adjust(DateTime(__DATE__, __TIME__));
    }
    Serial.println("Enabled RTC");
    delay(5000);
}