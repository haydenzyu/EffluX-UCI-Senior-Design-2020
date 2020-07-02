#include "RTC.h"
#include "RTClib.h"

char time_var[30];

void setup(){
  Serial.begin(115200);
  RTCSetup();
}

void loop(){
  // printDate();
  // printTime();
  getDateNow(time_var);
  Serial.println(time_var);
  getTimeNow(time_var);
  Serial.println(time_var);
  delay(1000);
}
