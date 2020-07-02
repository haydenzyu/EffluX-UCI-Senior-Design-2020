#include <Wire.h>
#include <Arduino.h>

#define TCAADDR 0x70
#define RTC 0x68

void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup(){
  Serial.begin(9600);
  Wire.begin();
  Serial.print("Starting Scan");
}

void loop(){
  for (uint8_t t=0; t<8; t++) {
     tcaselect(t);
     Serial.print("TCA Port #"); Serial.println(t);

     for (uint8_t addr = 0; addr<=127; addr++) {
       if (addr == TCAADDR) continue;
       if (addr == RTC) continue;
       Wire.beginTransmission(addr);
       Wire.write(0b1001);
       uint8_t found_status = Wire.endTransmission();
       if (found_status == 0){
         Serial.print("end transmission status: "); Serial.println(found_status);
         Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
       }
     }
     delay(1000);
   }
   delay(5000);
}
