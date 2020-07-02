/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/

#include <Wire.h>
 
#define TCAADDR 0x70
#define RTC 0x68

void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println("\nI2C Scanner");
}
 
void loop() {
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 0; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      //if (address == RTC){
      //  Serial.println("RTC found");
      //}
      //if (address == TCAADDR){
      //  Serial.println("Mux found");
      //}
      //if (address == 0x38){
       // Serial.println("Wattmeter found");
      //}
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
  delay(5000);          
}
