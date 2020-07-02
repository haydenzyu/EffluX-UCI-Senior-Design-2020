#include "setup.h"

void system_setup(){
    led_setup();
    delay(1000);
    RTCSetup();
    delay(1000);
    lcd_setup();
    delay(1000);
    gpio_setup();
    delay(1000);
    top_carafe_setup();
    delay(1000);
    bot_carafe_setup();
    delay(1000);
    setup_relay();
    delay(1000);
    //esp_wifi_mqtt_setup(); //can't see ap 
    delay(1000);
}

void I2Cscanner(){
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 0; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      if (address == RTC_ADDR){
        Serial.println("RTC found");
      }
      if (address == TCA_ADDR){
        Serial.println("Mux found");
      }
      if (address == WATT_ADDR){
        Serial.println("Wattmeter found");
      }
      if (address == LCD_ADDR){
        Serial.println("LCD found");
      }
      if (address == SX1509_ADDRESS){
        Serial.println("GPIO found");
      }
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