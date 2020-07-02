//Team Efflux (Sienna Ballot, James Faria, Hayden Yu)
//CalPlug Coffee Buddy

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TLC59711.h"
#include "setup.h"
#include "tasks.h"
#include "constants.h"
#include "relay.h"
#include "lcd.h"
#include "leds.h"

//Wattmeter calibration variable define
float calib = -7.665;

void setup() {
  Serial.begin(115200);
  Serial.println("setting up");
  Wire.begin(I2C_SDA, I2C_SCL);
  SPI.begin();
  I2Cscanner(); //check if I2C devices are connected
  system_setup();	// Calls setup for every component
  Serial.println("Setup Done");

  //add tasks here
  xTaskCreate(&control_task, "control_task", 1000, NULL, 1, NULL);
  xTaskCreate(&sensor_task, "senor_task", 1000, NULL, 1, NULL);
  xTaskCreate(&lcd_task, "lcd_task", 1000, NULL, 2, NULL);
  xTaskCreate(&watt_MQTT_task, "watt_MQTT_task", 1000, NULL, 2, NULL);
}
  

void loop() {
  
}
