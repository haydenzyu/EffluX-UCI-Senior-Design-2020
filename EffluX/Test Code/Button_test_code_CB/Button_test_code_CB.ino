//James Faria 2020
#include <SparkFunSX1509.h>
#include <Wire.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_TLC59711.h"

#define I2C_SDA 21
#define I2C_SCL 22

#define SX1509_B6 0
#define SX1509_B5 1
#define SX1509_B4 2
#define SX1509_B3 3
#define SX1509_Wattmeter_IRQ 7
#define SX1509_F_MOTION 8
#define SX1509_L_MOTION 9
#define SX1509_B2 10
#define SX1509_Select 11
#define SX1509_B1 12
#define SX1509_B0 13
#define SX1509_R_MOTION 15

Adafruit_TLC59711 tlc = Adafruit_TLC59711(1, 17, 16);
const byte SX1509_ADDRESS = 0x3E;  // SX1509 I2C address
SX1509 io;//can use pins 0:15

void colorWipe(uint16_t r, uint16_t g, uint16_t b, uint8_t wait) {
  for(uint16_t i=0; i<4; i++) {
      tlc.setLED(i, r, g, b);
      tlc.write();
      delay(wait);
  }
}

void set_led(uint16_t r, uint16_t g, uint16_t b, uint8_t wait, uint16_t led_num) {
  //led numbering: https://learn.adafruit.com/tlc5947-tlc59711-pwm-led-driver-breakout/programming
  tlc.setLED(led_num, r, g, b);
  tlc.write();
  delay(wait);
}

void setup(){
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("led driver setup");
  tlc.begin();
  tlc.write();
  delay(200);
  Serial.println("init io");
  if (!io.begin(SX1509_ADDRESS))
  {
    while (1) ; // If we fail to communicate, loop forever.
  }
  Serial.println("io init success");

  // //SSR

  // io.pinMode(SX1509_B6, INPUT);
  // io.pinMode(SX1509_B5, INPUT);
  // io.pinMode(SX1509_B4, INPUT);
  // io.pinMode(SX1509_B3, INPUT);
  // io.pinMode(SX1509_Wattmeter_IRQ, INPUT);
  // io.pinMode(SX1509_F_MOTION, INPUT);
  // io.pinMode(SX1509_L_MOTION, INPUT);
  // io.pinMode(SX1509_B2, INPUT);
  // io.pinMode(SX1509_Select, INPUT); 
  // io.pinMode(SX1509_B1, INPUT);
  // io.pinMode(SX1509_B0, INPUT);
  // io.pinMode(SX1509_R_MOTION, INPUT);
  //
  // io.pinMode(SX1509_MICRO_SENSE,INPUT);
  // enableInterrupt(SX1509_MICRO_SENSE, FALLING);
  //Buttons

  io.pinMode(SX1509_B0, INPUT);
  io.pinMode(SX1509_B1, INPUT);
  io.pinMode(SX1509_B2, INPUT);
  io.pinMode(SX1509_B3, INPUT);
  io.pinMode(SX1509_B4, INPUT);
  io.pinMode(SX1509_B5, INPUT);
  io.pinMode(SX1509_B6, INPUT);
  io.pinMode(SX1509_Select, INPUT);
}

void loop(){
  if(io.digitalRead(SX1509_B0)== HIGH){
    Serial.println("b0 button is being pushed");
    colorWipe(65535, 0, 0, 1000);
    delay(500);
    colorWipe(0, 0, 0, 0);
  }
  if(io.digitalRead(SX1509_B1)== HIGH){
    Serial.println("B1 button is being pushed");
    set_led(65535, 0, 0, 1000, 1);
    delay(1000);
  }
  if(io.digitalRead(SX1509_B2)== HIGH){
    Serial.println("B2 button is being pushed");
    set_led(65535, 0, 0, 1000, 2);
    delay(1000);
  }
  if(io.digitalRead(SX1509_B3)== HIGH){
    Serial.println("B3 button is being pushed");
    set_led(65535, 0, 0, 1000, 3);
    delay(1000);
  }
  if(io.digitalRead(SX1509_B4)== HIGH){
    Serial.println("B4 button is being pushed");
    set_led(65535, 0, 0, 1000, 0);
    delay(1000);
  }
  if(io.digitalRead(SX1509_B5)== HIGH){
    Serial.println("B5 button is being pushed");
    set_led(65535, 0, 0, 1000, 1);
    delay(1000);
  }
  if(io.digitalRead(SX1509_B6)== HIGH){
    Serial.println("B6 button is being pushed");
    set_led(65535, 0, 0, 1000, 2);
    delay(1000);
  }
  // if(io.digitalRead(SX1509_Select)== HIGH){
  //   Serial.println("Select button is being pushed");
  // }
}
