//James Faria 2020
#include <SparkFunSX1509.h>
#include <Wire.h>
#include <Arduino.h>
#include "Adafruit_TLC59711.h"
#include <SPI.h>

const byte SX1509_ADDRESS = 0x3E; //default address... for jumpers set to 00

SX1509 io; // Create an SX1509 object


// How many boards do you have chained?
#define NUM_TLC59711 1

#define data   16
#define clock  17

Adafruit_TLC59711 tlc = Adafruit_TLC59711(NUM_TLC59711, clock, data);

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

void setup(){
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
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
  //io.pinMode(SX1509_B1, INPUT);
  //io.pinMode(SX1509_B2, INPUT);
  //io.pinMode(SX1509_B3, INPUT);
  //io.pinMode(SX1509_B4, INPUT);
  //io.pinMode(SX1509_B5, INPUT);
  //io.pinMode(SX1509_B6, INPUT);
  //io.pinMode(SX1509_Select, INPUT)
  tlc.begin();
  tlc.write();
}

void loop(){
  if(io.digitalRead(SX1509_B0)== HIGH){
    Serial.println("button pressed");
    colorWipe(65535, 0, 0, 100); // "Red" (depending on your LED wiring)
    delay(2000);
    colorWipe(0, 65535, 0, 100); // "Green" (depending on your LED wiring)
    delay(2000);
    colorWipe(0, 0, 65535, 100); // "Blue" (depending on your LED wiring)
    delay(2000);
  }
}

void colorWipe(uint16_t r, uint16_t g, uint16_t b, uint8_t wait) {
  for(uint16_t i=0; i<8*NUM_TLC59711; i++) {
      tlc.setLED(i, r, g, b);
      tlc.write();
      delay(wait);
  }
}
