#include "Adafruit_TLC59711.h"
#include <SPI.h>
#include <Wire.h> 
#include <SparkFunSX1509.h>

const byte SX1509_ADDRESS = 0x3E; //default address... for jumpers set to 00

SX1509 io; // Create an SX1509 object


// How many boards do you have chained?
#define NUM_TLC59711 1

//Adafruit_TLC59711 tlc = Adafruit_TLC59711(NUM_TLC59711, clock, data);

#define RightMotionSensor 15 //GPIO 15 on GPIO Expander
#define LeftMotionSensor 9 //GPIO 9 on GPIO Expander
#define FrontMotionSenosor 8 //GPIO 8 on GPIO Expander

void setup() {
  Serial.begin(112500);
  if (!io.begin(SX1509_ADDRESS))
    {
        while (1) ; // If we fail to communicate, loop forever.
    }
  pinMode(FrontMotionSenosor, INPUT);
}

void loop() {
  Serial.println(io.digitalRead(FrontMotionSenosor));
  // if (digitalRead(MotionSensor) == 1){
  //   colorWipe(65535, 0, 0, 100); // "Red" (depending on your LED wiring)
  //   delay(200);
  // }
  // else{
  //   colorWipe(0, 65535, 0, 100); // "Green" (depending on your LED wiring)
  //   delay(200);
}

// void colorWipe(uint16_t r, uint16_t g, uint16_t b, uint8_t wait) {
//   for(uint16_t i=0; i<8*NUM_TLC59711; i++) {
//       tlc.setLED(i, r, g, b);
//       tlc.write();
//       delay(wait);
//   }
// }
