#ifndef MOTION_H
#define MOTION_H

#include <SPI.h>
#include <Wire.h> 
#include <SparkFunSX1509.h>

extern volatile int Motion;
extern SX1509 io;

void motionLight();
#endif