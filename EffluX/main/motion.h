#ifndef MOTION_H
#define MOTION_H

#include "constants.h"
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h> 
#include <SparkFunSX1509.h>
#include "buttons.h"
#include "relay.h"

extern volatile int Motion;
extern int l_sensor_val;
extern int r_sensor_val;
extern int f_sensor_val;

//void motionLight();

void detect_motion();
#endif