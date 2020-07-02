#ifndef CARAFE_H
#define CARAFE_H

#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include "TCA9548A.h"
#include "SparkFun_VEML6030_Ambient_Light_Sensor.h"
#include <SparkFun_VL6180X.h>
#include "constants.h"

extern TCA9548A I2CMux;
extern Adafruit_MLX90614 mlx;
extern VL6180xIdentification identification;


void top_carafe_setup();
void bot_carafe_setup();
bool sense_top_light();
double sense_top_temp();
bool sense_bot_light();
double sense_bot_temp(); 

#endif