#ifndef WATTMETER_H
#define WATTMETER_H

#include <ADE7953.h>
#include <Arduino.h>
#include <SPI.h>
#include "macros.h"

extern ADE7953 myADE7953;

struct wattmeter_data{
  long apnoload;
  long activeEnergyA;
  float vRMS;
  float iRMSA;
  float powerFactorA;
  float apparentPowerA;
  float reactivePowerA;
  float activePowerA;
};

void read_wattmeter();

#endif
