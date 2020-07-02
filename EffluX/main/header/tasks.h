#ifndef TASKS_H
#define TASKS_H

#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "constants.h"
#include "Wattmeter.h"
#include "buttons.h"
#include "lcd.h"
#include "leds.h"
#include "motion.h"
#include "RTC.h"
#include "carafe.h"
#include "relay.h"

extern int boiler_status;
extern int top_plate_status;
extern int top_plate_status;
extern SemaphoreHandle_t xCBStatusSemaphore;//created as a binary semaphore in setup_all() in setup.cpp



#pragma endif