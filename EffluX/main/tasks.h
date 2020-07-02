#ifndef TASKS_H
#define TASKS_H

#include <Arduino.h>
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
extern int carafe_ini_time;
extern int carafe_temp_time;
extern int carafe_passed_time;
extern bool in_menu;
extern uint8_t lcd_cursor;
extern char status[];

extern SemaphoreHandle_t xCBStatusSemaphore;//created as a binary semaphore in setup_all() in setup.cpp

void display_status();
void control_task(void *pvParameter);
void sensor_task(void *pvParameter);
void lcd_task(void *pvParameter);
void watt_MQTT_task(void *pvParameter);
#endif