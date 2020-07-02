//This is for IR functions define. Include both receive and transmit
#ifndef IRSendReceive
#define IRSendReceive

//IR lib (also need lib from RTOS lib included above)
#include "IRSendReceive.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "driver/periph_ctrl.h"
#include "soc/rmt_reg.h"
#include "Arduino.h"


//Function Declare
void storeCode();

void Send_ON_OFF_Signal();

void test_send();

void storeUserRemote();

void IRsetupRX(gpio_num_t pinNum);

void IRsetupTX(gpio_num_t pinNum);

void printIRValue(size_t rx_size, rmt_item32_t* item);

#endif






