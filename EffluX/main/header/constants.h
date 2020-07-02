#ifndef CONSTANTS_H
#define CONSTANTS_H
/*define constants and pin numbers here*/

//ESP32 IO pins
#define I2C_SDA 21
#define I2C_SCL 22

// How many boards do you have chained?
#define NUM_TLC59711 1 

//for ESP32 LED driver
#define DATA 16 //GPIO 16 , but phyical pin 28
#define CLOCK 17 //GPIO 17 , but physical pin 27

//I2C Device addresses
#define SX1509_ADDRESS 0x3E
#define TCA_ADDR 0x70
#define RTC_ADDR 0x68
#define LCD_ADDR 0x27
#define AL_ADDR 0x10
#define VL6180X_ADDRESS 0x29

//GPIO Expander pins
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

//PCA9685 (led driver) pinout, need to edit for coffeebuddy
#define PCA9685_BARLED_B1 0 //original code had this as 0
#define PCA9685_BARLED_B2 1 //original code had this as 1
#define PCA9685_BARLED_Y 2 //original code had this as 2
#define PCA9685_BARLED_R1 3 //original code had this as 3
#define PCA9685_BARLED_R2 4 //original code had this as 13
#define PCA9685_STATUS_R 8 //original code had this as 5
#define PCA9685_STATUS_B 9 //original code had this as 11
#define PCA9685_STATUS_G 10 //original code had this as 12
#define PCA9685_WIFI_R 11 //original code had this as 8
#define PCA9685_WIFI_B 12 //original code had this as 9
#define PCA9685_WIFI_G 13 //original code had this as 10

//MUX
#define TCA_TOP_MLX 0
#define TCA_TOP_VL 1
#define TCA_TOP_NOA 2
#define TCA_BOT_MLX 3
#define TCA_BOT_TL 4
#define TCA_BOT_NOA 5

//LED NUM
#define LED0 0
#define LED1 1
#define LED2 2
#define LED3 3

//Thermocouple calibration constants
#define TC1_gain 0.988 //Thermocouple correction gain
#define TC1_offset -2.5  //Thermocouple correction offset

//relay pins
#define RLY_TH_PWR 27
#define RLY_BH_PWR 14
#define RLY_BOILER_PWR 12

#endif