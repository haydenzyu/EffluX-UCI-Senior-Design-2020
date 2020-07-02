/*****************************************************/
/*Paolo Caraos, Andy Begey, Brandon Metcalf          */
/*****************************************************/
#ifndef WATERCOOLER_H
#define WATERCOOLER_H

#include <Adafruit_MAX31855.h>
#include <Adafruit_PWMServoDriver.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFi.h>
#include <LiquidCrystal_I2C.h>
#include <PubSubClient.h>
#include <SparkFunSX1509.h>
#include <SPI.h>
#include <Esp32WifiManager.h>
#include <Wire.h>
#include <math.h>
#include <Button.h>
#include <ADE7953ESP32.h>
#include <EEPROM.h>
#include <EMem.h>

typedef enum status{
	SLEEP = 0,
	IDLE,
	ACTIVE,
}status_t;

#define sampleLoop 10
#define TURN_INC 5
/****************************SETUP FOR SX1509 GPIO EXTENDER*****************************/
//#define CLOUD_MQTT
//#define DEBUG
//#define DEBUG_WIFI
//#define WIFI_UNIT_TEST
//#define DEBUG_DISPENSE
//#define DEBUG_DISTANCE		
//#define DEBUG_WATTMETER
//#define DEBUG_THERM
//#define DEBUG_FEEDBACK
//#define DEBUG_MIXING
//#define DEBUG_AP
//#define NO_LCD
//#define NO_LED

#define SX1509_RELAY_HEATER 			0
#define SX1509_RELAY_COOLER 			1
#define SX1509_MOTION0 					2
#define SX1509_MOTION1 					3
#define SX1509_MOTION2 					4

#define encoder0PinA  					5
#define encoder0PinB  					6
#define encoder0Select 					7

#define SX1509_MICROWAVE_MOTION_SENSOR  8
/*
The microwave motion sensor sends a digital signal
when motion is detected. While listening for motion
the sensor minimizes power consumption by putting 
itself to sleep. Thus, the output signal will
send out a non-determinate value.

To configure the sensor:

1)The bolt to the left of the LED controls the period 
of sensing. Counter clockwise is from min to max.

2)The bolt to the right of the LED controls the distance
it senses. Counter clockwise is from min to max.

*/

#define SX1509_RELEASE_BUTTON	 		12
#define SX1509_SOLENOID_HOT 	 		10
#define SX1509_SOLENOID_COLD 	 		9

/***************************************************************************************/
#define ESP32_PIR_SENSOR 		 		27 
#define ESP32_RESET_BUTTON 	 		    25
/*******************************LED and their PWM Channels****************************/

#define PCA9865_LED0 			 		0
#define PCA9865_LED1 			 		1
#define PCA9865_LED2 			 		2
#define PCA9865_LED3 			 		3
#define PCA9865_LED4 			 		13

#define PCA9865_RGB_LED_STATUS_R 		5
#define PCA9865_RGB_LED_STATUS_G 		11
#define PCA9865_RGB_LED_STATUS_B 		12

#define PCA9865_RGB_LED_WIFI_R 	 		8
#define PCA9865_RGB_LED_WIFI_G   		9
#define PCA9865_RGB_LED_WIFI_B   		10

//Thermistor calibration constants
#define TC1_gain 						0.988  //Thermocouple correction gain
#define TC1_offset 					   -2.5  //Thermocouple correction offset
#define upper_hysteresis_1 				1.5  //value for overshoot hysteresis (degrees)
#define lower_hysteresis_1 				2  //value for undershoot hysteresis (degrees)
#define min_trans_time 				 	5000
#define HOT 1
#define COLD 0
#define MINHIGHTEMP             		100  //Minimum temp for hot reservior
#define MINLOWTEMP              		70  //Minimum temp for cold reservior

#define local_SPI_freq 					1000000  //Set SPI_Freq at 1MHz (#define, (no = or ;) helps to save memory)
#define local_SS 						5  //Set the SS pin for SPI communication as pin 10  (#define, (no = or ;) helps to save memory)

#define clip(x, min, max)				(max < x)? max: (min > x)? min: x
#define maximum(x, y)				    (x > y)? x: y

/*****************************CONFIGURING WIFI************************************/
/*
1. When the LCD screen reads entering AP mode. Access the network

   Network name: Water Cooler Buddy DeviceConfig
   Password    : watercoolerbuddy

2. After connecting to the network, a UI will appear and help you configure the wifi.
   Enter the new network that water cooler buddy can connect to. The ESP8266 will save
   this information in its EEPROM.
*/
/********************************************************************************/

extern String MAC_ID;
extern String mqtt_topic_w_MAC_ID;

extern unsigned long previousMillis ;

extern const byte SX1509_ADDRESS;

/**********************************CREDENTIALS*******************************************/
extern const char* wifi_ssid ;
extern const char* wifi_pwd;

extern const char* mqtt_server;
extern const char* mqtt_port;
extern const char* mqtt_user;
extern const char* mqtt_pwd;
extern const char* AES_key ;
extern const char* AES_IV;
extern const char* wifiMode;
extern char configured[];

extern EMem emem;
extern LiquidCrystal_I2C lcd;
extern SX1509 io;
extern WifiManager wifiManager;
extern WiFiClient espClient;
extern PubSubClient client;

extern const int interruptPin;

extern double heaterTemp, coolerTemp, mixTemp;
extern float WCB_activePower;

extern Adafruit_PWMServoDriver pwm;

extern Button eepromRstButton;
extern int buttonCount;
extern boolean change;
extern int oldButtonCount;
extern int dispenseCountInt;
extern int rst;

/*Schedule variables*/
extern unsigned long currTime;
extern const unsigned long PERIOD;
extern unsigned long check_wattmeter_time;
extern unsigned long detect_time;
extern unsigned long check_feedback_time;
extern unsigned long check_mqtt_time;
extern unsigned long check_wifi_lcd_time;
extern unsigned long reconnectTime;
extern unsigned long check_adjustTempTime;
extern unsigned long check_AllowDispense;
extern unsigned long check_tempReg;
extern unsigned long check_LCD_refresh;

/*Solenoid control variables*/
extern const int DEBOUNCE_DELAY;
extern const byte DEBOUNCE_TIME;
extern volatile bool solenoid_hot_status;
extern volatile bool solenoid_cold_status;

/*Encoder variables*/
extern volatile int rot_encoder_pos;
extern volatile int target_temp;
extern volatile int dispense_temp;
extern volatile int adjustingTemp;
extern byte allowDispense;

//Steinheart Equation variables derived from temp vs voltage values
const double a0 = -0.0007475927522;
const double a1 = 0.0005618802728;
const double a2 = -0.000001333273773;

const double b0 = 0.005592306897;
const double b1 = -0.0005063608940;
const double b2 = 0.000002360204519;
/*Heater and Cooler relay variables

NOTE: Heater relays must not stay on for more than 5 seconds with
an empty tank. The heating system will be damage without water to 
absorb the heat.

The cooler must not be flicked on and off too frequently. After 
turning off it must stay off for at least a minute.
*/
extern volatile byte heater_status;
extern volatile byte cooler_status;

extern volatile double sensor_Distance;

/* Status variables*/
extern volatile int dispensing;
extern int dispenseTemps[2*sampleLoop];
extern int WCB_Status; /*water cooler buddy operational status 0=sleeping, 1=active */
extern int detect;
extern int Previous_WCB_Status;
extern int check_status_time;

extern ADE7953 myADE7953;

extern unsigned int Therm_0;
extern unsigned int Therm_1;
extern unsigned int Therm_2;
extern unsigned int Therm_3;

//Feedback control variables
extern volatile double hotTempSet;  // *F  - Control setpoint value (in degrees F)  (degrees F is used because of marginally higher resolution per degree)
extern volatile double coldTempSet;
extern double newHotTempSET;  // *F  - New Control setpoint value (in degrees F)
extern double newColdTempSET;
extern unsigned long lastswitcheventtime; //time in millis() since last switch event
extern unsigned long runtime; //tracker for runtime
extern unsigned long lastruntime;//used to detect millis() rollover and indicate transitions
extern byte ThermError1; //this is a flag if there is a Thermocouple 
extern byte ThermError2; //this is a flag if there is a Thermocouple error
extern byte overrideindicator; //used to detect millis() rollover
 
void handleInterrupt();

/*Updates dispense flags*/
void dispense_button_detect();

/*Update encoder flags and temperature control*/
void encoder_rotate_detect();

void motion_detect();

void read_wattmeter();

void read_thermistors();

double DAQ_to_Volt(double x);

double voltToDist(double x);

double averageVal(double x);

double read_PIR_sensor();

void read_thermistor_0();

void read_thermistor_1();

void read_thermistor_2();

void read_thermistor_3();

void callback(char* topic, byte* payload, unsigned int length);

unsigned long WCBreconnect();

void configModeCallback (WifiManager *myWiFiManager);

void resetWifi();

void setup_wifi();

void data_setup(char* data);

void system_reset();

double VoltageToResistance100k(double x);

double ResistanceToTemp100k(double x);

double VoltageToResistance10k(double x);

double ResistanceToTemp10k(double x);

double KtoF(double x);

void FeedbackControl();

void feedBack(int RelayCtrl_1_Pin, double temp_read1, byte TCError1, double temp_set1, bool direct);

void power_Down();

void power_Up();

void welcome();

double Mixing(int setTemp);

void Dispensing(double ratio, int dispPeriod);

int AverageDispenseTemp();

void Set_Scheduling();

void wipe_data();

void setMillis(unsigned long ms);

void WCB_mqtt_publish(String topic, String msg);

void toString(char str[], int num);

void toCharArr(char *ch, String str);

void trackEncoderTurn(int temp, int val);

void tempUpdate();

void WCBWriteMQTT();

int stringSplit(char *buffer[], char* src);

int isInteger(String src);

void EEPROMReset();//This will replace system_reset()

void APModeSetup();//AP mode for configuring Wifi
#endif
