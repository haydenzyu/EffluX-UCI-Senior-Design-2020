#include "Wattmeter.h"
#include "MQTT.h"
#include "RTC.h"
#include "RTClib.h"

//Wattmeter calibration variable define
float calib = -7.665;

void setup(){
    Serial.begin(115200);
    setup_mqtt();
    RTCSetup();
}

void loop(){
    WattmeterTest(); //Output reading from Wattmeter. I2C communication.

}