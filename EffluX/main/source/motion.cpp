#include "../inc/constants.h"
#include <Arduino.h>
#include "../inc/motion.h"

volatile int Motion;

void motionLight() {
	//if(!lock){
		//lock=true;
		int l_sensor_val = digitalRead(io.SX1509_L_MOTION);
        int r_sensor_val = digitalRead(io.SX1509_R_MOTION);
        int f_sensor_val = digitalRead(io.SX1509_F_MOTION);
		
        // Serial.println(l_sensor_val);
        // Serial.println(f_sensor_val);
        // Serial.println(r_sensor_val);

		if (l_sensor_val == 1 || r_sensor_val == 1 || f_sensor_val == 1){
	  		// digitalWrite(LED1Pin,HIGH);
	  		Motion = 1;
			Serial.println("motion detected");
		}
		else {
	  		// digitalWrite(LED1Pin,LOW);
	  		Motion = 0;
			Serial.println("no motion");
		}
		lock=false;
	//}

}

void detect_motion(){ //detemines whether to leave the device on or off
	int l_sensor_val = digitalRead(io.SX1509_L_MOTION);
	int r_sensor_val = digitalRead(io.SX1509_R_MOTION);
	int f_sensor_val = digitalRead(io.SX1509_F_MOTION);
	if (l_sensor_val == 1 || r_sensor_val == 1 || f_sensor_val == 1){
		// digitalWrite(LED1Pin,HIGH);
		Motion = 1;
		Serial.println("motion detected");
	}
	else {
		// digitalWrite(LED1Pin,LOW);
		Motion = 0;
		Serial.println("no motion");
		int ini_time = millis()/1000;
		int temp_minute;
		int minute_passed;
		while(l_sensor_val != 1 || r_sensor_val != 1 || f_sensor_val != 1){
			l_sensor_val = digitalRead(io.SX1509_L_MOTION);
			r_sensor_val = digitalRead(io.SX1509_R_MOTION);
			f_sensor_val = digitalRead(io.SX1509_F_MOTION);
			temp_minute = millis()/1000;
			minute_passed = temp_minute - ini_minute;
			if(minute_passed > 6000){//after 10mins
				//sleep mode
			}
		}
	}

}


