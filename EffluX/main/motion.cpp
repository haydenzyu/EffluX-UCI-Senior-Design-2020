#include "motion.h"

volatile int Motion;
int l_sensor_val;
int r_sensor_val;
int f_sensor_val;

void detect_motion(){ //detemines whether to leave the device on or off
	l_sensor_val = io.digitalRead(SX1509_L_MOTION);
	delay(500);
	r_sensor_val = io.digitalRead(SX1509_R_MOTION);
	delay(500);
	f_sensor_val = io.digitalRead(SX1509_F_MOTION);
	delay(500);
	if (l_sensor_val == 1 || r_sensor_val == 1 || f_sensor_val == 1){
		Motion = 1;
		Serial.println("motion detected");
	}
	else {
		Motion = 0;
		Serial.println("no motion");
		int ini_time = millis()/1000;
		int temp_sec;
		int sec_passed;
		while(l_sensor_val != 1 || r_sensor_val != 1 || f_sensor_val != 1){
			l_sensor_val = io.digitalRead(SX1509_L_MOTION);
			delay(500);
			r_sensor_val = io.digitalRead(SX1509_R_MOTION);
			delay(500);
			f_sensor_val = io.digitalRead(SX1509_F_MOTION);
			delay(500);
			temp_sec = millis()/1000;
			sec_passed = temp_sec - ini_time;
			if(sec_passed > 6000){//after 10mins
				turn_off_boiler();
				turn_off_top();
				turn_off_bot();
			}
		}
	}

}


