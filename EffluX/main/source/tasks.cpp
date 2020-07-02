#include "tasks.h"

SemaphoreHandle_t xCBStatusSemaphore;
int boiler_status = 0;
int top_plate_status = 0;
int bot_plate_status = 0;


//control
void control_task(void *pvParameter){
    Serial.println("starting control task");
    while(1){
        //button function here
        if(digitalRead(SX1509_INT)==LOW){
            if(button_6()){
                if(boiler_status == 0){
                    turn_on_boiler();
                }
                else(boiler_status == 1){
                    turn_off_boiler();
                }
            }
            else if(button_5()){
                if(top_plate_status == 0){
                    turn_on_top();
                    delay(100);

                }
                else if(top_plate_status == 1){
                    turn_off_top();
                }
            }
            else if(button_4()){
                if(bot_plate_status == 0){
                    turn_on_bot();
                }
                else if(bot_plate_status == 1){
                    turn_off_boiler();
                }
            }
        }
    }
    vTaskDelete(NULL);
}

void carafe_task(void *pvParameter){ //carafe sensors
    while(1){
        if(!sense_top_light() || !sense_bot_light()){

            Serial.print("room is dark, slowing turning down temp");
            turn_off_boiler();
			turn_off_top();
			turn_off_bot();
        }
        else continue;
    }
    vTaskDelete(NULL);
}

void lcd_task(void *pvParameter){
    // while(1){
    //     lcd_default();
    //     if(button_3()){
    //         lcd_menu(); //menu options and button controls
    //     }
    //     vTaskDelay(100/portTick_PERIOD_MS);
    // }
    // vTaskDelete(NULL);
}

void watt_MQTT_task(void *pvParameter){
    while(1){
        WattmeterTest();
		vTaskDelay(60000/portTICK_PERIOD_MS);//60sec
    }
    vTaskDelete(NULL);
}