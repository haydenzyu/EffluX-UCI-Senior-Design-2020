#include "tasks.h"

SemaphoreHandle_t xCBStatusSemaphore;
int boiler_status = 0;
int top_plate_status = 0;
int bot_plate_status = 0;
int carafe_ini_time = 0;
int carafe_temp_time = 0;
int carafe_passed_time = 0;
char status[1];
bool in_menu = false;
uint8_t lcd_cursor = 0;

void display_status(){
  strcpy(bottom_line, "bl:");
  sprintf(status, "%d", boiler_status);
  strcat(bottom_line, status);
  strcat(bottom_line, " t:");
  sprintf(status, "%d", top_plate_status);
  strcat(bottom_line, status);
  strcat(bottom_line, " b:");
  sprintf(status, "%d", bot_plate_status);
  strcat(bottom_line, status);
  lcd.setCursor(0,1);
  lcd.print(bottom_line);
  lcd_default();
}

//control
void control_task(void *pvParameter){
    Serial.println("starting control task");
    while(1){
        //button function here
        if(button_6()){
        //Serial.println("boiler status:");
        //Serial.println(boiler_status);
            if(boiler_status == 0){
                turn_on_boiler();
                boiler_status = 1;
            }
            else if(boiler_status == 1){
                turn_off_boiler();
                boiler_status = 0;
            }
        }
        else if(button_5()){
        //Serial.println("top status:");
        //Serial.println(top_plate_status);
            if(top_plate_status == 0){
                turn_on_top();
                top_plate_status = 1;
            }
            else if(top_plate_status == 1){
                turn_off_top();
                top_plate_status = 0;
            }
        }
        else if(button_4()){
        //Serial.println("bot status:");
        //Serial.println(bot_plate_status);
            if(bot_plate_status == 0){
                turn_on_bot();
                bot_plate_status = 1;
            }
            else if(bot_plate_status == 1){
                turn_off_bot();
                bot_plate_status = 0;
            }
        }
    }
    vTaskDelete(NULL);
}

void sensor_task(void *pvParameter){ //carafe sensors
    while(1){
        detect_motion();
        if(!sense_top_light() || !sense_bot_light()){
            carafe_ini_time = millis()/1000;
            while(!sense_top_light() || !sense_bot_light()){
                carafe_temp_time = millis()/1000;
                carafe_passed_time = carafe_temp_time - carafe_ini_time;
                if(carafe_passed_time > 6000){
                    Serial.print("room is dark, slowing turning down temp");
                    turn_off_boiler();
                    delay(100);
                    turn_off_top();
                    delay(100);
                    turn_off_bot();
                    delay(100);
                    boiler_status = 0;
                    top_plate_status = 0;
                    bot_plate_status = 0;
                }
            }
        }
        else {
            continue;
        }
    }
    vTaskDelete(NULL);
}

void lcd_task(void *pvParameter){
    while(1){
        display_status();
        if(button_3()){
            in_menu = true;
            while(in_menu){
                switch(lcd_cursor){
                    case 0: //
                        lcd.setCursor(0, lcd_cursor);
                        lcd.print("Reset WiFi");
                        lcd.setCursor(0, lcd_cursor+1);
                        lcd.print("Exit");
                        if(button_0()){
                            Serial.println("Up");
                            lcd_cursor=0;
                            lcd.setCursor(0, lcd_cursor);
                            Serial.println(lcd_cursor);
                        }
                        if(button_1()){
                            Serial.println("Down");
                            lcd_cursor=1;
                            lcd.setCursor(0, lcd_cursor);
                            Serial.println(lcd_cursor);
                        }
                        if(button_3){//confirm reset wifi
                            //insert reset wifi function
                           in_menu = false; 
                        }
                        break;
                    case 1:
                        lcd.setCursor(0, lcd_cursor-1);
                        lcd.print("Reset WiFi");
                        lcd.setCursor(0, lcd_cursor);
                        lcd.print("Exit");
                        if(button_0()){
                            Serial.println("Up");
                            lcd_cursor=0;
                            lcd.setCursor(0, lcd_cursor);
                            Serial.println(lcd_cursor);
                        }
                        if(button_1()){
                            Serial.println("Down");
                            lcd_cursor=1;
                            lcd.setCursor(0, lcd_cursor);
                            Serial.println(lcd_cursor);
                        }
                        if(button_3){//confirm exit
                            //exit to default
                           in_menu = false; 
                        }
                        break;
                    default:
                        continue;
                }
                lcd.blink();
            }
        }
    }
    vTaskDelete(NULL);
}

void watt_MQTT_task(void *pvParameter){
    while(1){
        WattmeterTest();
		vTaskDelay(60000/portTICK_PERIOD_MS);//60sec
    }
    vTaskDelete(NULL);
}