#include "relay.h"

void setup_relay(){
    Serial.println("setup relay");
    pinMode(RLY_BOILER_PWR, OUTPUT);
    pinMode(RLY_BH_PWR, OUTPUT);
    pinMode(RLY_TH_PWR, OUTPUT);
}

void turn_on_boiler(){
    digitalWrite(RLY_BOILER_PWR, HIGH);
    set_led(65535,0,0,0);
    delay(500);
}
void turn_off_boiler(){
    digitalWrite(RLY_BOILER_PWR, LOW);
    set_led(0,0,0,0);
    delay(500);
}
void turn_on_top(){
    digitalWrite(RLY_TH_PWR, HIGH);
    set_led(65535,0,0,1);
    delay(500);
}
void turn_off_top(){
    digitalWrite(RLY_TH_PWR, LOW);
    set_led(0,0,0,1);
    delay(500);
}
void turn_on_bot(){
    digitalWrite(RLY_BH_PWR, HIGH);
    set_led(65535,0,0,2);
    delay(500);
}
void turn_off_bot(){
    digitalWrite(RLY_BH_PWR, LOW);
    set_led(0,0,0,2);
    delay(500);
}