#include "../header/relay.h"
#include "../header/constants.h"

void setup_relay(){
    Serial.println("setup relay");
    pinMode(RLY_BOILER_PWR, OUTPUT);
    pinMode(RLY_BH_PWR, OUTPUT);
    pinMode(RLY_TH_PWR, OUTPUT);
}

void turn_on_boiler(){
    digitalWrite(RLY_BOILER_PWR, HIGH);
}
void turn_off_boiler(){
    digitalWrite(RLY_BOILER_PWR, LOW);
}
void turn_on_top(){
    digitalWrite(RLY_TH_PWR, HIGH);
}
void turn_off_top(){
    digitalWrite(RLY_TH_PWR, LOW);
}
void turn_on_bot(){
    digitalWrite(RLY_BH_PWR, HIGH);
}
void turn_off_bot(){
    digitalWrite(RLY_BH_PWR, LOW);
}