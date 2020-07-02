#include "../header/buttons.h"
#include "../header/constants.h"

void gpio_setup(){
    Serial.println("init io");
    if (!io.begin(SX1509_ADDRESS))
    {
        while (1) ; // If we fail to communicate, loop forever.
    }
    Serial.println("io init success");
    io.pinMode(SX1509_B0, INPUT);
    io.pinMode(SX1509_B1, INPUT);
    io.pinMode(SX1509_B2, INPUT);
    io.pinMode(SX1509_B3, INPUT);
    io.pinMode(SX1509_B4, INPUT);
    io.pinMode(SX1509_B5, INPUT);
    io.pinMode(SX1509_B6, INPUT);
    io.pinMode(SX1509_Select, INPUT);
}

bool button_0(){//up button
    if(io.digitalRead(SX1509_B0)== HIGH){
        Serial.println("b0 button is being pushed");
        delay(100);
        return true;
    }
    else{
        return false;
    }
}

bool button_1(){//down button
    if(io.digitalRead(SX1509_B1)== HIGH){
        Serial.println("B1 button is being pushed");
        delay(100);
        return true;
    }
    else{
        return false;
    }
}

bool button_2(){//confirm button
    if(io.digitalRead(SX1509_B2)== HIGH){
        Serial.println("B2 button is being pushed");
        delay(100);
        return true;
    }
    else{
        return false;
    }
}

bool button_3(){//menu button
    if(io.digitalRead(SX1509_B3)== HIGH){
        Serial.println("B3 button is being pushed");
        set_led(65535, 0, 0, 1000, 3);
        delay(100);
        return true;
    }
    else{
        return false;
    }
}

bool button_4(){//bottom plate on/off button
    if(io.digitalRead(SX1509_B4)== HIGH){
        Serial.println("B4 button is being pushed");
        delay(100);
        return true;
    }
    else{
        return false;
    }
}

bool button_5(){//top plate on/off button
    if(io.digitalRead(SX1509_B5)== HIGH){
        Serial.println("B5 button is being pushed");
        delay(100);
        return true;
    }
    else{
        return false;
    }
}

bool button_6(){//boiler on/off button
    if(io.digitalRead(SX1509_B6)== HIGH){
        Serial.println("B6 button is being pushed");
        return true;
    }
    else{
        return false;
    }
}