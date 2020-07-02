// #include <Wire.h>
// #include <Adafruit_MLX90614.h>
// #include "TCA9548A.h"
// #include "SparkFun_VEML6030_Ambient_Light_Sensor.h"
// #include <SparkFun_VL6180X.h>
#include "../header/carafe.h"

mlx = Adafruit_MLX90614();
SparkFun_Ambient_Light light(AL_ADDR);
VL6180x sensor(VL6180X_ADDRESS);
float gain = .125;
int time1 = 100;
long luxVal = 0; 

void top_carafe_setup(){
    I2CMux.begin(Wire);
    //IR thermometer setup
    Serial.println("top MLX90614 setup"); 
    I2CMux.openChannel(TCA_TOP_MLX); //TCA pin 4 and 5
    mlx.begin();

    //Proximity sensor setup
    Serial.println("top VL6180x setup"); 
    I2CMux.openChannel(TCA_TOP_VL); //top carafe sensor TCA pin 6 and 7
    sensor.getIdentification(&identification); // Retrieve manufacture info from device memory
    printIdentification(&identification); // Helper function to print all the Module information
    if(sensor.VL6180xInit() != 0){
        Serial.println("FAILED TO INITALIZE"); //Initialize device and check for errors
    }
    sensor.VL6180xDefautSettings(); //Load default settings to get started.

    //Ambient light sensor setup
    Serial.println("top VEML6030 setup"); 
    I2CMux.openChannel(TCA_TOP_NOA); //TCA pin 8 and 9
    if(light.begin())
        Serial.println("Ready to sense some light!"); 
    else
        Serial.println("Could not communicate with the sensor!");
    light.setGain(gain);
    light.setIntegTime(time1);
    Serial.println("Reading settings..."); 
    Serial.print("Gain: ");
    float gainVal = light.readGain();
    Serial.print(gainVal, 3); 
    Serial.print(" Integration Time: ");
    int timeVal = light.readIntegTime();
    Serial.println(timeVal);

    I2CMux.closeAll(); 

}

void bot_carafe_setup(){
    I2CMux.begin(Wire);
    //IR thermometer setup
    Serial.println("top MLX90614 setup"); 
    I2CMux.openChannel(TCA_BOT_MLX); //TCA pin 10 and 11
    mlx.begin();

    //Proximity sensor setup
    Serial.println("top VL6180x setup"); 
    I2CMux.openChannel(TCA_BOT_VL); //top carafe sensor TCA pin 13 and 14
    sensor.getIdentification(&identification); // Retrieve manufacture info from device memory
    printIdentification(&identification); // Helper function to print all the Module information
    if(sensor.VL6180xInit() != 0){
        Serial.println("FAILED TO INITALIZE"); //Initialize device and check for errors
    }
    sensor.VL6180xDefautSettings(); //Load default settings to get started.

    //Ambient light sensor setup
    Serial.println("top VEML6030 setup"); 
    I2CMux.openChannel(TCA_BOT_NOA); //TCA pin 15 and 16
    if(light.begin())
        Serial.println("Ready to sense some light!"); 
    else
        Serial.println("Could not communicate with the sensor!");
    light.setGain(gain);
    light.setIntegTime(time1);
    Serial.println("Reading settings..."); 
    Serial.print("Gain: ");
    float gainVal = light.readGain();
    Serial.print(gainVal, 3); 
    Serial.print(" Integration Time: ");
    int timeVal = light.readIntegTime();
    Serial.println(timeVal);

    I2CMux.closeAll(); 
}

bool sense_top_light(){
    I2CMux.openChannel(TCA_TOP_NOA);
    luxVal = light.readLight();
    Serial.print("Top Ambient Light Reading: ");
    Serial.print(luxVal);
    Serial.println(" Lux");
    //lux value from 0 to 255
    if(luxVal < 50){
        Serial.println("top dark");
        return false;
    }
    else   
        return true;
    delay(1000);
    I2CMux.closeChannel(TCA_TOP_NOA);
}

double sense_top_temp(){
    I2CMux.openChannel(TCA_TOP_MLX);
    Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempF()); 
    Serial.print("*F\tObject = "); Serial.print(mlx.readObjectTempF()); Serial.println("*F");
    Serial.println();
    return(mlx.readObjectTempF);
    I2CMux.closeChannel(TCA_TOP_MLX);
    delay(500);

}

bool sense_bot_light(){
    I2CMux.openChannel(TCA_BOT_NOA);
    luxVal = light.readLight();
    Serial.print("Top Ambient Light Reading: ");
    Serial.print(luxVal);
    Serial.println(" Lux");
    //lux value from 0 to 255
    if(luxVal < 50){
        Serial.println("bot dark");
        return false;
    }
    else   
        return true;
    delay(1000);
    I2CMux.closeChannel(TCA_BOT_NOA);
}

double sense_bot_temp(){
    I2CMux.openChannel(TCA_BOT_MLX);
    Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempF()); 
    Serial.print("*F\tObject = "); Serial.print(mlx.readObjectTempF()); Serial.println("*F");
    Serial.println();
    return(mlx.readObjectTempF);
    I2CMux.closeChannel(TCA_BOT_MLX);
    delay(500);
}