#include <Adafruit_MLX90614.h>
#include <Wire.h>

#define TCAADDR 0x70

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

void tcaselect(uint8_t i) {
    if (i > 7) return;

    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();  
}

void setup() {
    Serial.begin(9600);
    Wire.begin();
    Serial.println("Adafruit MLX90614 test");  
    tcaselect(2); 
    mlx.begin();  
}

void loop() {
    tcaselect(2); 
    Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempC()); 
    Serial.print("*C\tObject = "); Serial.print(mlx.readObjectTempC()); Serial.println("*C");
    Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempF()); 
    Serial.print("*F\tObject = "); Serial.print(mlx.readObjectTempF()); Serial.println("*F");

    Serial.println();
    delay(500);
}
