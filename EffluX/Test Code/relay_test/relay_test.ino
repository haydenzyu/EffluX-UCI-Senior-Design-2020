#include <SPI.h>
#include "Adafruit_MAX31855.h"

//thermocouple pins
#define MAXDO   19 //SPI MISO
#define MAXCS2_TOP 25 //SPI
#define MAXCS1_BOILER 33 //SPI
#define MAXCS3_BOT 26//SPI
#define MAXCLK  18 //SPI clock

//relay pins on esp32
#define RLY_TH_PWR 27
#define RLY_BH_PWR 14
#define RLY_BOILER_PWR 12

//Thermocouple declaration
Adafruit_MAX31855 thermoboil(MAXCLK, MAXCS1_BOILER, MAXDO);
Adafruit_MAX31855 thermobot(MAXCLK, MAXCS3_BOT, MAXDO);
Adafruit_MAX31855 thermotop(MAXCLK, MAXCS2_TOP, MAXDO);

void setup(){
    Serial.begin(115200);
    //while (!Serial) delay(1); // wait for Serial on Leonardo/Zero, etc
    Serial.println("relay test");
    pinMode(RLY_BOILER_PWR, OUTPUT);
    pinMode(RLY_BH_PWR, OUTPUT);
    pinMode(RLY_TH_PWR, OUTPUT);
    // wait for MAX chip to stabilize
    delay(500);
}

void loop(){
    //double a = thermotop.readFahrenheit();
    //double b = thermobot.readFahrenheit();
    //double b = thermobot.readCelsius();
    // Serial.println("\nThermotop: ");
    // Serial.print(a);
    // Serial.println("\nThermobot: ");
    // Serial.print(b);
    digitalWrite(RLY_BOILER_PWR, HIGH);
    digitalWrite(MAXCS1_BOILER, HIGH);
    delay(500);
    digitalWrite(MAXCS1_BOILER, LOW);
    delay(500);
    double c = thermoboil.readCelsius();
    if (isnan(c)) {
        Serial.println("Something wrong with thermocouple!");
    } else {
        Serial.print("\nC = ");
        Serial.println(c);
    }
    //digitalWrite(RLY_TH_PWR, HIGH);
    ///digitalWrite(RLY_BH_PWR, HIGH);
    Serial.println("\non");
    delay(5000);
    //digitalWrite(RLY_TH_PWR, LOW);
    //digitalWrite(RLY_BH_PWR, LOW);
    digitalWrite(RLY_BOILER_PWR, LOW);
    //Serial.println("\noff");
    delay(5000);
}
