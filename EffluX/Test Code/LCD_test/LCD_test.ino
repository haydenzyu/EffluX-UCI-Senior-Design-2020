#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h> 
#include <SparkFunSX1509.h>

#define LCD_ADDR 0x27
#define I2C_SDA 21
#define I2C_SCL 22

#define SX1509_B6 0
#define SX1509_B5 1
#define SX1509_B4 2
#define SX1509_B3 3
#define SX1509_Wattmeter_IRQ 7
#define SX1509_F_MOTION 8
#define SX1509_L_MOTION 9
#define SX1509_B2 10
#define SX1509_Select 11
#define SX1509_B1 12
#define SX1509_B0 13
#define SX1509_R_MOTION 15

const byte SX1509_ADDRESS = 0x3E;  // SX1509 I2C address
SX1509 io;//can use pins 0:15
LiquidCrystal_I2C lcd(0x27,16,2);
uint8_t cursor = 0;

void LCD_refresh(){
  lcd.init();
  lcd.print("Ready For");
  lcd.setCursor(0,1);
  lcd.print("Dispensing");
}

void setup(){
    Wire.begin(I2C_SDA, I2C_SCL);
    Serial.println("init io");
    Serial.begin(115200);
    lcd.init();
    lcd.backlight();
    if (!io.begin(SX1509_ADDRESS))
    {
        while (1) ; // If we fail to communicate, loop forever.
    }
    io.pinMode(SX1509_B0, INPUT);
    io.pinMode(SX1509_B1, INPUT);
    io.pinMode(SX1509_B2, INPUT);
    io.pinMode(SX1509_B3, INPUT);
    io.pinMode(SX1509_B4, INPUT);
    io.pinMode(SX1509_B5, INPUT);
    io.pinMode(SX1509_B6, INPUT);
    io.pinMode(SX1509_Select, INPUT);
    // Print 'Hello World!' on the first line of the LCD:
    lcd.setCursor(0, 0); // Set the cursor on the first column and first row.
    lcd.print("Set Boiler Temp"); // Print the string "Hello World!"
    lcd.setCursor(0, 1); //Set the cursor on the first column and the second row (counting starts at 0!).
    lcd.print("Set Top Temp");
    lcd.setCursor(0, cursor);
    Serial.println("done setup");
}

void loop(){

  lcd.blink();
  delay(500);
  if(io.digitalRead(SX1509_B0)== HIGH){
    Serial.println("b0 button is being pushed");
    cursor=(cursor==0)?1:0;
    lcd.setCursor(0, cursor);
    Serial.println(cursor);
  }
  if(io.digitalRead(SX1509_B1)== HIGH){
    Serial.println("b1 button is being pushed");
    cursor=(cursor==1)?0:1;
    lcd.setCursor(0, cursor);
    Serial.println(cursor);
  }
  
//   lcd.noBlink();
//   delay(500);
}