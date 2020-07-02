#include "lcd.h"

char top_line[30];
char bottom_line[30];
uint8_t row_num = 0;
uint8_t reset_wifi_row = 0;
uint8_t set_boiler_temp_row = 1;
uint8_t set_top_plate_row = 2;
uint8_t set_bot_plate_row = 3;
uint8_t exit_row = 4;
uint8_t cursor = 0;

LiquidCrystal_I2C lcd(0x27,16,2);

void lcd_default(){
  getDateTimeNow(top_line);
  lcd.setCursor(0,0);
  lcd.print(top_line);
}

void lcd_setup(){
  lcd.init();
  lcd.backlight();
  //io.pinMode(SX1509_Select, INPUT);
  //delay(1000);
  lcd.setCursor(0, cursor);
  lcd.blink();
  lcd_default();
  Serial.println("lcd done setup");
}
