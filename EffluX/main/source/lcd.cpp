#include "../header/lcd.h"

char top_line[30];
char bottom_line[30];
uint8_t row_num = 0;
uint8_t reset_wifi_row = 0;
uint8_t set_boiler_temp_row = 1;
uint8_t set_top_plate_row = 2;
uint8_t set_bot_plate_row = 3;
uint8_t exit_row = 4;
uint8_t cursor = 0;

void LCD_refresh(){
  lcd.init();
  lcd.print("Ready For");
  lcd.setCursor(0,1);
  lcd.print("Dispensing");
}

void lcd_setup(){
  lcd.init();
  lcd.backlight();
  io.pinMode(SX1509_Select, INPUT);
  delay(1000);
  lcd.setCursor(0, cursor);
  lcd.blink();
  Serial.println("lcd done setup");
}

void lcd_default(){
  getDateTimeNow(top_line);
  lcd.setCursor(0,0);
  lcd.print(top_line);
}

void lcd_menu(){
  switch(cursor){
    case 0: //
      if(button_0()){
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
      break;
    case 1:
      break;
    case 2:
      break;
    case 3:
      break;
    case 4:
      break;
    default:
  }
}