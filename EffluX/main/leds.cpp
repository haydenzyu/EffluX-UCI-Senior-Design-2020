#include "leds.h" 

//led0: rgb, led1: rgb, led2: r, led3: rg
Adafruit_TLC59711 tlc = Adafruit_TLC59711(NUM_TLC59711, CLOCK, DATA);

void set_led(uint16_t r, uint16_t g, uint16_t b, uint16_t led_num) { //set color to corresponding leds
  //led numbering: https://learn.adafruit.com/tlc5947-tlc59711-pwm-led-driver-breakout/programming
  tlc.setLED(led_num, r, g, b);
  tlc.write();
  delay(100);
}

void colorWipe(uint16_t r, uint16_t g, uint16_t b, uint8_t wait) { //turn on all 4 leds
  for(uint16_t i=0; i<4; i++) {
      tlc.setLED(i, r, g, b);
      tlc.write();
      delay(wait);
  }
}

void led_setup() {
  Serial.println("led driver setup");
  tlc.begin();
  tlc.write();
  colorWipe(65535, 0, 0, 100); // turn on all leds "Red"
  delay(1500);
  colorWipe(0, 0, 0, 100);
}
