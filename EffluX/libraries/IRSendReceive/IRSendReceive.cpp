//This is for IR functions define. Include both receive and transmit

#include "IRSendReceive.h"

//for signal tweaking (may or may not use...)
#define MARK_EXCESS 100
//From thermostat example
#define IRREPEAT 1
#define FREQUENCY 38000  /* Frequency in hz of carrier signal*/
//from NEC IR codes example
#define RMT_CLK_DIV      100    /*!< RMT counter clock divider */
#define RMT_TICK_10_US    (80000000/RMT_CLK_DIV/100000)   /*!< RMT counter value for 10 us.(Source clock is APB clock) */
#define rmt_item32_tIMEOUT_US  9500   /*!< RMT receiver timeout value(us) */

rmt_config_t blasterConfig; //config for transmit
rmt_config_t recieverConfig;//config for recieve
rmt_item32_t* item; //save codes here
size_t rx_size;
RingbufHandle_t rb = NULL;

const int items_size = 34;
rmt_item32_t test_items[items_size];
rmt_item32_t test_item = {579,1,579,0};

void printIRValue(size_t rx_size, rmt_item32_t* item){
  for (int i = 0; i < (rx_size>>2); i++){
    printf("%d:%dus %d:%dus\n", (item+i)->level0, (item+i)->duration0, (item+i)->level1, (item+i)->duration1);
  }
}

//Stores raw codes
void storeCode(){
  //get RMT RX ringbuffer
  rmt_get_ringbuf_handle(recieverConfig.channel, &rb);
  rmt_rx_start(recieverConfig.channel, 1);

  //if there is already an item stored, return the item to free RB storage space
  //if there is no item stored, continue.
  if (item) {
    vRingbufferReturnItem(rb, (void*) item);
  }

  while(rb) {
    rx_size = 0;//rx_size is the number of everything in the items array (i.e. 0, time, 1, time). The the array size of items is rx_size/4.

    //store raw codes into items
    item = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size, 1000);
    Serial.println("Ring Buffer Checked");

    if (item){
      Serial.println("Recieved Code: ");
      for (int i = 0; i < (rx_size>>2); i++){
        printf("%d:%dus %d:%dus\n", (item+i)->level0, (item+i)->duration0, (item+i)->level1, (item+i)->duration1);
      }
      Serial.println("store code success");

      // Tweak marks shorter, and spaces longer to cancel out IR receiver distortion
      // for (int i = 0; i < (rx_size>>2); i++){
      //   (item+i)->duration0 -= MARK_EXCESS;
      //   (item+i)->duration1 += MARK_EXCESS;
      // }
      // Serial.println("Tweaked Code: ");
      // printIRValue(rx_size, item);
      rb = NULL;
    }
    else {
      Serial.println("store code failed");
      break;
    }
  }
}

void Send_ON_OFF_Signal(){
  int items_size = (int)(rx_size>>2);
  rmt_item32_t items[items_size];
  //store item to items array
  for (int i = 0; i < (rx_size>>2); i++){
    items[i].level0 = (item+i)->level0;
    items[i].duration0 = (item+i)->duration0;
    items[i].level1 =  (item+i)->level1;
    items[i].duration1 =  (item+i)->duration1;
  }

  Serial.println("printing items to send: ");
  for (int i = 0; i < items_size; i++){
    printf("%d:%dus %d:%dus\n", items[i].level0, items[i].duration0, items[i].level1, items[i].duration1);
  }
  // for (int i; i<3; i++){
    delay(10);
    rmt_write_items(blasterConfig.channel, items, items_size, IRREPEAT);
    rmt_wait_tx_done(blasterConfig.channel, portMAX_DELAY);
  //   delay(1000);
  // }
  Serial.println("Message Sent");
}

void test_send(){
  for (int i = 0; i < items_size; i++){
    test_items[i].level0 = test_item.level0;
    test_items[i].duration0 = test_item.duration0;
    test_items[i].level1 =  test_item.level1;
    test_items[i].duration1 =  test_item.duration1;
  }
  for (int i = 0; i < items_size; i++){
     printf("%d:%dus %d:%dus\n", test_items[i].level0, test_items[i].duration0, test_items[i].level1, test_items[i].duration1);
  }
  vTaskDelay(2000/portTICK_PERIOD_MS);
  rmt_write_items(blasterConfig.channel, test_items, items_size, IRREPEAT);
  rmt_wait_tx_done(blasterConfig.channel, portMAX_DELAY);
  free(item);
  vTaskDelay(2000/portTICK_PERIOD_MS);
  Serial.println("Message Sent");
}

void storeUserRemote(){ //can call this to flag a store input response
		storeCode();
		//LED1_ON();
		delay(500);
		//LED1_OFF();
		//LED2_ON();
		delay(500);
		//LED2_OFF();
}

void IRsetupRX(gpio_num_t pinNum){
  // set up IR reciever - most of this is from https://github.com/espressif/esp-idf/blob/ad3b820e701c3ef0803b045b5a2c5ef19630fb0b/examples/peripherals/rmt_nec_tx_rx/main/infrared_nec_main.c
  recieverConfig.rmt_mode = RMT_MODE_RX;
  recieverConfig.channel = RMT_CHANNEL_1;
  recieverConfig.clk_div = 80;
  recieverConfig.gpio_num = pinNum;
  recieverConfig.mem_block_num = 1;
  recieverConfig.rx_config.filter_en = true;
  recieverConfig.rx_config.filter_ticks_thresh = 100;
  recieverConfig.rx_config.idle_threshold = 9500;//rmt_item32_tIMEOUT_US / 10 * (RMT_TICK_10_US);
  rmt_config(&recieverConfig);
  rmt_driver_install(recieverConfig.channel, 1000, 0);
}

void IRsetupTX(gpio_num_t pinNum){
    //set up IR transmitter - look at ThermostatControl in the Calplug github
    blasterConfig.rmt_mode = RMT_MODE_TX;
    blasterConfig.channel = RMT_CHANNEL_0;
    blasterConfig.clk_div = 80; /*per Jigar, resultion -> ESP32 clock speed is 80MHz so division of 8 gives us 1MHz resolution or 1us per tick */
    blasterConfig.gpio_num = pinNum;
    blasterConfig.tx_config.loop_en = 0;
    blasterConfig.tx_config.carrier_en = 1;
    blasterConfig.tx_config.idle_output_en = 1;
    blasterConfig.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    blasterConfig.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
    blasterConfig.tx_config.carrier_duty_percent = 50;
    blasterConfig.tx_config.carrier_freq_hz = FREQUENCY;
    rmt_config(&blasterConfig);
    rmt_driver_install(blasterConfig.channel, 0, 0);
}


