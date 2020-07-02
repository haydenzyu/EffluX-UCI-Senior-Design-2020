#include "WaterCooler.h"
#include "esp_task_wdt.h"

String MAC_ID;
String mqtt_topic_w_MAC_ID = "water_cooler/";
/**********************************CREDENTIALS*********************/
const char* wifi_ssid = "CalPlugIoT";
const char* wifi_pwd = "A8E61C58F8";

const char* mqtt_server = "m10.cloudmqtt.com";
const char* mqtt_port = "17934";
const char* mqtt_user = "dkpljrty";
const char* mqtt_pwd = "ZJDsxMVKRjoR";
const char* AES_key ="2222222222222222";
const char* AES_IV ="1111111111111111";
const char* wifiMode = "tkip";
char configured[] = {'0', 0};

/******************************************************************/

unsigned long previousMillis = 0;

const byte SX1509_ADDRESS = 0x3E;

EMem emem;
LiquidCrystal_I2C lcd(0x3F, 16, 2);

SX1509 io;
WifiManager wifiManager;
WiFiClient espClient;
PubSubClient client(espClient);

const int interruptPin = 12;

double heaterTemp, coolerTemp, mixTemp=0;
float WCB_activePower=0;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int buttonCount = 0;
boolean change = false;
int oldButtonCount = 0;
int dispenseCountInt = 0;
int rst = 0;

Button eepromRstButton = Button(ESP32_RESET_BUTTON,PULLDOWN);//Bring to ground

/*Schedule variables*/
unsigned long currTime;
const unsigned long PERIOD = 10000; /*every 10 seconds*/
unsigned long check_wattmeter_time;
unsigned long detect_time;
unsigned long check_feedback_time;
unsigned long check_mqtt_time;
unsigned long check_wifi_lcd_time;
unsigned long reconnectTime;
unsigned long check_AllowDispense;
unsigned long check_adjustTempTime;
unsigned long check_tempReg;
unsigned long checkTest;
unsigned long check_LCD_refresh;

/*Solenoid control variables*/
const int DEBOUNCE_DELAY = 200; /*100 millisec*/
const byte DEBOUNCE_TIME = 0.5;
volatile bool solenoid_cold_status = LOW;
volatile bool solenoid_hot_status = LOW;

/*Encoder variables*/
volatile int rot_encoder_pos = 75;
int rot_encoder0_pinA_last = LOW;
volatile int target_temp = rot_encoder_pos;
volatile int dispense_temp = target_temp;
volatile int adjustingTemp = 0;
byte allowDispense = 1;

/*Heater/cooler variables*/
volatile byte heater_status = LOW;
volatile byte cooler_status = LOW;

/* Status variables*/
volatile int dispensing = 0;
int dispenseTemps[2*sampleLoop] = {};
int WCB_Status = 1; /*water cooler buddy operational status 0=sleeping, 1=active */
int detect=0;
int Previous_WCB_Status = WCB_Status;
int check_status_time = 0;

volatile double sensor_Distance = 30;

ADE7953 myADE7953(local_SS, local_SPI_freq);

unsigned int Therm_0 = 0;
unsigned int Therm_1 = 0;
unsigned int Therm_2 = 0;
unsigned int Therm_3 = 0;

//Feedback control variables
volatile double hotTempSet = 140.0;  // *F  - Control setpoint value (in degrees F)  (degrees F is used because of marginally higher resolution per degree)
volatile double coldTempSet = 50.0;  // *F
double newHotTempSET = 140.0;  // *F  - New Control setpoint value (in degrees F) 
double newColdTempSET = 50.0;  // *F
unsigned long lastswitcheventtime=0; //time in millis() since last switch event
unsigned long runtime=0; //tracker for runtime
unsigned long lastruntime=0;//used to detect millis() rollover and indicate transitions
byte ThermError1=false; //this is a flag if there is a Thermocouple error
byte ThermError2=false; //this is a flag if there is a Thermocouple error
byte overrideindicator=0; //used to detect millis() rollover

void dispense_button_detect()
{
  delayMicroseconds(DEBOUNCE_DELAY*1000);
  if(io.digitalRead(SX1509_RELEASE_BUTTON)== HIGH)
  {
    #ifdef DEBUG_DISPENSE
      Serial.println("START DISPENSING");
    #endif 
   
    dispensing = 1;
  }
  else if(io.digitalRead(SX1509_RELEASE_BUTTON)== LOW)
  {
    #ifdef DEBUG_DISPENSE
      Serial.println("FINISHED DISPENSING");
    #endif     
    buttonCount++;
    dispensing = 0;
  }
  check_status_time += 3*PERIOD; // reset to check status again after 30 sec
}

void encoder_rotate_detect()
{
   /* For encoder, if pinA and pinB are both high or both low, it is spinning
     forward. If they're different, it's going backward.
     For more information on speeding up this process, see
     [Reference/PortManipulation], specifically the PIND register.

     http://playground.arduino.cc/Main/RotaryEncoders
  */
  //use light to indicate selection start
  int hotNormalized = (((int) heaterTemp)/TURN_INC)*TURN_INC; //normalize temperature to only display and dispense integer multiples of TURN_INC
  int coldNormalized = (((int) coolerTemp)/TURN_INC)*TURN_INC; //normalize temperature to only display and dispense integer multiples of TURN_INC
  
  if (io.digitalRead(encoder0PinA) == io.digitalRead(encoder0PinB)){
    rot_encoder_pos -= TURN_INC /*COUNTER CLOCKWISE*/;
  }
  else{
    rot_encoder_pos += TURN_INC /*CLOCKWISE*/;
  }
  target_temp = rot_encoder_pos;
  
  if (target_temp>hotNormalized){
    target_temp = hotNormalized;
    if((adjustingTemp == 0 || adjustingTemp == 1) && currTime >= check_tempReg)
    {
      trackEncoderTurn(HOT,rot_encoder_pos - hotNormalized);
    }
  }
  if (target_temp<coldNormalized){
    target_temp = coldNormalized;
    if((adjustingTemp == 0 || adjustingTemp == 2) && currTime >= check_tempReg)
    {
      trackEncoderTurn(COLD,coldNormalized - rot_encoder_pos);
    }
  }
  // let lcd screen reflect temp selection from encoder
  if (target_temp < 100){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Dispense Temp:");
    lcd.setCursor(0, 1);
    lcd.print(target_temp);
    lcd.setCursor(3 , 1);
    lcd.print("degrees F");
  }
  else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Dispense Temp:");
    lcd.setCursor(0, 1);
    lcd.print(target_temp);
    lcd.setCursor(4 , 1);
    lcd.print("degrees F");
  }
  // use light display to indicate range of temperature values to user
  if(target_temp >= coldNormalized && target_temp < coldNormalized*1.3){
    pwm.setPWM(0, 0, 4096);
    pwm.setPWM(1, 0, 4096);
    pwm.setPWM(2, 0, 4096);
    pwm.setPWM(3, 0, 4096);
    pwm.setPWM(13, 4096, 0);
  }
  else if(target_temp >= coldNormalized*1.3 && target_temp < coldNormalized*1.5){
    pwm.setPWM(0,0,4096);
    pwm.setPWM(1,0,4096);
    pwm.setPWM(2,0,4096);
    pwm.setPWM(3,4096,0);
    pwm.setPWM(13,0,4096);
  }
  else if(target_temp >= coldNormalized*1.5 && target_temp < hotNormalized*0.7){
    pwm.setPWM(0, 0, 4096);
    pwm.setPWM(1, 0, 4096);
    pwm.setPWM(2, 4096, 0);
    pwm.setPWM(3, 0, 4096);
    pwm.setPWM(13, 0, 4096);
  }  
  else if(target_temp >= hotNormalized*0.7 && target_temp < hotNormalized*0.83){
    pwm.setPWM(0, 0, 4096);
    pwm.setPWM(1, 4096, 0);
    pwm.setPWM(2, 0, 4096);
    pwm.setPWM(3, 0, 4096);
    pwm.setPWM(13, 0, 4096);
  }
  else if(target_temp >= hotNormalized*0.83 && target_temp <= hotNormalized){
    pwm.setPWM(0, 4096, 0);
    pwm.setPWM(1, 0, 4096);
    pwm.setPWM(2, 0, 4096);
    pwm.setPWM(3, 0, 4096);
    pwm.setPWM(13, 0, 4096);
  }
  if(rot_encoder_pos>target_temp){ //if encoder is beyond range, flash lights
    pwm.setPWM(13, 0, 4096);
    pwm.setPWM(3, 0, 4096);
    pwm.setPWM(2, 0, 4096);
    pwm.setPWM(1, 4096, 0);
    pwm.setPWM(0, 4096, 0);
    delayMicroseconds(200*1000);
    pwm.setPWM(1, 0, 4096);
    pwm.setPWM(0, 0, 4096);
    delayMicroseconds(200*1000);
    pwm.setPWM(1, 4096, 0);
    pwm.setPWM(0, 4096, 0);
    delayMicroseconds(200*1000);
    pwm.setPWM(1, 0, 4096);
    pwm.setPWM(0, 0, 4096);
    delayMicroseconds(200*1000);
    pwm.setPWM(0, 4096, 0);
    rot_encoder_pos = target_temp;
  }
  if(rot_encoder_pos<target_temp){ //if encoder is beyond range, flash lights
    pwm.setPWM(1, 0, 4096);
    pwm.setPWM(0, 0, 4096);
    pwm.setPWM(2, 0, 4096);
    pwm.setPWM(13, 4096, 0);
    pwm.setPWM(3, 4096, 0);
    delayMicroseconds(200*1000);
    pwm.setPWM(13, 0, 4096);
    pwm.setPWM(3, 0, 4096);
    delayMicroseconds(200*1000);
    pwm.setPWM(13, 4096, 0);
    pwm.setPWM(3, 4096, 0);
    delayMicroseconds(200*1000);
    pwm.setPWM(13, 0, 4096);
    pwm.setPWM(3, 0, 4096);
    delayMicroseconds(200*1000);
    pwm.setPWM(13, 4096, 0);
    rot_encoder_pos = target_temp;
  }
  target_temp=((target_temp)/TURN_INC)*TURN_INC;
  check_status_time += 3*PERIOD; // reset timer for power down
}

void read_wattmeter()
{
  long apnoload, activeEnergyA;
  float vRMS, iRMSA, powerFactorA, apparentPowerA, reactivePowerA, activePowerA;
  String msg;
  char* msg_publish;
  String topic_string;
  char* topic_publish;

  static long prev_apnoload;
  apnoload = myADE7953.getAPNOLOAD();
  #ifdef DEBUG_WATTMETER
  Serial.print("APNOLOAD (hex): ");
  Serial.println(apnoload, HEX);
  #endif
  
  static float prev_vRMS;
  vRMS = myADE7953.getVrms(); 
  #ifdef DEBUG_WATTMETER 
  Serial.print("Vrms (V): ");
  Serial.println(vRMS);
  #endif

  static float prev_iRMSA;
  iRMSA = myADE7953.getIrmsA();
  #ifdef DEBUG_WATTMETER  
  Serial.print("IrmsA (mA): ");
  Serial.println(iRMSA);
  #endif

  static float prev_apparentPowerA;
  apparentPowerA = myADE7953.getInstApparentPowerA();
  #ifdef DEBUG_WATTMETER  
  Serial.print("Apparent Power A (mW): ");
  Serial.println(apparentPowerA);
  #endif

  static float prev_activePowerA;
  activePowerA = myADE7953.getInstActivePowerA();
  WCB_activePower = myADE7953.getInstActivePowerA();
  #ifdef DEBUG_WATTMETER  
  Serial.print("Active Power A (mW): ");
  Serial.println(activePowerA);
  #endif

  static float prev_reactivePowerA;
  reactivePowerA = myADE7953.getInstReactivePowerA(); 
  #ifdef DEBUG_WATTMETER 
  Serial.print("Reactive Power A (mW): ");
  Serial.println(reactivePowerA);
  #endif

  static float prev_powerFactorA;
  powerFactorA = myADE7953.getPowerFactorA(); 
  #ifdef DEBUG_WATTMETER 
  Serial.print("Power Factor A (x100): ");
  Serial.println(powerFactorA);
  #endif

  static long prev_activeEnergyA;
  activeEnergyA = myADE7953.getActiveEnergyA(); 
  #ifdef DEBUG_WATTMETER 
  Serial.print("Active Energy A (hex): ");
  Serial.println(activeEnergyA);
  #endif
}

void read_thermistor_0()  //100k Mixing Thermistor
{
    Wire.beginTransmission(50); // transmit to device #50
    Wire.write(75);              // sends one byte  
    Wire.requestFrom(50, 2);    // request 2 bytes from slave device #50
    byte LSB_1 = Wire.read(); // receive a byte as character
    byte MSB_1 = Wire.read();
    Wire.endTransmission();    // stop transmitting

    Therm_0 = LSB_1 +(MSB_1 << 8);

    Therm_0 = KtoF(ResistanceToTemp10k(VoltageToResistance10k(Therm_0))); 
   
    #ifdef DEBUG_THERM
      Serial.print(MSB_1);
      Serial.print(" + ");
      Serial.print(LSB_1);
      Serial.print(" = ");
      Serial.print(Therm_0);
      Serial.print(" TC0  (MIX)");
      Serial.println();
    #endif
}

void read_thermistor_1() //10k Cold Thermistor 
{
    Wire.beginTransmission(50); // transmit to device #50
    Wire.write(76);              // sends one byte  
    Wire.requestFrom(50, 2);    // request 2 bytes from slave device #50
    byte LSB_1 = Wire.read(); // receive a byte as character
    byte MSB_1 = Wire.read();
    Wire.endTransmission();    // stop transmitting

    Therm_1 = LSB_1 +(MSB_1 << 8);  

    Therm_1 = KtoF(ResistanceToTemp10k(VoltageToResistance10k(Therm_1))); 

    #ifdef DEBUG_THERM
      Serial.print(MSB_1);
      Serial.print(" + ");
      Serial.print(LSB_1);
      Serial.print(" = ");
      Serial.print(Therm_1);  
      Serial.print(" TC1  (COLD)");
      Serial.println();
    #endif
}

void read_thermistor_2() //10k Hot Thermistor
{
    Wire.beginTransmission(50); // transmit to device #50
    Wire.write(77);              // sends one byte
    Wire.requestFrom(50, 2);    // request 2 bytes from slave device #50
    byte LSB_1 = Wire.read(); // receive a byte as character
    byte MSB_1 = Wire.read();
    Wire.endTransmission();    // stop transmitting

    Therm_2 = LSB_1 +(MSB_1 << 8);  

    Therm_2 = KtoF(ResistanceToTemp100k(VoltageToResistance100k(Therm_2)));

    #ifdef DEBUG_THERM
      Serial.print(MSB_1);
      Serial.print(" + ");
      Serial.print(LSB_1);
      Serial.print(" = ");
      Serial.print(Therm_2);
      Serial.print(" TC2  (HOT)");
      Serial.println();
    #endif
}

void read_thermistor_3() // Extra Thermistor
{
    Wire.beginTransmission(50); // transmit to device #50
    Wire.write(78);              // sends one byte
    Wire.requestFrom(50, 2);    // request 2 bytes from slave device #50
    byte LSB_1 = Wire.read(); // receive a byte as character
    byte MSB_1 = Wire.read();
    Wire.endTransmission();    // stop transmitting

    Therm_3 = LSB_1 +(MSB_1 << 8);  

    Therm_3 = KtoF(ResistanceToTemp10k(VoltageToResistance10k(Therm_3)));

    #ifdef DEBUG_THERM
      Serial.print(MSB_1);
      Serial.print(" + ");
      Serial.print(LSB_1);
      Serial.print(" = ");
      Serial.print(Therm_3);
      Serial.print(" TC3 (extra)");
      Serial.println();
    #endif
}

// transfers ananolg read values into actual voltage
double DAQ_to_Volt(double x) 
{
  return x*(3.3/1023);
}
// voltage to distance in (cm) conversion based on measured output as described
// in sensor datasheet (THIS IS ONLY ACCURATE FOR DISTANCES 3-30cm)
// The accuracy of this function has NOT been fully explored
double voltToDist(double x)
{
  return -5.5904*(x*x*x)+34.698*(x*x)-71.345*x+54.993;
}

double read_PIR_sensor()
{
  double samples_dist[sampleLoop]={0};
  double distance = 0;
  for (int i=0; i<sampleLoop; i++) //average the sequential samples
  {
    distance = voltToDist(DAQ_to_Volt(analogRead(A0)))+distance;
    delayMicroseconds(1000);
  }
  distance=distance/(double)sampleLoop;
  return distance;
}

void LED_control()
{
  for(int i = 0; i < 16; i++)
    pwm.setPWM(i, 0, 4096);
  for(int i = 0; i < 16; i++)
    pwm.setPWM(i, 4096, 0);
}   

void handleInterrupt()
{ 
  unsigned int intStatus = io.interruptSource(false);// find out which pin generated an interrupt and clear the SX1509's interrupt output, the true argument clears the interrupt.

  if (intStatus & 4096) //check for pin 12 on sx
  {  
    if(WCB_Status == 1){ //WCB in main operation
      dispense_button_detect();
    }
  }
  else if (intStatus & 64) //check for pin 6 on sx
  {
    if(WCB_Status == 1){ //WCB in main operation
      encoder_rotate_detect();
    }
  }
  else if (intStatus & 32) //check for pin 5 on sx
  {
    if(WCB_Status == 1){ //WCB in main operation
      encoder_rotate_detect();
    }
  }
  else if (intStatus & 128) //check for pin 7 on sx
  {
    if(WCB_Status == 1){ //WCB in main operation
      dispense_temp = target_temp;
      lcd.clear();
      lcd.print("Temp Selected");
    }
  }
  else if (intStatus & 256) //check for pin 8 on sx
  {
    detect=1;
  }
  else if (intStatus & 4) //check for pin 2 on sx
  {
    detect=1;
  }
  else if (intStatus & 8) //check for pin 3 on sx
  {
    // if motion detected on the front sensor, check the distance sensor
    detect=2;   
  }
  else if (intStatus & 16) //check for pin 4 on sx
  {
    detect=1;
  }
  io.interruptSource(true);// find out which pin generated an interrupt and clear the SX1509's interrupt output, the true argument clears the interrupt.}
  
  if(detect==2){
    sensor_Distance = read_PIR_sensor();
  }
}

void callback(char* topic, byte* payload, unsigned int length) 
{
  /*UPDATE LATER*/
  int i = 0;

  char message_buff[100] = {};

  #ifdef DEBUG
    Serial.println("Message arrived:  topic: " + String(topic));
    Serial.println("Length: " + String(length,DEC));
  #endif
  
  // create character buffer with ending null terminator (string)
  for(i=0; i<length; i++) {
    message_buff[i] = payload[i];
  }
  message_buff[i] = '\0';
  
  char *tokenizedString[10] = {};
  int splitStrLen = stringSplit(tokenizedString, message_buff);
  #ifdef DEBUG
    Serial.println("Payload: " + String(message_buff));
  #endif 
  
  if(String(tokenizedString[0]) == "coldTempSet"){
    if(splitStrLen > 1){//if there are enough arguments
      String strVal = String(tokenizedString[1]);
      if(isInteger(strVal)){
        //set cold temperature to tempVal 
        coldTempSet = clip(strVal.toInt(), 40, 65);
          lcd.clear();
		  lcd.setCursor(0, 0);
		  lcd.print("Server:");
		  lcd.setCursor(0, 1);
		  lcd.print("COLD=" + String(coldTempSet));
        //Serial.println("TEST_CALLBACK: coldTempSet" + tempVal);
      }else{
        //argument is not an integer
      }
    }else{
      //there are not enough arguments
      #ifdef DEBUG
        Serial.println("TEST_CALLBACK: Not enough arguments");
      #endif
    }
  }
  else if(String(tokenizedString[0]) == "hotTempSet"){
    if(splitStrLen > 1){//if there are enough arguments
      String strVal = String(tokenizedString[1]);
      if(isInteger(strVal)){
        //set cold temperature to tempVal 
        hotTempSet = clip(strVal.toInt(), 100, 145);
          lcd.clear();
		  lcd.setCursor(0, 0);
		  lcd.print("Server:");
		  lcd.setCursor(0, 1);
		  lcd.print("HOT=" + String(hotTempSet));
        //Serial.println("TEST_CALLBACK: coldTempSet" + tempVal);
      }else{
        //argument is not an integer
      }
    }else{
      //there are not enough arguments
      #ifdef DEBUG
        Serial.println("TEST_CALLBACK: Not enough arguments");
      #endif
    }
  }else{
    //invalid command
  }
}

unsigned long WCBreconnect() 
{
  unsigned long timeElapsed = millis();

  if (!client.connected()) 
  {
    #ifdef DEBUG_WIFI
    Serial.print("Attempting a connection to MQTT...");
    #endif

    #ifndef NO_LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Connecting to MQTT...");
    lcd.setCursor(0, 1);
    #endif

    if (client.connect(MAC_ID.c_str(), emem.getMqttUser().c_str(), emem.getMqttPwd().c_str())) 
    {
      #ifndef NO_LED
      pwm.setPWM(PCA9865_RGB_LED_WIFI_G, 4096, 0);
      #endif

      #ifndef NO_LCD
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("MQTT Connected:");
      lcd.setCursor(0, 1);
      lcd.print("Publishing...");
      #endif

      #ifdef DEBUG_WIFI
      Serial.println("connected");
      #endif
      
      WCB_mqtt_publish("out/"+mqtt_topic_w_MAC_ID+"/reconnect", String(ACTIVE));
      String subscribeStr = "in/"+mqtt_topic_w_MAC_ID+"/control";
      client.subscribe(subscribeStr.c_str());//subscribe to data from main topic

      #ifndef NO_LCD
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("MQTT published.");
      lcd.setCursor(0, 1);
      #endif
    } 
    else 
    {
      #ifdef DEBUG_WIFI
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" will try connecting again in 2 secs");
      #endif

      #ifndef WIFI_UNIT_TEST
      pwm.setPWM(PCA9865_RGB_LED_WIFI_B, 0, 4096);
      #endif

      #ifndef NO_LCD
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Failed to connect :/");
      lcd.setCursor(0, 1);      
      lcd.print("Try in ~30 sec");
      #endif

      #ifndef NO_LED
      pwm.setPWM(PCA9865_RGB_LED_WIFI_B, 0, 4096);
      #endif
      delayMicroseconds(200*1000);
    }
  }else{
      // mqtt connected
  }

  return millis() - timeElapsed;
}

void configModeCallback (WifiManager *myWiFiManager) 
{
  #ifdef DEBUG_WIFI
  Serial.println("Entered config mode");
  #endif
}

/*void resetWifi() 
{
  wifiManager.resetSettings();
}*/

void system_reset()
{
  lcd.clear();
  
  pwm.setPWM(10, 0, 4096); //wifi green
  pwm.setPWM(12, 0, 4096);  //status green
  pwm.setPWM(9, 0, 256); //wifi red
  pwm.setPWM(8, 0, 4096); //wifi blue
  pwm.setPWM(5, 0, 256);  //status red
  pwm.setPWM(11, 0, 4096); //status blue
  
  while(io.digitalRead(ESP32_RESET_BUTTON)==HIGH && rst<3)
  {
    delayMicroseconds(500000);
    lcd.print(".");
    rst++;
  }

  lcd.clear();
  if(rst >= 3)
  {    
    #ifdef DEBUG_WIFI
    Serial.println("Hard reset.");
    #endif

    lcd.print("Hard Reset.");

    resetWifi();
    delayMicroseconds(5000);

    #ifdef DEBUG_WIFI
    Serial.println("Hard Reset finished.");
    #endif

    lcd.clear();
    lcd.print("Hard Reset finsihed.");
    
    wipe_data();
    
    ESP.restart();
   } 
   else{    
    ESP.restart();
   }
   rst=0;
}
/*
void setup_wifi() 
{
   WiFi.mode(WIFI_STA);
   WiFi.disconnect();
   delayMicroseconds(100*1000);
  int n = WiFi.scanNetworks();

  if (n == 0){
	  #ifdef DEBUG_AP
	  Serial.println("No networks found");
	  #endif

	  #ifndef NO_LCD
	  lcd.clear();
	  lcd.setCursor(0, 0);
	  lcd.print("No networks found.");
	  lcd.setCursor(0, 1);
	  lcd.print("Resetting...");

	  lcd.setCursor(0, 0);
	  #endif
    char data[100] = "0#ssid#pw123456789#x#x#x#x#x#x"; //blank String with 0 as EEPROM config value
    emem.saveData(data);
    ESP.restart();
    delayMicroseconds(1000*1000);
  }
 // wifiManager.setAPCallback(configModeCallback); //configModeCallback is void
  //wifiManager.setTimeout(30); //30 seconds to connect //commented out for testing
  #ifdef DEBUG_WIFI
  Serial.println("Setting up Wifi");
  #endif

  if (!wifiManager.autoConnect(MAC_ID.c_str(), "watercoolerbuddy")) 
  {
    #ifdef DEBUG_WIFI
    Serial.println("failed to connect and hit timeout");
    #endif

    #ifndef NO_LCD
	  lcd.clear();
	  lcd.setCursor(0, 0);
	  lcd.print("Failed to connect.");
	  lcd.setCursor(0, 1);
	  lcd.print("Timeout. Restting.");

	  lcd.setCursor(0, 0);
	  #endif
    //reset and try again, or maybe put it to deep sleep
     char data[100] = {};
     configured[0] = emem.getConfigStatus().charAt(0) - 1; 
     data_setup(data); 
     emem.saveData(data);  
    ESP.restart();
    delayMicroseconds(1000*1000);
  }else{//success
    char data[100] = {};

      #ifndef NO_LCD
	  lcd.clear();
	  lcd.setCursor(0, 0);
	  lcd.print("Connected.");
	  lcd.setCursor(0, 1);
	  lcd.print("Resetting and saving.");

	  lcd.setCursor(0, 0);
	  #endif

    configured[0] = '9';
    data_setup(data); 
    emem.saveData(data); 
    ESP.restart();
    delayMicroseconds(1000*1000);
  }
}
*/
void data_setup(char* data)
{
  char* sep = "#";
  strcat(data, configured);
  strcat(data, sep);
  //strcat(data, wifiManager.getSSID().c_str()); //commented out for testing
  strcat(data, sep);
  //strcat(data, wifiManager.getPassword().c_str()); //commented out for testing
  strcat(data, sep);
  strcat(data, mqtt_server);
  strcat(data, sep);
  strcat(data, mqtt_port);
  strcat(data, sep);
  strcat(data, mqtt_user);
  strcat(data, sep);
  strcat(data, mqtt_pwd);
  strcat(data, sep);
  #ifdef DEBUG
  Serial.println("Current values ready to be upated to EEPROM:");
  Serial.println(data);
  Serial.println();
  #endif
}


//Functions for converting thermistor values into temperatures -----------------
double VoltageToResistance10k(double x){
  return 3300/((1023/x)-1);
}

double VoltageToResistance100k(double x){
  return 33000/((1023/x)-1);
}

double ResistanceToTemp10k(double x){
  return 1/(a0+a1*log(x)+a2*pow(log(x),3));
}

double ResistanceToTemp100k(double x){
  return 1/(b0+b1*log(x)+b2*pow(log(x),3));
}

double KtoF(double x){
  return x*(9.0/5.0)-459.67;
}
//------------------------------------------------------------------------------

// Feedback control Function----------------------------------------------------

void FeedbackControl()
{ //modified to simultaneously control temperature hysteresis of both reserviors
  heaterTemp=0; //initialize the temp variable for the averages
  coolerTemp=0;
  
  double samples_hot[sampleLoop]={0};
  double samples_cold[sampleLoop]={0};
  for (int i=0; i<sampleLoop; i++)//read sequential samples
  {
    read_thermistor_0();
    read_thermistor_1();
    read_thermistor_2();
    
    samples_hot[i] = (TC1_gain*Therm_2)+TC1_offset;  //Measurement and calibration of TC input
    samples_cold[i] = (TC1_gain*Therm_1)+TC1_offset;  //Measurement and calibration of TC input 
  }

  for (int i=0; i<sampleLoop; i++) //average the sequential samples
  {
    heaterTemp = samples_hot[i]+heaterTemp;
    coolerTemp = samples_cold[i]+coolerTemp;
  }
  heaterTemp=heaterTemp/(double)sampleLoop;
  coolerTemp=coolerTemp/(double)sampleLoop;

  tempUpdate(); //update available temperatures for UI

  #ifdef DEBUG_FEEDBACK
    Serial.print("AvgHeatTemp(F): "); Serial.println(heaterTemp);
    Serial.print("AvgCoolTemp(F): "); Serial.println(coolerTemp);
  #endif

  //WCB_mqtt_publish(mqtt_topic_w_MAC_ID+"/feedBack", String(heaterTemp, 3) + "-"+ String(coolerTemp, 3));

  byte ThermError1=false; //this is a flag if there is a Thermocouple error
  byte ThermError2=false; //this is a flag if there is a Thermocouple error
  
  feedBack(SX1509_RELAY_HEATER,heaterTemp,ThermError1, hotTempSet, true); // for heater
  feedBack(SX1509_RELAY_COOLER,coolerTemp,ThermError2, coldTempSet, false); // for cooler
}

// code taken directly from Michael Klopfer's "feedback_temp_ctrl" sketch. Made to take variables for modulatiry
void feedBack(int RelayCtrl_1_Pin, double temp_read1, byte TCError1, double temp_set1, bool direct)
{
  if(isnan(temp_read1)) //check for NAN, if this is not done, if the TC messes up, the controller can stick on!
  {
    temp_read1=temp_set1; //fail safe! 
    TCError1=true;
  }
  else
  {
    TCError1=false;
  }
 runtime=millis(); //set runtime
 if (lastruntime>runtime) //check for millis() rollover event, prepare accordingly, skip and wait until time reaccumulates
 {
  overrideindicator=1;
  lastruntime=runtime;
  delayMicroseconds(min_trans_time*100); //delay if overflow event is detected as failsafe
 }


 //direct or inverse control (direct: (true) = heating, (false) = cooling)
if (direct==true) // heating
{
  if (heater_status==0 && temp_read1<=temp_set1-lower_hysteresis_1 && min_trans_time<(runtime-lastruntime) && !overrideindicator)
  {
          io.digitalWrite(RelayCtrl_1_Pin, HIGH);
          lastswitcheventtime = runtime-lastswitcheventtime;  //reset transition time counter (verify no issue with millis() rollover)
          heater_status=1;  //toggle relay status indicator
          lastruntime=runtime;  //update lastruntime variable - used to check switchine period and to permit checking for millis() overflow event
          overrideindicator=0; //reset millis() overflow event indicator
          
  }
   else if (heater_status==1 && temp_read1>=temp_set1+upper_hysteresis_1 && min_trans_time<(runtime-lastruntime) && !overrideindicator)
    {
         io.digitalWrite(RelayCtrl_1_Pin, LOW);
         lastswitcheventtime =  runtime-lastswitcheventtime; //reset transition time counter (verify no issue with millis() rollover)
         heater_status=0;  //toggle relay status indicator
         lastruntime=runtime;   //update lastruntime variable - used to check switchine period and to permit checking for millis() overflow event
         overrideindicator=0; //reset millis() overflow event indicator
    } 
  else {}
}

if (direct==!true) // cooling
{
  if (cooler_status==1 && temp_read1<=temp_set1-lower_hysteresis_1 && min_trans_time<(runtime-lastruntime) && !overrideindicator)
  {
      io.digitalWrite(RelayCtrl_1_Pin, LOW);
      lastswitcheventtime = runtime-lastswitcheventtime;  //reset transition time counter (verify no issue with millis() rollover)
      cooler_status=0;  //toggle relay status indicator
      lastruntime=runtime;  //update lastruntime variable - used to check switchine period and to permit checking for millis() overflow event
      overrideindicator=0; //reset millis() overflow event indicator
          
  }
   else if (cooler_status==0 && temp_read1>=temp_set1+upper_hysteresis_1 && min_trans_time<(runtime-lastruntime) && !overrideindicator)
    {
         io.digitalWrite(RelayCtrl_1_Pin, HIGH);
         lastswitcheventtime = runtime-lastswitcheventtime;  //reset transition time counter (verify no issue with millis() rollover)
         cooler_status=1;  //toggle relay status indicator
         lastruntime=runtime;  //update lastruntime variable - used to check switchine period and to permit checking for millis() overflow event
         overrideindicator=0; //reset millis() overflow event indicator
    } 
  else {}
  }
}
// -----------------------------------------------------------------------------


void power_Down()
{
  if(Previous_WCB_Status != WCB_Status)
  {
    //WCB_mqtt_publish(mqtt_topic_w_MAC_ID+"/status", String(IDLE));
    #ifdef DEBUG
      Serial.println("Powering Down...");
    #endif
    pwm.setPWM(0, 0, 256);
    pwm.setPWM(1, 0, 256);
    pwm.setPWM(2, 0, 256);
    pwm.setPWM(3, 0, 256);
    pwm.setPWM(13, 0, 256);
    //status light
    pwm.setPWM(12, 0, 256);  //status green
    pwm.setPWM(5, 0, 4096);  //status red
    pwm.setPWM(11, 0, 4096); //status blue
    //WIFI LED
    pwm.setPWM(8, 0, 4096); //wifi red
    pwm.setPWM(9, 0, 4096); //wifi blue
    pwm.setPWM(10, 0, 256); //wifi green
    lcd.noBacklight(); 
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Power Save Mode");
    Previous_WCB_Status = WCB_Status; //update status so this function isnt repeated
    sensor_Distance = 40.00; // reset sensor distance
  }
}

void power_Up(){
  if(Previous_WCB_Status != WCB_Status)
  {
   	//WCB_mqtt_publish(mqtt_topic_w_MAC_ID+"/status", String(ACTIVE));
    #ifdef DEBUG
      Serial.println("Powering Up...");
    #endif
    pwm.setPWM(0, 4096, 0);
    pwm.setPWM(1, 4096, 0);
    pwm.setPWM(2, 4096, 0);
    pwm.setPWM(3, 4096, 0);
    pwm.setPWM(13, 4096, 0);
    //status LED
    pwm.setPWM(5, 4096, 0);
    pwm.setPWM(11, 4096, 0);
    pwm.setPWM(12, 4096, 0);
    //WIFI LED
    pwm.setPWM(8, 4096, 0);
    pwm.setPWM(9, 4096, 0);
    pwm.setPWM(10, 4096, 0);
    lcd.clear();
    lcd.backlight();
    lcd.print("Ready For");
    lcd.setCursor(0,1);
    lcd.print("Dispensing");
    Previous_WCB_Status = WCB_Status; //update status so this function isnt repeated
    sensor_Distance = 40.00; // reset sensor distance
  }
}

void welcome()
{
  lcd.clear();
  lcd.backlight();
  lcd.print("Water Cooler Buddy"); 
  lcd.setCursor(0,1);
  lcd.print("Starting up");

  for(int i=4;i>=0;i--){
      if(i==4){ //correction since channel 4 no longer is working
        pwm.setPWM(13,0,4096);
      }
      pwm.setPWM(i,0,4096);
      delayMicroseconds(90000);
  }
  for(int i=0; i<=4 ;i++){
      if(i==4){ //correction since channel 4 no longer is working
        pwm.setPWM(13,4096,0);
      }
      pwm.setPWM(i,4096,0);
      delayMicroseconds(90000);   
  }

  //status LED
  pwm.setPWM(5, 4096, 0);
  pwm.setPWM(11, 4096, 0);
  pwm.setPWM(12, 4096, 0);
  //WIFI LED
  pwm.setPWM(8, 4096, 0);
  pwm.setPWM(9, 4096, 0);
  pwm.setPWM(10, 4096, 0);
  
  delayMicroseconds(900000);
  lcd.clear();
  lcd.print("Ready For");
  lcd.setCursor(0,1);
  lcd.print("Dispensing");
}

double Mixing(int setTemp){
  double ratio = (hotTempSet-setTemp)/hotTempSet;
  
  if(buttonCount<20){ // precaution to ensure there is no memory misalignment
    dispenseTemps[buttonCount]=setTemp; // save dispense temp to array
  }
  return ratio;
}

void Dispensing(double ratio, int dispPeriod){ //function uses hot-to-cold ratio (1=all cooling)
  int START_TIME = millis();
  //found that the hot solenoid puts out about twice as much water per second as the cold solenoid
  int offsetcold = 400;
  int offsethot = 200;
  if(dispensing==1){ //dispensing button flag was triggered
    #ifdef DEBUG_MIXING
      Serial.print("Dispense Ratio: ");
      Serial.println(ratio);
    #endif
    io.digitalWrite(SX1509_SOLENOID_HOT, HIGH);
    delayMicroseconds(dispPeriod*(1-ratio)*offsethot);
    io.digitalWrite(SX1509_SOLENOID_HOT, LOW);
    io.digitalWrite(SX1509_SOLENOID_COLD, HIGH);
    delayMicroseconds(dispPeriod*(ratio)*offsetcold);
    io.digitalWrite(SX1509_SOLENOID_COLD, LOW);
  }
}

int AverageDispenseTemp(){
  int avg = 0;
  if(!buttonCount){
    return 0;
  }
  else if(buttonCount>0){
    for(int i; i<20; i++){
      avg = dispenseTemps[i]/buttonCount;
    }
    return avg;
  }
}

void Set_Scheduling()
{
  currTime = millis();
  detect_time           = currTime;
  check_wattmeter_time  = 2253 + currTime;
  check_adjustTempTime  = 4502 + currTime;
  check_AllowDispense   = 4901 + currTime;
  check_wifi_lcd_time   = currTime;
  check_status_time     = 5008 + currTime + 3*PERIOD; 
  check_mqtt_time		    = 5500 + currTime;
  check_LCD_refresh     = 6500 + currTime;
  check_tempReg         = 60000 + currTime; // wait ~10 min before changing reservior temperature
  check_feedback_time   = 90500 + currTime; //wait 1 min 30 sec before switching back on
}

void wipe_data() 
{
 Serial.println("Wipe EEPROM");
 char* sep = "#";
 EEPROM.begin(512);
 for (int i = 0; i < 6; ++i)
 {
   EEPROM.write(i, (int)sep);
   delayMicroseconds(1000);
   EEPROM.write(i, 50);
   delayMicroseconds(1000);
 }
 EEPROM.commit();
 Serial.println("Wipe data complete");
}

void WCB_mqtt_publish(String topic_string, String msg_string)
{
     char topic_publish[50];
     char msg_publish[50];
     toCharArr(topic_publish, topic_string);
     toCharArr(msg_publish, msg_string);
     client.publish(topic_publish, msg_publish);
}

//function converts a number into a string
void toString(char str[], int num) 
{
   int i, rem, len = 0, n;
   n = num;
   if(n==0){
    len=1;
   }
   while (n != 0)
   {
       len++;
       n /= 10;
   }
   for (i = 0; i < len; i++)
   {
       rem = num % 10;
       num = num / 10;
       str[len - (i + 1)] = rem + '0';
   }
   str[len] = '\0';
}

//function converts a string to a character array
void toCharArr(char *ch, String str)
{
	int len = str.length();
	for(int i = 0; i < len; i++)
	{
		ch[i] = str.charAt(i);
	}
	ch[len] = NULL;
}

// function uses the encoder position value to determine if a user wants
// temperatures currently unavailable and modifies the temperature settings so that WCB
// can dispense at lower/higher temperatures.
void trackEncoderTurn(int temp, int val)
{  
  if(temp==HOT)
  {
    if(hotTempSet<165)
    {
      hotTempSet = hotTempSet + val;
      adjustingTemp = 2;
    }
  }

  else if(temp==COLD)
  {
    if(coldTempSet>=40)
    {
      coldTempSet = coldTempSet - val;
      adjustingTemp = 1;
    }
  }
}

void tempUpdate() //update available heater/cooler temperatures as they are read by thermistors
{
  if(currTime>check_tempReg){ // if ~10 min has passed, the hot and cold set points can start to be manipulated...
    //coldTempSet=newColdTempSET;
    //hotTempSet=newHotTempSET;
  }
}

void WCBWriteMQTT() // function compresses all sendable information for WCB into one topic sent to MQTT server.
{
  if(!client.connected())
    return;

  char WCBStatus[1] = "";
  char ColdTempSet[4] = "";
  char HotTempSet[4] = "";
  char ColdTempCurr[4] = "";
  char HotTempCurr[4] = "";
  char activePower[7] = "";
  char dispenseTemp[4] = "";
  char dispenseCount[10] = ""; //41
  char MQTTSend[100] = "";

  // When adjusting reservior temp, WCB does not create an extra status state.
  // This information is important for the server-side controller however, so we send a
  // "2" to specify this operational mode.
  if(adjustingTemp){
    toString(WCBStatus,2);
  }
  else{
    toString(WCBStatus,WCB_Status);
  }
  toString(ColdTempSet,coldTempSet);
  toString(HotTempSet,hotTempSet);
  toString(ColdTempCurr,coolerTemp);
  toString(HotTempCurr,heaterTemp);
  toString(activePower,WCB_activePower);
  toString(dispenseTemp,dispense_temp);
  toString(dispenseCount,buttonCount);

  //strcpy(MQTTSend,"Stat:");
  strcpy(MQTTSend,WCBStatus);
  strcat(MQTTSend,",");
  //strcat(MQTTSend,"CSet(F):");
  strcat(MQTTSend,ColdTempSet);
  strcat(MQTTSend,",");
  //strcat(MQTTSend,"HSet(F):");
  strcat(MQTTSend,HotTempSet);
  strcat(MQTTSend,",");
  //strcat(MQTTSend,"CRead(F):");
  strcat(MQTTSend,ColdTempCurr);
  strcat(MQTTSend,",");
  //strcat(MQTTSend,"HRead(F):");
  strcat(MQTTSend,HotTempCurr);
  strcat(MQTTSend,",");  
  //strcat(MQTTSend,"Pow(mW):");
  strcat(MQTTSend,activePower);
  strcat(MQTTSend,",");
  //strcat(MQTTSend,"Disp(F):");
  strcat(MQTTSend,dispenseTemp);
  strcat(MQTTSend,",");
  //strcat(MQTTSend,"Disp#:");
  strcat(MQTTSend,dispenseCount);

  //Protocol: MAC_address/status/cold_temp_set/hot_temp_set/cold_temp_current/hot_temp_current/active_power/dispense_temp/dispense_count
  WCB_mqtt_publish("out/" + mqtt_topic_w_MAC_ID + "/" ,MQTTSend);
}

int stringSplit(char *buffer[], char* src)
{
    char *ptr = strtok(src, " ");  // takes a list of delimiters
    int index = 0;
    while(ptr != NULL)
    {
        buffer[index] = ptr;
        index++;
        ptr = strtok(NULL, " ");  // takes a list of delimiters
    }
    return index;
}

int isInteger(String src)
{
  for(int i = 0; i < src.length(); i++)
  {
    if(!isDigit(src.charAt(i)))
      return 0;
  }
  return 1;
}

void EEPROMReset()  
{
  WiFi.disconnect();//Close WiFi Connections
  delay(200);
  //noInterrupts(); //Disable further interrupt calls - Typically you want to turn this off, but this is the end of the line before a reset when an interrupt is called.
  int rst = 0;
  while(eepromRstButton.isPressed())
  {
      esp_task_wdt_feed();//Keep WDT Timer fed to prevent WDT restart
      delay(100); //This is in place to slow down the loop, it will freeze other operations!  Will either exit with a soft or hard reset   
      rst++;
      if(rst>=50) //Check to see if heald for more than 4 seconds, if so, hard reset
      {//reset after 40 loops with button press (4 seconds)
        #ifdef DEBUG_AP
        Serial.println("EEPROM Reset (Hard or Soft) command received");
        //wifiManager.resetSettings();  //No need to use, you are just stacking deck chairs on the Titanic at this point, EEPROM will be erased
        Serial.println("Preparing to default EEPROM (Hard Reset)");
        #endif

        #ifndef NO_LCD
          lcd.clear();
          lcd.setCursor(0, 0);
		  lcd.print("EEPROM Reset Command:");
		  lcd.setCursor(0, 0);
        #endif
        delay(10);
        while(eepromRstButton.isPressed()) //inside the IF statement to check if button is still being pressed (pin pulled low)     
          {
           //Stay in this loop until the button is released.  This prevents bootup issues if the pin is kept low at reset, this is an issue for GPIO2
           esp_task_wdt_feed();
          }
        #ifdef DEBUG_AP
        Serial.println("Now Resetting EEPROM to default and restarting (Hard Reset)");
        #endif

        #ifndef NO_LCD
        lcd.clear();
        lcd.setCursor(0, 0);
  		  lcd.print("EEPROM Hard Resetting");
  		  lcd.setCursor(0, 0);
        #endif

        delay (20);
        char data[100] = "3#ssid#pw123456789#x#x#x#x#x#x"; //blank String with 0 as EEPROM config value
        emem.saveData(data);
        delay (500);
        #ifdef DEBUG_AP
        Serial.println("EEPROM overwrite complete, restarting...");
        #endif
        #ifndef NO_LCD
          lcd.clear();
          lcd.setCursor(0, 0);
		  lcd.print("EEPROM overwrite complete.");
		  lcd.setCursor(0, 0);
        #endif
        ESP.restart();  //Board will reset before leaving loop
        delay (2000);
       }
       else
       {} //Rounding out IF statement
  }   
      #ifdef DEBUG_AP
      Serial.print("Soft Reset command received!  Will be issued on Button release"); //should already be released, this is just a warning
      #endif
      #ifndef NO_LCD
          lcd.clear();
          lcd.setCursor(0, 0);
		  lcd.print("Soft Reset command received.");
		  lcd.setCursor(0, 1);
		  lcd.print("Release to Soft Reset.");
		  lcd.setCursor(0, 0);
        #endif
      while(eepromRstButton.isPressed()) //inside the IF statement to check if button is still being pressed (pin pulled low)     
        {
         //Stay in this loop until the button is released.  This prevents bootup issues if the pin is kept low at reset, this is an issue for GPIO2
         esp_task_wdt_feed(); //Keep WDT Timer fed to prevent WDT restart
        }
        #ifdef DEBUG_AP
        Serial.println("Soft Reset command received, restarting...");
        #endif
        #ifndef NO_LCD
          lcd.clear();
          lcd.setCursor(0, 0);
		  lcd.print("Soft Resetting...");
		  lcd.setCursor(0, 0);
        #endif
      delay (500);
      ESP.restart(); //Button not held for 4 seconds, loop terminates early - this provides dual function for this reset, soft and hard reset depending on hold time
      delay (2000); 
}

void APModeSetup()
{
	emem.loadData();
 
 	#ifdef DEBUG_AP
	  Serial.println();
	  Serial.println("First read");
	  Serial.print("EEPROM recorded Configured state: "); Serial.println( emem.getConfigStatus());
	  Serial.print("EEPROM recorded SSID: ");Serial.println(emem.getWifiSsid());
	  Serial.print("EEPROM recorded PWD: ");Serial.println(emem.getWifiPwd());
	  Serial.println(emem.getMqttServer());
	  Serial.println(emem.getMqttPort());
	  Serial.println(emem.getMqttUser());
	  Serial.println(emem.getMqttPwd());


	  Serial.println();
	  Serial.print("Connecting to ");
	  Serial.println(emem.getWifiSsid());
	#endif

	  #ifndef NO_LCD
	  lcd.clear();
	  lcd.setCursor(0, 0);
	  lcd.print("Connecting to");
	  lcd.setCursor(0, 1);
	  lcd.print(emem.getWifiSsid());

	  lcd.setCursor(0, 0);
	  #endif

	  //configured[0] ='0';//forced into AP mode for testing
	  if(emem.getConfigStatus().charAt(0) <= 48)//**ASCII value of '0' is 48;
	  {
	  	  #ifdef DEBUG_AP
	      Serial.println("Too many timeouts. Resuming without Wifi."); 
	      #endif
	  }
	  else if(emem.getConfigStatus() != "9")//check first character in EEPROM array which is the configured status
	  {
	  	  #ifdef DEBUG_AP
	      Serial.println();
	      Serial.println("Connection Time Out...");
	      Serial.println("Enter AP Mode...");
	      #endif

	      #ifndef NO_LCD
		  lcd.clear();
		  lcd.setCursor(0, 0);
		  lcd.print("APMode: Config-"+ emem.getConfigStatus());
		  lcd.setCursor(0, 1);
	      lcd.print(MAC_ID.c_str());

		  lcd.setCursor(0, 0);
		  #endif

	      //setup_wifi();

	      char data[100] = {};
	      #ifdef DEBUG_AP
	      Serial.println("Credentials set from user response in AP mode.");	      
	      //Serial.print("SSID: "); Serial.println(wifiManager.getSSID().c_str()); //commented out for testing
	      //Serial.print("Password: "); Serial.println(wifiManager.getPassword().c_str()); //commented out for testing  
	      #endif

	      //WiFi.begin(wifiManager.getSSID().c_str(), wifiManager.getPassword().c_str()); //commented out for testing
	      
	      #ifdef DEBUG_AP
	      Serial.println("Configuration entered. Testing connection.");
	      #endif

	      for(int j = 0; WiFi.status() != WL_CONNECTED; j++)
	      {
	      	#ifdef DEBUG_AP
	        Serial.print(".");
	        #endif

	        #ifndef NO_LED
	        pwm.setPWM(PCA9865_RGB_LED_WIFI_B, 4096, 0);
		    delayMicroseconds(200*1000);
		    pwm.setPWM(PCA9865_RGB_LED_WIFI_B, 0, 4096);
		    #endif

	        delay(1000);

	        if(j >= 100)
	        {
	          #ifdef DEBUG_AP
	          Serial.println("Timeout initial connection to AP");
	          #endif

	          #ifndef NO_LCD
	          lcd.clear();
	          lcd.setCursor(0, 0);
	          lcd.print("Time Out...");
	          lcd.setCursor(0, 1);
	          lcd.print("Resetting and entering AP Mode...");
	          #endif

	          configured[0] = emem.getConfigStatus().charAt(0) - 1; 
	          data_setup(data); 
	          emem.saveData(data);  
	             
	          ESP.restart();
	          delay(1000);
	        }
	      }
	  }
	  WiFi.begin(emem.getWifiSsid().c_str(), emem.getWifiPwd().c_str());

	  #ifdef DEBUG_AP
	  Serial.println("Connected");
	  #endif
}
