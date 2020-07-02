#include <SPI.h>
#include "Adafruit_MAX31855.h"

//thermocouple pins
#define MAXDO   19 //SPI MOSI
#define MAXCS1_TOP  9 //SPI DO
#define MAXCS3_BOILER  10 //SPI DO
#define MAXCS2_BOT 11 //SPI DO
#define MAXCLK  18 //SPI clock

//relay pins on esp32
#define RLY_TH_PWR 27
#define RLY_BH_PWR 14
#define RLY_BOILER_PWR 12

//Thermocouple declaration
Adafruit_MAX31855 thermoboil(MAXCLK, MAXCS1_TOP, MAXDO);
Adafruit_MAX31855 thermobot(MAXCLK, MAXCS2_BOT, MAXDO);
Adafruit_MAX31855 thermotop(MAXCLK, MAXCS3_BOILER, MAXDO);

//Thermocouple calibration constants
#define TC1_gain 0.988 //Thermocouple correction gain
#define TC1_offset -2.5  //Thermocouple correction offset

//#define direct true //direct or inverse control (direct (true) = heating, reverse (false) = cooling)
#define upper_hysteresis_1 1.5  //value for overshoot hysteresis (degrees)
#define lower_hysteresis_1 2  //value for undershoot hysteresis (degrees)
#define min_trans_time 5000 // minimum time in a state (value in ms), prevents rapid switching for compressors or likewise which can not be rapidly cycled

int sampleloop = 10; //number of averaged reads for calculation (increased number reduces outliers and improves percision at the cost of responsiveness)
double temp_set_boiler = 155.0;  // *F  - Control setpoint value (in degrees F)  (degrees F is used because of marginally higher resolution per degree)
double temp_set_top = 155.0;
double temp_set_bottom = 155.0;

//Initialization of control variables
unsigned long lastswitcheventtime=0; //time in millis() since last switch event
double temp_read_boiler;  //Temperature that system is currently maintaining
double temp_read_top;
double temp_read_bottom;
unsigned long runtime=0; //tracker for runtime
unsigned long lastruntime=0;//used to detect millis() rollover and indicate transitions
byte TCError1 = false; //this is a flag if there is a Thermocouple error
byte RelayStatus_1 = 0; //status of relay
byte overrideindicator =0; //used to detect millis() rollover

double heaterTemp = 0;
byte ThermError1=false; //this is a flag if there is a Thermocouple error
byte ThermError2=false; //this is a flag if there is a Thermocouple error
volatile byte heater_status = LOW;
volatile byte cooler_status = LOW;
// PID PID_bot(&temp_read1, &Output_T1, &temp_set1, 1, 5, 0.5, PID::DIRECT);
// PID PID_top(&temp_read1, &output2, &temp_set2, 1, 5, 0.5, PID::DIRECT);
// PID PID_boil(&temp_read1, &output3, &temp_set3, 1, 5, 0.5, PID::DIRECT);



void setup(){
    Serial.begin(115200);
    //while (!Serial) delay(1); // wait for Serial on Leonardo/Zero, etc
    Serial.println("MAX31855 test");
    
    // wait for MAX chip to stabilize
    delay(500);
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
          digitalWrite(RelayCtrl_1_Pin, HIGH);
          lastswitcheventtime = runtime-lastswitcheventtime;  //reset transition time counter (verify no issue with millis() rollover)
          heater_status=1;  //toggle relay status indicator
          lastruntime=runtime;  //update lastruntime variable - used to check switchine period and to permit checking for millis() overflow event
          overrideindicator=0; //reset millis() overflow event indicator

  }
   else if (heater_status==1 && temp_read1>=temp_set1+upper_hysteresis_1 && min_trans_time<(runtime-lastruntime) && !overrideindicator)
    {
         digitalWrite(RelayCtrl_1_Pin, LOW);
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
      digitalWrite(RelayCtrl_1_Pin, LOW);
      lastswitcheventtime = runtime-lastswitcheventtime;  //reset transition time counter (verify no issue with millis() rollover)
      cooler_status=0;  //toggle relay status indicator
      lastruntime=runtime;  //update lastruntime variable - used to check switchine period and to permit checking for millis() overflow event
      overrideindicator=0; //reset millis() overflow event indicator

  }
   else if (cooler_status==0 && temp_read1>=temp_set1+upper_hysteresis_1 && min_trans_time<(runtime-lastruntime) && !overrideindicator)
    {
         digitalWrite(RelayCtrl_1_Pin, HIGH);
         lastswitcheventtime = runtime-lastswitcheventtime;  //reset transition time counter (verify no issue with millis() rollover)
         cooler_status=1;  //toggle relay status indicator
         lastruntime=runtime;  //update lastruntime variable - used to check switchine period and to permit checking for millis() overflow event
         overrideindicator=0; //reset millis() overflow event indicator
    }
  else {}
  }
}
// -----------------------------------------------------------------------------

void FeedbackControl()
{ //modified to simultaneously control temperature hysteresis of both reserviors
  heaterTemp=0; //initialize the temp variable for the averages

  double samples_boiler[sampleloop]={0};
  for (int i=0; i<sampleloop; i++)//read sequential samples
  {
    double Therm_0 = thermoboil.readFahrenheit();
    Serial.println("\nBoiler temp: ");
    Serial.print(Therm_0);
    samples_boiler[i] = (TC1_gain*Therm_0)+TC1_offset;  //Measurement and calibration of TC input
  }

  for (int i=0; i<sampleloop; i++) //average the sequential samples
  {
    heaterTemp = samples_boiler[i]+heaterTemp;
  }
  heaterTemp=heaterTemp/(double)sampleloop;

  //?idk what this is for?tempUpdate(); //update available temperatures for UI

  #ifdef DEBUG_FEEDBACK
    Serial.print("AvgHeatTemp(F): "); Serial.println(heaterTemp);
    Serial.print("AvgCoolTemp(F): "); Serial.println(coolerTemp);
  #endif

  //WCB_mqtt_publish(mqtt_topic_w_MAC_ID+"/feedBack", String(heaterTemp, 3) + "-"+ String(coolerTemp, 3));

  ThermError1=false; //this is a flag if there is a Thermocouple error
  ThermError2=false; //this is a flag if there is a Thermocouple error

  feedBack(RLY_BOILER_PWR,heaterTemp,ThermError1, temp_set_boiler, true); // for boiler
}


void loop(){
    double c = thermoboil.readFahrenheit();
    if (isnan(c)) {
        Serial.println("Something wrong with thermocouple!");
    } else {
        Serial.print("C = ");
        Serial.println(c);
    }
    // double a = thermotop.readFahrenheit();
    // double b = thermobot.readFahrenheit();

    // Serial.println("Thermoboil: ");
    // Serial.print(c);
    // Serial.println("Thermotop: ");
    // Serial.print(a);
    // Serial.println("Thermobot: ");
    // Serial.print(b);
    FeedbackControl();
}

