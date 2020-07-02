//2019-2020 firmware contributors: Mugared Khalifa, Kejia Qiang

#include "ESP32_Wifi_MQTT.h"

////////////////////////////////INITIALIZERS/////////////////////////////////
String MACID = "";
char* AP_NameString = "Coffee Buddy ESP_AP";//change this if you want to change the AP's name
bool isAPmode = false;
IPAddress ip;
//bool EEPROMResetButtonPress = false ;
//#define EEPROMRST 2 //TODO: configure this to where the reset button will be attached to.
//Button button = Button(EEPROMRST, PULLUP);
String data = "0#mqttserver#mqttport#mqttuser#mqttpassword#";//form used for writing data to eeprom. this will be used whenever the eeprom gets reset.
int size_of_data_written_in_eeprom = data.length();
long lastMsg = 0;
int to_reset = 1;
char mqttServer[25] ; //server name
char mqttPort[10]; //port number, it will be read as a char array, but will be converted to int once used using the "atoi" function
char mqttUser[25] ; //username
char mqttPassword[25] ; //password
char configured[] = {'0', 0};
char ssid[25];
char pwd[25];
int ip0, ip1, ip2, ip3;

WiFiClient espClient;
PubSubClient client(espClient);
WiFiManager wifiManager;
WiFiServer server(80);
// Set web server port number to 80

//these custom parameters will appear as boxes in portal.
//they take the name of the parameter,  the placeholder of it i.e. where the data inputed in the portal will be stored,
//a default value that will be displayed on the portal, and the size of the placeholder.
WiFiManagerParameter custom_mqttServer("mqttServer",  mqttServer,"MQTTSERVER" , 25);
WiFiManagerParameter custom_mqttPort("mqttPort", mqttPort,"MQTTPORT" ,10);
WiFiManagerParameter custom_mqttUser("mqttUser",  mqttUser,"MQTTUSER" ,25);
WiFiManagerParameter custom_mqttPassword("mqttPassword", mqttPassword, "MQTTPASSWORD" , 25);


/////////////////////////////////////////////////////////////////////////////

void start_mqtt() {
  //initializes mqtt connections
  client.setServer(mqttServer, atoi(mqttPort));  // sets the MQTT server details
  client.setCallback(callback);  // callback function for mqtt message handling
  server.begin();
}

// configure wifi settings and establish connection
void setup_wifi() {
  // lcd.clear();
  // lcd.print("Please connect WCB-1 to Wifi...");
    //WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(1000);
    WiFi.softAPdisconnect(true);
    delay(1000);
    //Scans for networks available
    int n = WiFi.scanNetworks(); // gives you NUMBER of networks
    Serial.println("scan done");

    if (n == 0) {
      Serial.println("no networks found");
    } else {
      Serial.print(n);
      Serial.println(" networks found");
      for (int i = 0; i < n; ++i)
       {
        // Print SSID and RSSI for each network found
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(WiFi.SSID(i));
        Serial.print(" (");
        Serial.print(WiFi.RSSI(i));
        Serial.print(")");
        delay(10);
       }
    }
    Serial.println("");
    wifiManager.setAPCallback(configModeCallback);
    Serial.println("Setting up Wifi");

    if (!wifiManager.autoConnect(AP_NameString)) //credentials for SSID in AP mode //creates access point
    {
      //Autoconnect will handle setting up the conifguration portal and connecting to the chosen ssid and password
      //should it fail, Autoconnect handles that and reopens the portal. It does so until it succeeds.
    Serial.println("failed to connect and hit timeout");
    delay(1000);
    }

    //copies the mqtt credentials that were retrieved from the AP connection
    Serial.println("printing values given in the custom params created:");
    Serial.println(custom_mqttServer.getValue());
    Serial.println(custom_mqttUser.getValue());
    Serial.println(custom_mqttPassword.getValue());
    strcpy(mqttServer , custom_mqttServer.getValue());
    strcpy(mqttPort , custom_mqttPort.getValue());
    Serial.print("mqttPort value in setup values: ");
    Serial.print(mqttPort);
    Serial.println("^^^^these values were were given in the custom params created:");
    strcpy(mqttUser, custom_mqttUser.getValue());
    strcpy(mqttPassword , custom_mqttPassword.getValue());
    delay(100);
    char data[150];
    configured[0] = '1';
    eeprom_data_setup(data); // sets up the data string. NOTE: this is a local data array, so it will not affect the global data string.
    Serial.print("data in wifi setup after data_setup");
    Serial.println(data);
    eeprom_save_data(data); // saves the data string just created to the eeprom
    delay(100);


  // connected
  Serial.println("Connected to the WiFi network");
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  // lcd.clear();
  // lcd.print("Wifi is connected");
}

void addCustomParameters()
{
  // this function is called to add the custom parameters to the portal
  wifiManager.addParameter(&custom_mqttServer);
 wifiManager.addParameter(&custom_mqttPort);
  wifiManager.addParameter(&custom_mqttUser);
 wifiManager.addParameter(&custom_mqttPassword);
}

void delayfor(long milliseconds)
{
    long d;
    d = millis();

    while (millis()-d < milliseconds) {

        yield();
    }
}

void wifi_Connector(void *pvParameters)
{


  int trials = 0;
  for(;;)
  {
    if(WiFi.status() != WL_CONNECTED && millis()%20000 == 0)
    {
      WiFiManager localwifiManager;
      localwifiManager.setConfigPortalTimeout(600);
      Serial.println("disconnected");
      trials++;
      if(trials >= 1)
      {
        Serial.println("CALL TO SETUP_WIFI IN MAIN LOOP FUNCTION");
        Serial.println();
        WiFi.mode(WIFI_STA);
        //WiFi.disconnect();
        delay(1000);
        //WiFi.mode(WIFI_AP);
        delay(1000);
        //localwifiManager.setAPCallback(configModeCallback);
        Serial.println("Setting up Wifi");

      if (!localwifiManager.startConfigPortal("WCB_RECONFIG_ESP")) //credentials for SSID in AP mode //creates access point
      {
        //Autoconnect will handle setting up the conifguration portal and connecting to the chosen ssid and password
        //should it fail, Autoconnect handles that and reopens the portal. It does so until it succeeds.
      Serial.println("failed to connect and hit timeout");
      //WiFi.mode(WIFI_STA);
      delay(1000);
      }
    }

    }
    else
    {
      if(millis()%5000 == 0)
      {
        Serial.println("connected");
        // connected
          Serial.println("Connected to the WiFi network");
          // Print local IP address and start web server
          Serial.println("");
          Serial.println("IP address: ");
          Serial.println(WiFi.localIP());
      }
    }
  }

}

void reconnect() {
  // establishing connection to MQTT server
  int count_of_connection_trail_cycles = 0;
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");

    // connects the client with a username and password specified
    if (client.connect("ESP32Client", mqttUser, mqttPassword )) {
      Serial.println("connected");
      client.subscribe("esp32/output");

    } else {
      count_of_connection_trail_cycles++;
      Serial.println("failed with state ");
      Serial.println(client.state());
      delay(2000);
      if(count_of_connection_trail_cycles == 50)
      {
        //if it fails to connect after twenty cycles, it resets the ESP
        //which will make it go back to AP mode.
        reset_wifi_credentails();
        eeprom_save_data(data);
        ESP.restart();
      }
    }
  }
}

// handle received messages
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message received on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
}


void enterWiFiManager() {
  Serial.println();
  Serial.println("Connection Time Out...");
  Serial.println("Entering Wifi Manager Mode...");
  //wifiManager.resetSettings();
  setup_wifi();

  Serial.println("Credentials set from user response in Wifi Manager mode.");
}

void connectToWifi() {
  // this functions handles the case where the ESP has already been configured before
  // and connects to the saved Credentials
  //NOTE: The WiFiManager handles where the previous SSID and Password
  // are saved, so calling autoconnect finds these values and connects to them automatically
  //WiFi.softAPdisconnect();
  WiFi.disconnect();
  //WiFi.mode(WIFI_STA);
  delay(1000);
  Serial.print("SSID: "); Serial.println(wifiManager.getSSID());
  Serial.print("Password: "); Serial.println(wifiManager.getPassword());

  bool connected = wifiManager.autoConnect(AP_NameString);
  for (int j = 0; WiFi.status() != WL_CONNECTED; j++) {
    Serial.print(".");
    delay(1000);

    if(j >= 100){
      //if connecting to the previous SSID and PASSWORD takes long
      // it will reset(eeprom) and restart the ESP
      Serial.println("Timeout initial connection to AP");
      configured[0] = '0';
      char data[150];
      // eeprom_data_setup(data);
      // eeprom_save_data(data);
      //EEPROMResetButtonAction();
      delay(1000);
    }
  }
  Serial.print("I'm Connected to WiFi! My IP is: ");
  //print the local IP address
  ip = WiFi.localIP();
  Serial.println(ip);
  load_eeprom_data();
}

void setupWifiClient()
{
  WiFi.disconnect();
  Serial.println("Connecting to " + String(ssid));

  if(strlen(pwd) != 0){
    WiFi.begin(ssid, pwd);
    MACID = WiFi.macAddress();  
  }

  else{
     Serial.println("NO PASSWORD");
    WiFi.begin(ssid);
  }

  // try 30 times to connect
  int triesMax = 30;
  int tries = 0;

  // WL_CONNECTED is a constant defined in some library...
  while ((WiFi.status() != WL_CONNECTED) && (tries < triesMax)) {
    //blinkOnce(LEDR, 500);
    delay(500);
    Serial.print(".");
    tries++;
  }
  Serial.println();
}


// void EEPROMResetButtonPressISR()
// {
//   EEPROMResetButtonPress = true;
// }

// void EEPROMResetButtonAction()
// {
//   Serial.println();
//   Serial.println("CALL TO RESET EEPROM");
//   Serial.println();
//   WiFi.disconnect();//Close WiFi Connections
//   if (button.uniquePress()) {
//     Serial.println("BUTTON HAS BEEN PRESSED");
//     Serial.println("button has been pressed");
//     //String data = "0#ssid#psswd#mqttserver#mqttport#mqttuser#mqttpassword";//just used for reference
//     //resets all credentials saved on memory
//     //both on the eeprom and god knows where the WiFIManager saved the wifi credentials
//     reset_wifi_credentails();
//     eeprom_save_data(data);
//     load_eeprom_data();
//     delay (500);
//     Serial.println("EEPROM overwrite complete, restarting...");
//     configured[0] = 0;
//     ESP.restart();  //Board will restart before leaving loop
//  } else {
//    Serial.println("Not pressed");
//    Serial.println();

//  }
//   delay (2000);
// }

void load_eeprom_data()
{
  //Loads the data written in the eeprom to the fields declared at the topic (mqttUser,mqttPort...)
  Serial.print("size of data written in eeprom");
  Serial.println(size_of_data_written_in_eeprom);
  Serial.println("Call to read data from EEPROM");
  EEPROM.begin(512);
  int count = 0;
  int address_of_prev_seperator = -1;//does not exist since it starts at data[0] and there is no #(the seperator) before that
  char temp_data[300] = {};//used to print out what was written in eeprom
  Serial.println("begin my func reading data");
  Serial.println();
  for(int address = 0; address<size_of_data_written_in_eeprom+10; address++)
  {
    //Loops through the number of addresses there are(size of what was written to it)
    //and whenever it encounters a seperator '#', it calls a helper function copy data to fields
    // then switch cases which field to write the data to,
    // and updates the address_of_prev_seperator
    char read_char = char(EEPROM.read(address));
    temp_data[address] = read_char;
    //char data_to_write_to_fields[address] = read_char;
    //load the data read into fields used(mqttpassword,user....)
    if(read_char == '#')
    {
      //String data = "0#mqttserver#mqttport#mqttuser#mqttpassword#";//form used for writing data to eeprom. this will be used whenever the eeprom gets reset.
      switch(count)
      {
        case 0:
        Serial.println("case 0:");
        Serial.println(temp_data);
        configured[0] = temp_data[0];//did this manually to not worry about weird cases in writing the copy_data_to_field function :)
        break;
        case 1:
        Serial.println("case 1:");
        Serial.println(temp_data);
        copy_data_to_field(temp_data, mqttServer,address_of_prev_seperator, address);
        break;
        case 2:
        Serial.println("case 2:");
        Serial.println(temp_data);
        copy_data_to_field(temp_data, mqttPort,address_of_prev_seperator, address);
        break;
        case 3:
        Serial.println("case 3:");
        Serial.println(temp_data);
        copy_data_to_field(temp_data, mqttUser,address_of_prev_seperator, address);
        break;
        case 4:
        Serial.println("case 3:");
        Serial.println(temp_data);
        copy_data_to_field(temp_data, mqttPassword,address_of_prev_seperator, address);
        break;
      }
      count++;
      address_of_prev_seperator = address;

    }
  }
  Serial.print("data stored in eeprom:");
  Serial.println(temp_data);
  Serial.println();
  Serial.println("end my func reading data");
  delay(100);
  Serial.println("<--Read data complete, this was read");
  Serial.println("printing field values to confirm they were copied to them from memory");
  Serial.println(mqttServer);
  Serial.println(mqttPort);
  Serial.println(mqttUser);
  Serial.println(mqttPassword);
  Serial.println("end of printing credentials' field values");
  Serial.println();
}

void copy_data_to_field(char* this_data, char* field, int address_of_prev_seperator,int end_address)
{
  //This function takes the address of the seperator before what we want to write
  //to the data fields, and the end address which is the address of the seperator after
  //what we want to write to the data field, then does a one by one copying to the char
  //array of the data field and what has been read from the eeprom
  Serial.println("copy_data_to_field called:");
  Serial.print("end address: ");
  Serial.print(end_address);
  Serial.print("address of prev seperator: ");
  Serial.println(address_of_prev_seperator);
  int j = 0;
  for(int i = address_of_prev_seperator+1; i<end_address; i++)
  {
    Serial.print(i);
    Serial.println(this_data[i]);
    field[j] = this_data[i];
    j++;
  }
}


void eeprom_save_data(String this_data)
{
  //Saves this_data to eeprom by looping through each char in the
  //this_data stringand copies it to a corresponding address in the eeprom
  Serial.println(this_data);
  Serial.println("Call to write data to EEPROM");
  size_of_data_written_in_eeprom = this_data.length();
  EEPROM.begin(512);
  for (int i = 0; i<this_data.length(); i++) {
    EEPROM.write(i, (int)this_data[i]);
    EEPROM.commit();
    delay(10);
  }

  Serial.println("Write data complete");
  Serial.print("Data after writing in save_data):");
  Serial.println(data);
  load_eeprom_data();// read back in data to verify write was proper

  delay(100);
}


void esp_wifi_mqtt_setup()
{
  //////////////////////////WIFI SETUP/////////////////////////////

  //pinMode(modepin, INPUT_PULLUP);
  //pinMode(EEPROMRST, INPUT_PULLUP);
  Serial.println("Now adding parameters");
  addCustomParameters();//calls the WiFiManager.addParameter function on each custom paramter added
  EEPROM.begin(512);//512 is how big the memory on the esp is
  //NOTE: the line below should be commented out. only uncomment it when you want to 'reset' the esp
  //EEPROM.write(0,0);
  Serial.println((char)EEPROM.read(0));
  if ((char)EEPROM.read(0)!= '1') {
    //reads first value in eeprom, which is the bit signifiying if data is written or not
    //if no values written to EEPROM save default values to  EEPROM (at this point the string data is set to default values)
    Serial.println("No values written to the EEPROM");
    eeprom_save_data(data);
  }
  load_eeprom_data();

  // check the EEPROM data
  //String data = "0#mqttserver#mqttport#mqttuser#mqttpassword";
  //Serial.print("EEPROM successful client mode conection: "); Serial.println(success);


  Serial.println("\nEEPROM properly configured. Ready to continue.");
  Serial.println();
  Serial.println("First read");
  Serial.print("EEPROM recorded Configured state: "); Serial.println(configured);
  // Serial.print("EEPROM recorded SSID: ");Serial.println(ssid);
  // Serial.print("EEPROM recorded PWD: ");Serial.println(pwd);
  Serial.print("EEPROM recorded MQTTSERVER: "); Serial.println(mqttServer);
  Serial.print("EEPROM recorded MQTTPORT: "); Serial.println(mqttPort);
  Serial.print("EEPROM recorded MQTTUSER: "); Serial.println(mqttUser);
  Serial.print("EEPROM recorded MQTTPASSWORD: "); Serial.println(mqttPassword);
  Serial.println();
  // if (configured[0] != '1')
  // { //if you get AP mode to work make it check if in client mode; need to set default port if so
  //   Serial.println("not configured");
  //   enterWiFiManager();
  // }
  // else
  // {
  //   Serial.println("configured");
  //   connectToWifi();
  // }
  enterWiFiManager();
  //////////////////////////MQTT SETUP///////////////////////////////////

  start_mqtt();

  ///////////////////////////////////////////////////////////////////////


  /////////////////////////////////////////////////////////////////////
}


void eeprom_data_setup(char* data)
{
  //Sets up the given data string to contain what the fields above store(mqttUser...) in them.
  //String data = "0#mqttserver#mqttport#mqttuser#mqttpassword#";//form used for writing data to eeprom. this will be used whenever the eeprom gets reset.
  //char data [150];
  Serial.println("inside data setup");
  char* sep = "#";
  strcat(data, configured);
  strcat(data, sep);
  strcat(data, mqttServer);
  strcat(data, sep);
  strcat(data, mqttPort);
  strcat(data, sep);
  strcat(data, mqttUser);
  strcat(data, sep);
  strcat(data, mqttPassword);
  strcat(data, sep);

  Serial.print("Current values ready to be updated to EEPROM: ");
  Serial.println(data);

  Serial.println();
}

void reset_wifi_credentails(){
  //Magical function that handles erasing the wifi credentials the WiFiManager library
  //saves. Look at https://github.com/espressif/arduino-esp32/issues/400 for more information.
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); //load the flash-saved configs
  esp_wifi_init(&cfg); //initiate and allocate wifi resources (does not matter if connection fails)
  //wait a bit
  delay(200); //Don't use vTaskDelay, we don't want other tasks to run while resetting.
  if(esp_wifi_restore()!=ESP_OK)
  {
      Serial.println("WiFi is not initialized by esp_wifi_init ");
   }else{
       Serial.println("WiFi Configurations Cleared!");
   }
   //continue
  delay(300);//Don't use vTaskDelay, we don't want other tasks to run while resetting.
  //line below has been commented out as it has no use. it will force the esp into an infinte loop of setting up
  // esp_restart();//restart to reset configs
}




void configModeCallback(WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  // print the ssid that we should connect to configure the ESP32
  Serial.print("Created config portal AP named:  ");
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

//publishes the char* message via MQTT, please include a topic and message
void pub_msg(char * topic, char * msg) {
  // establish connection to mqtt
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  #ifdef DEBUG_WIFI
    Serial.println("publishing mqtt message");
  #endif

  client.publish(topic,msg);
}

void send_data(String attribute, float value, String unit, char timestamp[]) {
    char buffer[512];
    DynamicJsonDocument doc(1024);
    //doc["deviceid"] = MACID;
    doc["value"] = value;    
    doc["unit"] = unit;
    doc["timestamp"] = timestamp;  
    //doc["attribute"] = attribute;
    size_t n = serializeJson(doc, buffer);
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
    client.publish(("out/devices/"+MACID+"/"+attribute).c_str(), buffer, n);
}
// void check_connectivity()
// {
//   while ((WiFi.status() != WL_CONNECTED) && (tries < triesMax)) {
//     //blinkOnce(LEDR, 500);
//     delay(500);
//     Serial.print(".");
//     tries++;
//   }
//   Serial.println();
// }
