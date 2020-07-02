#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

String MACID = "";
const char* ssid = "microsemi-test"; //wifi name
const char* password =  "calit2uci123456789"; //wifi password
// const char* ssid = "BadNamingConventions"; //wifi name
// const char* password =  "exoticoboe065"; //wifi password

const char* mqttServer = "soldier.cloudmqtt.com"; //server name
const int mqttPort =  17322; //port number
const char* mqttUser = "postjxtf"; 
const char* mqttPassword = "Irb7kj1qz2DR";
long lastMsg = 0;
 
WiFiClient espClient;
PubSubClient client(espClient);
WiFiManager wifiManager;
// Set web server port number to 80
WiFiServer server(80);

// function to send data from device to database
void message_response(char component, String processor, String method, String value) {
    char buffer[512];
    DynamicJsonDocument doc(1024);
    //doc["deviceid"] = MACID;
    doc["value"] = value;    
    size_t n = serializeJson(doc, buffer);
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
    client.publish(("out/devices/"+MACID+"/"+component+"/"+processor+"/"+method).c_str(), buffer, n);
}

void callback(char* topic, byte* payload, unsigned int len) {
  Serial.print("Message received on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < len; i++) {
    Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }
  Serial.println();

  char *subtopics[10];
  int i = 0
  char *token;
  token = strtok(topic, "/");
  while(token != NULL) {
    subtopics[i] = token;
    Serial.println(subtopics[i])
    token = strtok(NULL, "/");
    i++;
  }

  int component = (int) subtopics[3];
  String processor = subtopics[4];
  String method = subtopics[5];

  int value = 0;
  switch(component) {
    case 1:
      if (method == 'On') { 
        value = 1;
        turn_on_boiler(); }
      else if (method == 'Off') { 
        value = 0;
        turn_off_boiler(); }
      message_response('1', processor, method, value)
    case 2:
      if (method == 'On') { 
        value = 1;
        turn_on_top(); }
      else if (method == 'Off') { 
        value = 0;
        turn_off_top(); }
      message_response('2', processor, method, value)
    case 3:
      if (method == 'On') { 
        value = 1;
        turn_on_bottom(); }
      else if (method == 'Off') { 
        value = 0;
        turn_off_bottom(); }
      message_response('3', processor, method, value)
    default:
      Serial.println(messageTemp)
  }
}

void reconnect() {
  // establishing connection to MQTT server
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");

    // connects the client with a username and password specified
    if (client.connect("ESP32Client", mqttUser, mqttPassword )) {
      Serial.println("connected");
      client.subscribe("in/devices/#");
 
    } else {
      Serial.println("failed with state ");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

void setup_mqtt() {
  Serial.begin(115200);
  //Serial.println("Setting Access Point...");
  WiFi.begin(ssid, password);
  MACID = WiFi.macAddress();  
  Serial.println(MACID);  // 3C:71:BF:F8:A6:58
  //setup_wifi();
  
  client.setServer(mqttServer, mqttPort);  // sets the MQTT server details
  client.setCallback(callback);  // callback function for mqtt message handling
  server.begin();

  if (!client.connected()) {
      reconnect();
  }
  client.loop();
}

// {"table":"data",
// "fields":["device_id","value","unit","timestamp"],
// "values":["123",7,"kW","2020-01-31T15:00:00"]}


// function to send data from device to database
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
    client.publish(("out/devices/"+MACID+"/0/Wattmeter/"+attribute).c_str(), buffer, n);
}
