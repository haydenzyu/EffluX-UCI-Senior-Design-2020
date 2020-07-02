#include "MQTT.h"

String MACID = "";
const char* ssid = "microsemi-test"; //wifi name
const char* password =  "calit2uci123456789"; //wifi password
//const char* ssid = "CalPlugIoT"; //wifi name
//const char* password =  "A8E61C58F8";
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
