#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiManager.h>
#include <PubSubClient.h>

//const char* ssid = "CalPlugIoT"; //wifi name
//const char* password =  "A8E61C58F8"; //wifi password
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

void setup() {
  Serial.begin(115200);
  Serial.println("Setting Access Point...");

  setup_wifi();

  client.setServer(mqttServer, mqttPort);  // sets the MQTT server details
  client.setCallback(callback);  // callback function for mqtt message handling
  server.begin();
}

// configure wifi settings and establish connection
void setup_wifi() {
  // disconnect to previous wifi settings
//  WiFi.disconnect();
//  delay(100);
//  int n = WiFi.scanNetworks();
//  Serial.println("scan done");
//  if (n == 0)
//    Serial.println("no networks found");
//  else {
//    Serial.print(n);
//    Serial.println(" networks found");
//    for (int i = 0; i < n; ++i) {
//      // Print SSID and RSSI for each network found
//      Serial.print(i + 1);
//      Serial.print(": ");
//      Serial.print(WiFi.SSID(i));
//      Serial.print(" (");
//      Serial.print(WiFi.RSSI(i));
//      Serial.print(")");
//      Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN )?" ":"*"); //ENC_TYPE_NONE is for ESP8266 and is assumed to be a byte = 7? Note:  TKIP (WPA) = 2, WEP = 5, CCMP (WPA) = 4, NONE = 7, AUTO = 8
//      delay(10);
//    }
//  }
//  Serial.println("");
//  wifiManager.setAPCallback(configModeCallback);  //fetches ssid and pass and tries to connect
//  wifiManager.setTimeout(60);

  // initializes the WiFi library's network settings and provides current status
  //WiFi.begin(ssid, password);

  // autoconnect to previous network settings or input new network credentials
  // connect to "AutoConnectAP" connection on phone or laptop and configure wifi
  wifiManager.autoConnect("ESP32_APmode");

  // establishing connection to WiFi
  // loop until connection successful
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connecting to WiFi..");
    delay(500);
  }

  // connected
  Serial.println("Connected to the WiFi network");
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // establishing connection to MQTT server
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");

    // connects the client with a username and password specified
    if (client.connect("ESP32Client", mqttUser, mqttPassword )) {
      Serial.println("connected");
      client.subscribe("esp32/output");

    } else {
      Serial.println("failed with state ");
      Serial.println(client.state());
      delay(2000);
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

void configModeCallback(WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  // print the ssid that we should connect to configure the ESP32
  Serial.print("Created config portal AP named:  ");
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

void loop() {
  // establish connection to mqtt
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // send test message every 10 seconds to MQTT server
  long now = millis();
  if (now - lastMsg > 10000) {
    lastMsg = now;
    Serial.println("Sending message");
    client.publish("esp32/test", "Message sent");
  }
  Serial.println(WiFi.status());
}
