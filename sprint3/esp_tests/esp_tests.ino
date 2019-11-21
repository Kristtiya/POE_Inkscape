/*
    This sketch establishes a TCP connection to a "quote of the day" service.
    It sends a "hello" message, and then prints received data.
*/

#include <ESP8266WiFi.h>

#ifndef STASSID
#define STASSID "OLIN-DEVICES"
#define STAPSK  "Design&Fabric8"
#endif

const char* ssid     = STASSID;
const char* password = STAPSK;

const char* host = "192.168.33.215";
const uint16_t port = 9090;

const byte numChars = 32;
char inputData[numChars];

void setup() {
  Serial.begin(115200);

  // We start by connecting to a WiFi network

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  /* Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
     would try to act as both a client and an access-point and could cause
     network-issues with your other WiFi-devices on your WiFi-network. */
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  Serial.print("connecting to ");
  Serial.print(host);
  Serial.print(':');
  Serial.println(port);

  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  if (!client.connect(host, port)) {
    Serial.println("connection failed");
    delay(3000);
    return;
  }

  // This will send a string to the server
  Serial.println("sending data to server");
  if (client.connected()) {
    client.println("hello from ESP8266");
  }

  echoServer(c);

  // Close the connection
  Serial.println();
  Serial.println("closing connection");
  client.stop();

  delay(3000); // execute once every 5 minutes, don't flood remote service
}

void echoServer(WiFiClient c) {
  static byte ndx = 0;
  char rc;
  int numCount = 0;
  unsigned long timeout = millis();
  while (c.available() == 0) {
    if (millis() - timeout > 5000) {
      Serial.println("0,0");
      c.stop();
      delay(60000);
      return;
    }
  }
  
  while (c.available()) {
    rc = static_cast<char>(c.read());
    inputData[ndx] = rc;
    ndx++;
    if (ndx >= numChars) {
      ndx = numChars - 1;
    }
  }
  
  inputData[ndx] = '\0'; // terminate the string
  ndx = 0;
  Serial.println(inputData);
}
