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
char serialData[numChars] = "0.0,0.0>"; // Data from Serial input -> Server
char clientData[numChars] = "0.0,0.0>"; // Data from client req (Server) -> Serial
static byte ndx = 0;
char rc;
char endMarker = '>';
unsigned long timeout = millis();
//bool zoops = false;

void setup() {
  Serial.begin(115200);
  /* Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
     would try to act as both a client and an access-point and could cause
     network-issues with your other WiFi-devices on your WiFi-network. */
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

void loop() {
  // Use WiFiClient class to create TCP connections
//  WiFiClient client;
//  if (!client.connect(host, port)) {
//    Serial.println("15.0,15.0>");
//    delay(3000);
//    return;
//  }

  // This will send a string to the server
//  if (client.connected()) {
//    echoSerial(client);
//  }
//
//  echoServer(client);
//
//  client.stop();
  pushServer();
  pushSerial();
  delay(3000); // execute once every 3 seconds, don't flood remote service
}

void pushServer() {
  WiFiClient c;
  if (!c.connect(host, port)) {
    return;
  }

  c.print(serialData);
  c.println(">");

  timeout = millis();
  if (c.connected()) {
    while (c.available() == 0) {
      if (millis() - timeout > 5000) {
        c.stop();
        return;
      }
    }

    while (c.available()) {
      rc = static_cast<char>(c.read());
      if (rc != endMarker) {
        clientData[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        clientData[ndx] = '\0'; // terminate the string
        ndx = 0;
        c.stop();
        return;
      }
    }
  
    clientData[ndx] = '\0'; // terminate the string
    ndx = 0;
    c.stop();
    return;
  }
}

void pushSerial() {
  Serial.print(clientData);
  Serial.println(">");

  timeout = millis();
  while (Serial.available() == 0) {
    if (millis() - timeout > 1000) {
      return;
    }
  }

  while (Serial.available()) {
    rc = Serial.read();
    if (rc != endMarker) {
      serialData[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    }
    else {
      serialData[ndx] = '\0'; // terminate the string
      ndx = 0;
      return;
    }
  }

  serialData[ndx] = '\0'; // terminate the string
  ndx = 0;
  return;
}

//void echoServer(WiFiClient c) {
//  static byte ndx = 0;
//  char rc;
//  int numCount = 0;
//  unsigned long timeout = millis();
//  while (c.available() == 0) {
//    if (millis() - timeout > 5000) {
//      Serial.println("0.0,0.0>");
//      c.stop();
//      delay(3000);
//      return;
//    }
//  }
//
//  zoops = true;
//
//  while (c.available()) {
//    rc = static_cast<char>(c.read());
//    clientData[ndx] = rc;
//    ndx++;
//    if (ndx >= numChars) {
//      ndx = numChars - 1;
//    }
//  }
//
//  clientData[ndx] = '\0'; // terminate the string
//  ndx = 0;
//  Serial.print(clientData);
//  Serial.println(">");
//}
//
//void echoSerial(WiFiClient c) {
//  static byte ndx = 0;
//  char rc;
//  int numCount = 0;
//  unsigned long timeout = millis();
//  while (Serial.available() == 0) {
//    if (millis() - timeout > 1000) {
//      if (zoops) {
//        c.println("15.0,15.0>");
//      }
//      else {
//        c.print(serialData);
//        c.println(">");
//      }
//      return;
//    }
//  }
//
//  while (Serial.available()) {
//    rc = Serial.read();
//    if (rc != endMarker) {
//      serialData[ndx] = rc;
//      ndx++;
//      if (ndx >= numChars) {
//        ndx = numChars - 1;
//      }
//    }
//    else {
//      serialData[ndx] = '\0'; // terminate the string
//      ndx = 0;
//    }
//  }
//  if (zoops) {
//    c.println("15.0,15.0>");
//  }
//  else {
//    c.print(serialData);
//    c.println(">");
//  }
//}
