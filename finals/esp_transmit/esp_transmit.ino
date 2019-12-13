/*
    This sketch is for an ESP8266 chip.
    It establishes a TCP connection to a computer at the host/port parameters specified.
    It communicates to the computer via the socket, and to the Arduino via the Serial connection.
    It uses two buffers to store data from the Arduino and computer, and operates in two steps:
      1.a First, it sends data from the Arduino data buffer to the computer server
      1.b It stores the response in the server buffer.
      2.a Now, it sends data from the server buffer to the Arduino.
      2.b It will store the response from the Arduino in the Arduino buffer
      When either the server or the Arduino doesn't respond, it will not update the buffer.

    This sketch will operate as fast as the chip's clock and operations will allow it.
    That means the delay in communications comes from the speed of the TCP comms.

    @author: Shashank Swaminathan
    This sketch is based off the WiFiClient example in the ESP8266 library.
*/

#include <ESP8266WiFi.h>

// Use ifndef to catch if network information is already defined
#ifndef STASSID
#define STASSID "*********"
#define STAPSK  "*********"
#endif

// Set up network, host, and other related TCP information
const char* ssid     = STASSID;
const char* password = STAPSK;

const char* host = "192.168.35.49";
const uint16_t port = 9090;

// Initialize interal buffers and other variables involved in reading from external buffers
const byte numChars = 32;
char serialData[numChars] = "0.0,0.0,0.0>"; // Data from Serial input -> Server
char clientData[numChars] = "0.0,0.0,1.0>"; // Data from client req (Server) -> Serial
char failData[numChars] = "2.0,2.0,2.0>"; // Data if failed
static byte ndx = 0;
char rc;
char startMarker = '<';
char endMarker = '>';
unsigned long timeout = millis();
bool zoops = false;
static bool rip = false;

void setup() {
  // Begin serial
  Serial.begin(115200);
  /* Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
     would try to act as both a client and an access-point and could cause
     network-issues with your other WiFi-devices on your WiFi-network. */
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Loop here till connected
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

void loop() {
  pushServer(); // Server comm part of code
  pushSerial(); // Serial comm part of code
}

void pushServer() {
  // Bind to host
  WiFiClient c;
  if (!c.connect(host, port)) {
    return; // If failed, stop trying
  }

  // If flag is true, then proceed as normal. Otherwise, send 'failed' message
  if (zoops) {
    c.print("<");
    c.print(serialData);
    c.print(">");
  }
  else {
    c.print("<");
    c.print(failData);
    c.print(">");
  }

  /* Wait for server to prepare response
   * If the wait time exceeds 5 seconds, timeout and return
   */ 
  timeout = millis();
  if (c.connected()) {
    while (c.available() == 0) {
      if (millis() - timeout > 5000) {
        c.stop();
        return;
      }
    }

    /* Code is motivated from online forum named Serial Input Basics
     * Read it here: https://forum.arduino.cc/index.php?topic=396450.0
     * It will explain how this code functions.
     */
    rip = false;
    while (c.available()) {
      rc = static_cast<char>(c.read());
      if (rip == true) {
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
          c.stop(); // Always make sure to terminate connections
          return;
        }
      }
      else if (rc == startMarker) {
        rip = true;
      }
    }
  
    clientData[ndx] = '\0'; // terminate the string
    ndx = 0;
    c.stop(); // Always make sure to terminate connections
    return;
  }
}

void pushSerial() {
  // Send whatever is on internal server data buffer to Arduino
  Serial.print("<");
  Serial.print(clientData);
  Serial.print(">");

  /* Wait for Arduino to prepare response
   * If the wait time exceeds 1 second, timeout and return
   */ 
  timeout = millis();
  while (Serial.available() == 0) {
    if (millis() - timeout > 1000) {
      zoops = false;
      return;
    }
  }

  /* Code is motivated from online forum named Serial Input Basics
   * Read it here: https://forum.arduino.cc/index.php?topic=396450.0
   * It will explain how this code functions.
   */
  rip = false;
  while (Serial.available()) {
    rc = Serial.read();
    if (rip == true) {
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
        zoops = true;
        return;
      }
    }
    else if (rc == startMarker) {
      rip = true;
    }
  }

  zoops = true;

  serialData[ndx] = '\0'; // terminate the string
  ndx = 0;
  return;
}
