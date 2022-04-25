////////////////////////////////////////////////////////////////
// Advanced Lights controller
// ESP8266
// Name / Date
// Piotr Kosek / 08.04.2022 
////////////////////////////////////////////////////////////////

#include <Wire.h>
// Handshaking lines
#define DEMAND D1
#define GRANTED D2

// Safe way of pulling the DEMAND line high and low.
// For high, the pin is set as an input_pullup, pulled to logic 1.
// For low, the pin is set as an output, pulled to logic 0.
#define DEMAND_HI pinMode(D1,INPUT_PULLUP);
#define DEMAND_LO digitalWrite(D1, LOW); \
                  pinMode(D1, OUTPUT);

// Using the built-in LED on the ESP8266.                  
#define LED D4
#define LED_ON digitalWrite(LED, LOW)
#define LED_OFF digitalWrite(LED, HIGH)

// Defining the RESOURCE MANAGER FSM states 
#define noTriggerNotGranted 0 // idle state
#define triggerNotGranted 1   // trigger detected, waiting to be granted 
#define triggerMaster 2       // access granted, esp is master

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>

static char command;
static bool espTrigger;
static int espState;
int count = 0;

// Give your access point a name and a password
const char* ssid = "Embedded IoT AP 2022";
const char* password = "password7";

// Create instance of the ESP8266WebServer with a port number of
// 80 for HTTP.
ESP8266WebServer server(80);

// This is the 'root'/'index page. This can *typically* be 
// accessed at http://192.168.4.1. This contains two radio
// buttons and a submit button
const char INDEX_HTML[] =
"<!DOCTYPE HTML>"
"<html>"
"<head>"
"<title>IoT final project ESP</title>"
"</head>"
"<body>"

"<form action=\"/ledon\">"
"<input type=\"submit\" value=\"LED ON\">"
"</form>"

"<form action=\"/ledoff\">"
"<input type=\"submit\" value=\"LED OFF\">"
"</form>"

"<h1>Traffic Lights system controller: </h1>"

"<form action=\"/equalPriority\">"
"<input type=\"submit\" value=\"Equal Priority\">"
"</form>"

"<form action=\"/set1Priority\">"
"<input type=\"submit\" value=\"Set 1 Priority\">"
"</form>"

"<form action=\"/set2Priority\">"
"<input type=\"submit\" value=\"Set 2 Priority\">"
"</form>"

"<form action=\"/maintenance\">"
"<input type=\"submit\" value=\"Maintenance Mode\">"
"</form>"

"<form action=\"/barrierCrossing\">"
"<input type=\"submit\" value=\"Barrier Crossing Mode\">"
"</form>"

"<form action=\"/motorRacing\">"
"<input type=\"submit\" value=\"Motor Racing Start Mode\">"
"</form>"

"<h1>Tri-coloured LED controller: </h1>"

"<form action=\"/setTriLEDoff\">"
"<input type=\"submit\" value=\"Set Tri-coloured LED off\">"
"</form>"

"<form action=\"/setTriLEDamber\">"
"<input type=\"submit\" value=\"Hazard\">"
"</form>"

"<form action=\"/setTriLEDblue\">"
"<input type=\"submit\" value=\"Emergency\">"
"</form>"

"<form action=\"/setTriLEDgreen\">"
"<input type=\"submit\" value=\"Doctor on Call\">"
"</form>"

"<form action=\"/setTriLEDred\">"
"<input type=\"submit\" value=\"Stationary Police Car\">"
"</form>"

"<form action=\"/getStatus\">"
"<input type=\"submit\" value=\"GetStatus\">"
"</form>"


"</body>"
"</html>";


// This is our '404' Error - Page Not Found page
const char NOTFOUND_HTML[] =
"<!DOCTYPE HTML>"
"<html>"
"<head>"
"<title>404 Not Found</title>"
"<body>"
"<h1>Oops! It's 404-time! Page not found!</h1>"
"</body>"
"</html>";


// This is our '500' Error - Internal Server Error
const char INTERNAL_SERVER_ERROR_HTML[] =
"<!DOCTYPE HTML>"
"<html>"
"<head>"
"<title>500 Internal Server Error</title>"
"<body>"
"<h1>500 Internal Server Error</h1>"
"<p>LED mode unsupported</p>"
"</body>"
"</html>";

void handleLEDon()
{
    digitalWrite(BUILTIN_LED, LOW);   // Inverse logic
    server.send(200, "text/html", INDEX_HTML);
}

void handleLEDoff()
{
    digitalWrite(BUILTIN_LED, HIGH);   // Inverse logic
    server.send(200, "text/html", INDEX_HTML);
}

void handleNotFound()
{
    server.send(404, "text/html", NOTFOUND_HTML);
}

void handleEqual()
{
    command = 0x61;
    espTrigger = true;
    server.send(200, "text/html", INDEX_HTML);
}

void handleSet1()
{
    command = 0x62;
    espTrigger = true;
    server.send(200, "text/html", INDEX_HTML);
}

void handleSet2()
{
    command = 0x63;
    espTrigger = true;
    server.send(200, "text/html", INDEX_HTML);
}

void handleMaintenance()
{
    command = 0x64;
    espTrigger = true;
    server.send(200, "text/html", INDEX_HTML);
}

void handleBarrierCrossing()
{
    command = 0x65;
    espTrigger = true;
    server.send(200, "text/html", INDEX_HTML);
}

void handleMotorRacingStart()
{
    command = 0x66;
    espTrigger = true;
    server.send(200, "text/html", INDEX_HTML);
}

void handleTriLEDoff()
{
    command = 0x67;
    espTrigger = true;
    server.send(200, "text/html", INDEX_HTML);
}

void handleTriLEDamber()
{
    command = 0x68;
    espTrigger = true;
    server.send(200, "text/html", INDEX_HTML);
}

void handleTriLEDblue()
{
    command = 0x69;
    espTrigger = true;
    server.send(200, "text/html", INDEX_HTML);
}

void handleTriLEDgreen()
{
    command = 0x6A;
    espTrigger = true;
    server.send(200, "text/html", INDEX_HTML);
}

void handleTriLEDred()
{
    command = 0x6B;
    espTrigger = true;
    server.send(200, "text/html", INDEX_HTML);
}

void handleGetStatus()
{
    command = 0x6C;
    espTrigger = true;
    server.send(200, "text/html", INDEX_HTML);
}

void writeLED(bool LEDon)
{
    // Note inverted logic for NodeMCU 12E ESP8266
    if (LEDon)
    {
        digitalWrite(BUILTIN_LED, 0);
    }
    else
    {
        digitalWrite(BUILTIN_LED, 1);
    }
}

bool granted()
{
  return (!digitalRead(D2));   //if line is low then resource was granted, returns true
}     

void setup()
{


    DEMAND_HI;
    pinMode(GRANTED, INPUT_PULLUP);
  
    espState = noTriggerNotGranted;
    espTrigger = false;

    pinMode(LED, OUTPUT);
    Wire.begin(D6, D7);
    
    // Set-up the LED GPIO
    pinMode(BUILTIN_LED, OUTPUT);
    writeLED(false);

    Serial.begin(9600);
    WiFi.softAP(ssid, password);
    Serial.println("");
    Serial.println(""); Serial.print("Connected to ");
    Serial.println(ssid); Serial.print("IP address: ");
    Serial.println(WiFi.softAPIP());

    // Associate functions for root, ledon, ledoff and "Not Found"

    server.on("/ledon", handleLEDon);
    server.on("/ledoff", handleLEDoff);
    server.on("/equalPriority", handleEqual);
    server.on("/set1Priority", handleSet1);
    server.on("/set2Priority", handleSet2);
    server.on("/maintenance", handleMaintenance);
    server.on("/barrierCrossing", handleBarrierCrossing);
    server.on("/motorRacing", handleMotorRacingStart);
    
    server.on("/setTriLEDoff", handleTriLEDoff);
    server.on("/setTriLEDamber", handleTriLEDamber);
    server.on("/setTriLEDblue", handleTriLEDblue);
    server.on("/setTriLEDgreen", handleTriLEDgreen);
    server.on("/setTriLEDred", handleTriLEDred);
    server.on("/getStatus", handleGetStatus);
    server.onNotFound(handleNotFound);
    server.begin();

    // Kick-start the server
    server.begin();

    Serial.print("Connect to http://");
    Serial.println(WiFi.softAPIP());
}

void loop()
{
    server.handleClient();
    
   // RESOURCE MANAGER MODULE
   {
    switch(espState)
    {
      case noTriggerNotGranted:
        DEMAND_HI;
        LED_OFF;
        if(espTrigger)
          espState = triggerNotGranted;
        else
          espState = noTriggerNotGranted;
       break;
  
      case triggerNotGranted:
        DEMAND_LO;
        LED_OFF;
        if(granted())
          espState = triggerMaster;
        else
          espState = triggerNotGranted;
       break;

      case triggerMaster:
        DEMAND_LO;
        LED_ON;
        if(!espTrigger) 
          espState = noTriggerNotGranted;
        else
          espState = triggerMaster;  
       break;
      default: espState = noTriggerNotGranted;
    }
   }

   if(espState == triggerMaster)
      {
        Wire.beginTransmission(8);
        Wire.write(command);
        Serial.println(command);
        Wire.endTransmission();
        espTrigger = false;
      }
}
