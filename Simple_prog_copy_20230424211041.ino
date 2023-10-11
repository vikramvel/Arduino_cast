/**
 * This implements a simple chromecast control example, tested on wemos D1 mini,
 * but should work on any ESP8266 (and probably ESP32)
 * Current status gathered from chromecast will be printed on serial.
 * Buttons on D5/D6/D7 will act as pause/prev/next respectively
 */

#include <Arduino.h>
#include "ArduCastControl.h"
#include <ESP8266WiFi.h>

// Rotary Encoder Inputs
#define Clock D1   //Clock pin connected to D9
#define Data D2    //Data pin connected to D8
#define Push D4   //Push button pin connected to D10 

#define CHROMECASTIP "192.168.1.15"
#define SECRET_SSID "MenOfLetters"
#define SECRET_PASS "Vik19932016!"

char ssid[] = SECRET_SSID;      // your network SSID (name)
char pass[] = SECRET_PASS;   // your network password

IPAddress staticIP(192,168,1,20);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

/////////////////////////////////////////////////////////////////
int counter = 0;                    //Use this variable to store "steps"
int currentStateClock;              //Store the status of the clock pin (HIGH or LOW)
int lastStateClock;                 //Store the PREVIOUS status of the clock pin (HIGH or LOW)
String currentDir ="";              //Use this to print text 
unsigned long lastButtonPress = 0;  //Use this to store if the push button was pressed or not

/////////////////////////////////////////////////////////////////

ArduCastControl cc = ArduCastControl();
bool bPaused = false, bLeftPressed=false, bRightPressed=false;
int status = WL_IDLE_STATUS;
double volume = 0;
bool pause = false;


void setup() {
   pinMode(Clock,INPUT_PULLUP);
  pinMode(Data,INPUT_PULLUP);
  pinMode(Push, INPUT_PULLUP);

  Serial.begin(115200);
  Serial.println("booted");

  WiFi.mode(WIFI_STA);
  WiFi.config(staticIP, gateway, subnet);
  status = WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());
  
   /*  Set encoder pins as inputs with pullups. If you use the Encoder Module, you don't need 
   *  pullups for Clock and Data, only for the push button.*/

  // Setup Serial Monitor

  // Read the initial state of Clock pin (it could be HIGH or LOW)
  lastStateClock = digitalRead(Clock);

}

uint32_t lastUpdated = 0;
uint32_t updatePeriod = 500;

uint32_t bLastUpdated = 0;
uint32_t bUpdatePeriod = 25;

void loop() {
Counting();
  /*
  delay(1);
  //wait for 5s to boot - this is useful in case of a bootloop to keep OTA running
      //Serial.print("Connecting...12345");
//printWifiStatus();

  if ( millis() - lastUpdated > updatePeriod ) {
    if ( cc.getConnection() != WAIT_FOR_RESPONSE ){
      cc.dumpStatus();
    }
    int st;

    if ( cc.getConnection() == DISCONNECTED ){
      Serial.print("Connecting...");
      st = cc.connect(CHROMECASTIP);
      Serial.println(st);
    } else {
      //at this point, cc.volume and cc.isMuted should be valid 
      connection_t c = cc.loop();
      if ( c == WAIT_FOR_RESPONSE || c == CONNECT_TO_APPLICATION ){
        updatePeriod = 50; 
      } else if ( c == APPLICATION_RUNNING ){
        updatePeriod = 500;
        //at this point, all public fields describing the casting
        //(e.g. cc.artist, cc.title) should be valid
      } else {
        updatePeriod = 5000;
      }
    }
    lastUpdated = millis();
  }
  if ( millis() - bLastUpdated > bUpdatePeriod && cc.getConnection() == APPLICATION_RUNNING ){
    if((volume || bPaused != pause)){
    int status_v = cc.setVolume(1, volume);
    
    Serial.print("volume:");
    Serial.println(volume);

    //Serial.println(status_v);
    bLastUpdated = millis();
    volume = 0;
    status = cc.pause(pause);
    bPaused = pause;
  }}
  */
}


/////////////////////////////////////////////////////////////////
void Counting() {
  
  // Read the current state of CLK
  currentStateClock = digitalRead(Clock);

  // If last and current state of Clock are different, then "pulse occurred"
  // React to only 1 state change to avoid double count
  if (currentStateClock != lastStateClock  && currentStateClock == 1){

    // If the Data state is different than the Clock state then
    // the encoder is rotating "CCW" so we decrement
    if (digitalRead(Data) != currentStateClock) {
      //volume =  - 0.1;
      //Serial.println(volume);
      currentDir ="Counterclockwise";
    } else {
      // Encoder is rotating CW so increment
      //volume =  0.1;
      //Serial.println(volume);
      currentDir ="Clockwise";
    }
    Serial.print("Direction: ");
    Serial.print(currentDir);
    Serial.print(" | Counter: ");
    Serial.println(counter);
  }

  // We save last Clock state for next loop
  lastStateClock = currentStateClock;

  // Read the button state
  int btnState = digitalRead(Push);

  //If we detect LOW signal, button is pressed
  if (btnState == LOW) {
    //if 50ms have passed since last LOW pulse, it means that the
    //button has been pressed, released and pressed again
    if (millis() - lastButtonPress > 50) {
      Serial.println(pause);
      pause = !pause;
    }

    // Remember last button press event
    lastButtonPress = millis();
  }

  // Put in a slight delay to help debounce the reading
  delay(1);
}
/////////////////////////////////////////////////////////////////