//GPS moodmeter for SaMPL Masterclass
//Made by "Chi ha ucciso Il Conte?" from a shopofthings.ch script 
//chihauccisoilconte.eu


/*
  GPS pin:
   ESP 3.3V > GPS VCC
   ESP GND  > GPS GND
   ESP RX   > GPS TX
   ESP TX   > GPS RX
  
*/// Library http://arduiniana.org/libraries/tinygpsplus/ 

#include <TinyGPS++.h> 
#include <HardwareSerial.h> 


//EEPROM Library creates a sort of small memory to save the GPS data

#include "EEPROM.h" 
#define EEPROM_SIZE 128

// the number of the pushbutton pin
const int buttonPin = 34;     
const int button2Pin  = 35;
const int button3Pin = 25;

 // the number of the LED pin

const int ledPin =  32;
const int led2Pin = 18;
const int led3Pin = 33;

// variable for reading the pushbutton status
int buttonState = 0;   
int button2State = 0;
int button3State = 0;
 
//creating the GPS opject

TinyGPSPlus gps;
 
//creating a serial connection with the GPS
HardwareSerial SerialGPS(1);
 
//creating a template to store the GPS variables

struct GpsDataState_t {
  double originLat = 0;
  double originLon = 0;
  double originAlt = 0;
  double distMax = 0;
  double dist = 0;
  double altMax = -999999;
  double altMin = 999999;
  double spdMax = 0;
  double prevDist = 0;
};
GpsDataState_t gpsState = {};

 
//defining the speed for the serial comunication in milliseconds

#define TASK_SERIAL_RATE 1000 // ms
uint32_t nextSerialTaskTs = 0;
uint32_t nextOledTaskTs = 0;


//wifi libraries
#include <Arduino.h>
#if defined(ESP32)
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>

//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
#define WIFI_SSID "FRITZ!Box 7530 JR"
#define WIFI_PASSWORD "58637389311628142843"

// Insert Firebase project API Key
#define API_KEY "AIzaSyBkpIVjeS7dDszxE5htEBNRIaIazYGgLgQ"

// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "https://esp32-firebase-test-46085-default-rtdb.europe-west1.firebasedatabase.app/" 

//Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
int count = 0;
bool signupOK = false;

//defining the mood variable 
int mood = 0;

void setup() {
 
  // Serial ist die Ausgabe im Serial Monitor
  Serial.begin(115200);

   WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("ok");
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

// initialize the led pins as an output:
pinMode(ledPin, OUTPUT);
  pinMode(led2Pin, OUTPUT);
  pinMode(led3Pin, OUTPUT);


  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  pinMode(button2Pin, INPUT);
  pinMode(button3Pin, INPUT);


//  baud: baud rate according to the specification of the GPS module, in this case 9600
//  rxPin: an RX pin e.g. 16
//  txPin: an RX pin e.g. 17
  
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);
 
//Initialize EEPROM memory

  while (!EEPROM.begin(EEPROM_SIZE)) {
    true;
  }
 

  long readValue;
  EEPROM_readAnything(0, readValue);
  gpsState.originLat = (double)readValue / 1000000;
 
  EEPROM_readAnything(4, readValue);
  gpsState.originLon = (double)readValue / 1000000;
 
  EEPROM_readAnything(8, readValue);
  gpsState.originAlt = (double)readValue / 1000000;
 
}
 
//Helper functions to read and write memory easily

template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
  const byte* p = (const byte*)(const void*)&value;
  int i;
  for (i = 0; i < sizeof(value); i++)
    EEPROM.write(ee++, *p++);
  return i;
}
 
template <class T> int EEPROM_readAnything(int ee, T& value)
{
  byte* p = (byte*)(void*)&value;
  int i;
  for (i = 0; i < sizeof(value); i++)
    *p++ = EEPROM.read(ee++);
  return i;
}
 
 

void loop() {



  buttonState = digitalRead(buttonPin);
  button2State = digitalRead(button2Pin);
  button3State = digitalRead(button3Pin);
 
  static int p0 = 0;
 
  // GPS coodinates
  gpsState.originLat = gps.location.lat();
  gpsState.originLon = gps.location.lng();
  gpsState.originAlt = gps.altitude.meters();
 
  // Write current position to ESP32 non-volatile memory
  long writeValue;
  writeValue = gpsState.originLat * 1000000;
  EEPROM_writeAnything(0, writeValue);
  writeValue = gpsState.originLon * 1000000;
  EEPROM_writeAnything(4, writeValue);
  writeValue = gpsState.originAlt * 1000000;
  EEPROM_writeAnything(8, writeValue);
  EEPROM.commit(); 
 
  gpsState.distMax = 0;
  gpsState.altMax = -999999;
  gpsState.spdMax = 0;
  gpsState.altMin = 999999;
 
  /*
Raw data from serial connection to GPS module
   * read in. The data is processed using TinyGPS++
   * The data becomes aware only after the assignment of the variable
   * read so that we can simplify the following
   * Be able to do calculations.
   */
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }
 
  /*

* Various calculations of maximum and minimum values ​​and distance covered
   * These will only be made if at least one fix with 4 satellites is available
   * is, at most the accuracy would not be given and it would be wrong
   * Values ​​are calculated.
   */
  if (gps.satellites.value() > 4) {
    gpsState.dist = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), gpsState.originLat, gpsState.originLon);
 
    if (gpsState.dist > gpsState.distMax && abs(gpsState.prevDist - gpsState.dist) < 50) {
      gpsState.distMax = gpsState.dist;
    }
    gpsState.prevDist = gpsState.dist;
 
    if (gps.altitude.meters() > gpsState.altMax) {
      gpsState.altMax = gps.altitude.meters();
    }
 
    if (gps.speed.kmph() > gpsState.spdMax) {
      gpsState.spdMax = gps.speed.kmph();
    }
 
    if (gps.altitude.meters() < gpsState.altMin) {
      gpsState.altMin = gps.altitude.meters();
    }
  }
 
  /*
 
So that not too much data is output in the serial monitor,
we limit the output to the number of milliseconds
which we have stored in the constant "TASK_SERIAL_RATE".
  */
  if (nextSerialTaskTs < millis()) {
    Serial.print("LAT=");  Serial.println(gps.location.lat(), 6);
    Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
    Serial.print("ALT=");  Serial.println(gps.altitude.meters());
    Serial.print("Sats=");  Serial.println(gps.satellites.value());
    Serial.print("DST: ");
    Serial.println(gpsState.dist, 1);
    nextSerialTaskTs = millis() + TASK_SERIAL_RATE;
  }

  float latdata = gps.location.lat();

//change the mood values

  if (buttonState == HIGH) {
    // turn LED on:
    digitalWrite(ledPin, HIGH);
    mood = 1;
  } else {
    // turn LED off:
    digitalWrite(ledPin, LOW);
    
  }

    if (button2State == HIGH) {
    // turn LED on:
    digitalWrite(led2Pin, HIGH);
    mood = 2;
  } else {
    // turn LED off:
    digitalWrite(led2Pin, LOW);
  }

      if (button3State == HIGH) {
    // turn LED on:
    digitalWrite(led3Pin, HIGH);
    mood = 3;
  } else {
    // turn LED off:
    digitalWrite(led3Pin, LOW);
  }

//we state that if any button is pressed, the ESP32 will send data to firebase

      if (Firebase.ready() && signupOK && buttonState == HIGH or button2State == HIGH or button3State == HIGH) {
    sendDataPrevMillis = millis();
    // Write an Float number on the database path test/int
    if (Firebase.RTDB.setFloat(&fbdo, "test/lat", gps.location.lat())){
      Serial.println("PASSED");
      Serial.println("PATH: " + fbdo.dataPath());
      Serial.println("TYPE: " + fbdo.dataType());
    }
    else {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }
    
    
    // Write an Float number on the database path test/float
    
    if (Firebase.RTDB.setFloat(&fbdo, "test/lon", gps.location.lng())){
      Serial.println("PASSED");
      Serial.println("PATH: " + fbdo.dataPath());
      Serial.println("TYPE: " + fbdo.dataType());
    }
    else {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }

    if (Firebase.RTDB.setFloat(&fbdo, "test/mood", mood)){
      Serial.println("PASSED");
      Serial.println("PATH: " + fbdo.dataPath());
      Serial.println("TYPE: " + fbdo.dataType());
    }
    else {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }    
  }
  
}

//this machine kills fascists
