//Bluetooth touch sensor for SaMPL Masterclass
//Made by "Chi ha ucciso Il Conte?"  
//chihauccisoilconte.eu

#include <Arduino.h>
#include <BLEMidi.h>

// set pin numbers
const int touchPin1 = 12; 
const int touchPin2 = 14; 
const int touchPin3 = 27; 
const int touchPin4 = 33; 
const int touchPin5 = 32; 


const int ledPin1 = 16;
const int ledPin2 = 17;
const int ledPin3 = 5;
const int ledPin4 = 18;
const int ledPin5 = 19;


// change with your threshold value
const int threshold = 40;
// variable for storing the touch pin value 
int touchValue1;
int touchValue2;
int touchValue3;
int touchValue4;
int touchValue5;

void setup(){
  Serial.begin(115200);
  Serial.println("Initializing bluetooth");
  BLEMidiServer.begin("Basic MIDI device");
  Serial.println("Waiting for connections...");

  
  delay(1000); // give me time to bring up serial monitor
  // initialize the LED pin as an output:
  pinMode (ledPin1, OUTPUT);
  pinMode (ledPin2, OUTPUT);
  pinMode (ledPin3, OUTPUT);
  pinMode (ledPin4, OUTPUT);
  pinMode (ledPin5, OUTPUT);
}

void loop(){
  // read the state of the pushbutton value:
  touchValue1 = touchRead(touchPin1);
  touchValue2 = touchRead(touchPin2);
  touchValue3 = touchRead(touchPin3);
  touchValue4 = touchRead(touchPin4);
  touchValue5 = touchRead(touchPin5);
  
  //serials to check the values
  //Serial.print(touchValue1);
  // check if the touchValue is below the threshold
  // if it is, set ledPin to HIGH




//several "ifs" to trigger midi notes
  
  if(touchValue1 < threshold && BLEMidiServer.isConnected()){
    // turn LED on

    digitalWrite(ledPin1, HIGH);
    Serial.println(" - LED on");
      BLEMidiServer.noteOn(0, 69, 127);
      delay(1000);
      BLEMidiServer.noteOff(0, 69, 127);        // Then we make a delay of one second before returning to the beginning of the loop
      
  }
  else{
    // turn LED off
    digitalWrite(ledPin1, LOW);
    Serial.println(" - LED off");
  }

    //Serial.print(touchValue2);
  // check if the touchValue is below the threshold
  // if it is, set ledPin to HIGH

  
  if(touchValue2 < threshold && BLEMidiServer.isConnected()){
    // turn LED on
    digitalWrite(ledPin2, HIGH);
    //Serial.println(" - LED on");
      BLEMidiServer.noteOn(0, 70, 127);
            delay(1000);

      BLEMidiServer.noteOff(0, 70, 127);        // Then we make a delay of one second before returning to the beginning of the loop
  }
  else{
    // turn LED off
    digitalWrite(ledPin2, LOW);
    //Serial.println(" - LED off");
  }

  //Serial.print(touchValue3);
  // check if the touchValue is below the threshold
  // if it is, set ledPin to HIGH
  if(touchValue3 < threshold  && BLEMidiServer.isConnected()){
    // turn LED on
    digitalWrite(ledPin3, HIGH);
    //Serial.println(" - LED on");

  BLEMidiServer.noteOn(0, 71, 127);
        delay(1000);

      BLEMidiServer.noteOff(0, 71, 127);        // Then we make a delay of one second before returning to the beginning of the loop
    
  }
  else{
    // turn LED off
    digitalWrite(ledPin3, LOW);
   // Serial.println(" - LED off");
  }

    //Serial.print(touchValue4);
  // check if the touchValue is below the threshold
  // if it is, set ledPin to HIGH
  if(touchValue4 < threshold && BLEMidiServer.isConnected()){
    // turn LED on
    digitalWrite(ledPin4, HIGH);
   // Serial.println(" - LED on");
      BLEMidiServer.noteOn(0, 72, 127);
            delay(1000);

      BLEMidiServer.noteOff(0, 72, 127);        // Then we make a delay of one second before returning to the beginning of the loop
  }
  else{
    // turn LED off
    digitalWrite(ledPin4, LOW);
   // Serial.println(" - LED off");
  }
  
    //Serial.print(touchValue5);
  // check if the touchValue is below the threshold
  // if it is, set ledPin to HIGH
  if(touchValue5 < threshold && BLEMidiServer.isConnected()){
    // turn LED on
    digitalWrite(ledPin5, HIGH);
    Serial.println(" - LED on");
   BLEMidiServer.noteOn(0, 73, 127);
           delay(1000);

      BLEMidiServer.noteOff(0, 73, 127);        // Then we make a delay of one second before returning to the beginning of the loop
      
    
  }
  else{
    // turn LED off
    digitalWrite(ledPin5, LOW);
   // Serial.println(" - LED off");
  }

  
  delay(500);
}

//this machine kills fascists
