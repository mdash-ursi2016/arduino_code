// This program prints tests how accurate TimerOne is by printing out a timestamp 
// every 5000 milliseconds. We'll use a python script to see if the timestamps 
// are properly spaced out.

#include "CurieTimerOne.h"

int i = 0; // counter

int frequency = 5000; // rate at which the hart rate is checked
                      // (in microseconds): works out to 200 hz

int heartrate; // holds current (fake) heart rate from analog
int rates[1000]; // holds the first 1000 heart rates calculated

unsigned long timestamp; // holds the current timestamp
unsigned long stamps[1000]; // holds all the timestamps

void setup() {
  
  Serial.begin(9600); // set up the serial monitor
  while(!Serial);     // wait for the serial monitor
  
  pinMode(10, INPUT); // Setup for leads off detection LO +
  pinMode(11, INPUT); // Setup for leads off detection LO -

  CurieTimerOne.start(frequency, &updateHeartRate);  // set timer and callback

  delay(10000); // wait 10 seconds so the arrays have time to get filled
                // before we print them out

  // print all the timestamps and heart rates
  for (int j = 0; j < 1000; j++){
    Serial.println(stamps[j]); 
    // Serial.println(rates[j]);
  }

  Serial.println("done");

}

void loop() {
  
}

void updateHeartRate(){

  // we're getting 1000 data points for this test
  if(i < 1000){

    heartrate = analogRead(A0); // fake heart rate
    timestamp = micros(); // timestamp

    rates[i] = heartrate;  // store these values in an array
    stamps[i] = timestamp; //   so we can print them afterward

    i++; // increment the counter
      
  } 
}

