// This program records a time stamp and analog input at 200 times per second 
// and prints all them to the serial monitor in the form data,time

#include "CurieTimerOne.h"
#include <QueueArray.h>

int frequency = 5000; // rate at which the hart rate is checked
                      // (in microseconds): works out to 200 hz

QueueArray<int> queue; // holds the data to be printed

void setup() { // called when the program starts
  
  Serial.begin(57600); // set up the serial monitor
  while(!Serial);     // wait for the serial monitor

  queue.setPrinter(Serial);

  pinMode(10, INPUT); // Setup for leads off detection LO +
  pinMode(11, INPUT); // Setup for leads off detection LO -

  CurieTimerOne.start(frequency, &updateHeartRate);  // set timer and callback
  
}

void loop() {  // called continuously 

  if(!(queue.count() < 2)){ // make sure there are at least two items
                            // in the queue before we print them
    
    Serial.print(queue.dequeue()); // print the time stamp
    Serial.print(","); // separate them by commas for easy csv processing later
    Serial.println(queue.dequeue()); // print the ecg measurement
    
  }

}

void updateHeartRate(){ // interrupt handler
 
  int stamp = (int) micros(); // get the time stamp in microseconds
  int ecg = analogRead(A0);   // get the ecg measurement from the sensor
  queue.enqueue(stamp);       // add the time stamp and the ecg measurement
  queue.enqueue(ecg);   // to the queue of things to be printed
  
}

