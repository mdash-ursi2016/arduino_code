/*  This program obtains live ECG readings from a sensor, uses the Pan-Tompkins 
 *  QRS-detection algorithm to calculate the corresponding Beats Per Minute (BPM), 
 *  and sends the BPM data to a central device using the BLE Heart Rate Service. */

#include "CurieTimerOne.h"
#include <QueueArray.h>
#include <stdio.h>
#include <stdlib.h>
#include <BLEPeripheral.h>
#include <BLEDescriptor.h>
#include <BLEUuid.h>
#include <BLECommon.h>
#include <BLEAttribute.h>
#include <BLETypedCharacteristics.h>
#include <CurieBLE.h>
#include <BLECentral.h>
#include <BLEService.h>
#include <BLECharacteristic.h>
#include <BLETypedCharacteristic.h>

/* The portions of this code that implement the Pan-Tompkins QRS-detection algorithm were 
 *  modified from code taken from Blake Milner's real_time_QRS_detection GitHub repository:
 https://github.com/blakeMilner/real_time_QRS_detection/blob/master/QRS_arduino/QRS.ino */

const int LED_PIN = 13;                  // the number of the LED pin (digital)
const int ECG_PIN = A0;                  // the number of the ECG pin (analog)

// pins for leads off detection
const int LEADS_OFF_PLUS_PIN  = 10;      // the number of the LO+ pin (digital)
const int LEADS_OFF_MINUS_PIN = 11;      // the number of the LO- pin (digital) 

const int frequency = 5000; // rate at which the heart rate is checked
                      // (in microseconds): works out to 200 hz

QueueArray<int> bpm_queue; // holds the BPMs to be printed

QueueArray<int> ecg_queue; // holds the ECGs to be printed

int ecg_q_count = 0; // used to space out the ECG measurements sent to

// hold the ECG measurements in the array that is sent to the phone
unsigned char zero, one, two, three, four, five, six, seven,
  eight, nine, ten, eleven, twelve, thirteen;


// Bluetooth stuff --------------------------------

bool ble_connected = false; // keeps track of whether the Bluetooth is connected

BLEPeripheral blePeripheral;       // BLE Peripheral Device (the board you're programming)
BLEService myService("aa7b3c40-f6ed-4ffc-bc29-5750c59e74b3"); // BLE Heart Rate Service

// Custom BLE characteristic to hold BPM
BLECharacteristic bpmChar("95d344f4-c6ad-48d8-8877-661ab4d41e5b",  
    BLERead | BLENotify, 1);  

// Custom BLE characteristic to hold chunks of raw ECG measurements
BLECharacteristic ecgChar("1bf9168b-cae4-4143-a228-dc7850a37d98", 
    BLERead | BLENotify, 14); 

BLECentral central = blePeripheral.central(); // the Blutooth central device (the phone)
    
// ------------------------------------------------


// QRS-detection stuff-----------------------------

#define M             5   // for high pass filter
#define N             30  // for low pass filter
#define winSize       200 // size of the window 
#define HP_CONSTANT   ((float) 1 / (float) M)
#define MAX_BPM       100

// resolution of RNG
#define RAND_RES 100000000

// timing variables
unsigned long foundTimeMicros = 0;        // time at which last QRS was found
unsigned long old_foundTimeMicros = 0;    // time at which QRS before last was found

// interval at which to take samples and iterate algorithm (microseconds)
const long PERIOD = 1000000 / winSize;

// circular buffer for BPM averaging
float bpm = 0; 
#define BPM_BUFFER_SIZE 5
unsigned long bpm_buff[BPM_BUFFER_SIZE] = {0};
int bpm_buff_WR_idx = 0;
int bpm_buff_RD_idx = 0;

int tmp = 0;

// ---------------------------------------------------


void setup() { // called when the program starts

    /* Set a local name for the BLE device
     This name will appear in advertising packets
     and can be used by remote devices to identify this BLE device
     The name can be changed but maybe be truncated based on space left in advertisement packet */
  blePeripheral.setLocalName("YOOOUUUU");
  blePeripheral.setAdvertisedServiceUuid(myService.uuid());  // add the service UUID
  blePeripheral.addAttribute(myService);   // add the BLE service
  blePeripheral.addAttribute(bpmChar); // add the BPM characteristic
  blePeripheral.addAttribute(ecgChar); // add the ECG characteristic

  
  Serial.begin(115200); // set up the serial monitor
  while(!Serial);     // wait for the serial monitor
  ecg_queue.setPrinter(Serial); // allows the ecg queue to print error messages

    /* Now activate the BLE device.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection */
  blePeripheral.begin();
  Serial.println("Bluetooth device active, waiting for connections...");

  pinMode(LEADS_OFF_PLUS_PIN, INPUT); // Setup for leads off detection LO +
  pinMode(LEADS_OFF_MINUS_PIN, INPUT); // Setup for leads off detection LO -

  CurieTimerOne.start(frequency, &updateHeartRate);  // set timer and callback
  
}

void loop() { // called continuously

  if(ble_connected){ 
    
    if(central.connected()){ // check if we're still connected to the phone
      
      if(!bpm_queue.isEmpty()){ // check if there's the BPM to print

        // remove a BPM from the queue and send it to the phone
        int heartRate = bpm_queue.dequeue();
        unsigned char bpmCharArray[1] = { (unsigned char)heartRate };
        bpmChar.setValue(bpmCharArray, 1); 
          
      }

      if(ecg_queue.count() >= 14){ // check if there are at least 14 ECG measurements to print

        // remove 14 ECG measurements from the queue, package them, and send them to the phone
        zero = ecg_queue.dequeue();
        one = ecg_queue.dequeue();
        two = ecg_queue.dequeue();
        three = ecg_queue.dequeue();
        four = ecg_queue.dequeue();
        five = ecg_queue.dequeue();
        six = ecg_queue.dequeue();
        seven = ecg_queue.dequeue();
        eight = ecg_queue.dequeue();
        nine = ecg_queue.dequeue();
        ten = ecg_queue.dequeue();
        eleven = ecg_queue.dequeue();
        twelve = ecg_queue.dequeue();
        thirteen = ecg_queue.dequeue();
        unsigned char ecgCharArray[14] = { zero, one, two, three, four, five, six, seven, 
          eight, nine, ten, eleven, twelve, thirteen };
        ecgChar.setValue(ecgCharArray, 14);
      }
      
    } else { // if we disconnect from the phone, we stop trying to send things
             // to it and turn off the LED
      ble_connected = false;
      digitalWrite(LED_PIN, LOW);
      Serial.print("Disconnected from central: ");
      Serial.println(central.address());
    }
    
  } else { // If we haven't connected to the phone yet, we attempt to do so
      central = blePeripheral.central();
      
      if(central){ // when we've successfully connected to the phone
        ble_connected = true;
        Serial.print("Connected to central: ");
        // print the central's MAC address:
        Serial.println(central.address());
        // turn on the LED to indicate the connection:
        digitalWrite(LED_PIN, HIGH);
      }
  }
}

void updateHeartRate(){ // interrupt handler
 
    boolean QRS_detected = false; // keeps track of whether it's time to update the BPM
    
    // only read data if ECG chip has detected that leads are attached to patient
    boolean leads_are_on = (digitalRead(LEADS_OFF_PLUS_PIN) == 0) && (digitalRead(LEADS_OFF_MINUS_PIN) == 0);
    if(leads_are_on && ble_connected){     
           
      // read next ECG data point
      int next_ecg_pt = analogRead(ECG_PIN);

      ecg_q_count++;
      if(ecg_q_count > 1){ // we only need to send every nth ECG value to the phone
                           // a lower value in this loop means a higher resolution
                           // for the graph on the phone
        
        int ecg = map(next_ecg_pt, 0, 1023, 0, 100); // scale each measurement to fit on the graph
        ecg_queue.enqueue(ecg); // add each measurement to the queue
        ecg_q_count = 0;
      }
      
      // give next data point to algorithm
      QRS_detected = detect(next_ecg_pt);
            
      if(QRS_detected == true){
        
        foundTimeMicros = micros();

        // use the distance in time between when the last two peaks were detected to calculate BPM
        
        bpm_buff[bpm_buff_WR_idx] = (60.0 / (((float) (foundTimeMicros - old_foundTimeMicros)) / 1000000.0));
        bpm_buff_WR_idx++;
        bpm_buff_WR_idx %= BPM_BUFFER_SIZE;
        bpm += bpm_buff[bpm_buff_RD_idx];
    
        tmp = bpm_buff_RD_idx - BPM_BUFFER_SIZE + 1;
        if(tmp < 0) tmp += BPM_BUFFER_SIZE;

        bpm_queue.enqueue(bpm/BPM_BUFFER_SIZE); // sends the current average BPM to the queue to be printed
    
        bpm -= bpm_buff[tmp];
        
        bpm_buff_RD_idx++;
        bpm_buff_RD_idx %= BPM_BUFFER_SIZE;

        old_foundTimeMicros = foundTimeMicros;

      }
    }
}


/* Portion pertaining to Pan-Tompkins QRS detection */

// circular buffer for input ecg signal
// we need to keep a history of M + 1 samples for HP filter
float ecg_buff[M + 1] = {0};
int ecg_buff_WR_idx = 0;
int ecg_buff_RD_idx = 0;

// circular buffer for input ecg signal
// we need to keep a history of N+1 samples for LP filter
float hp_buff[N + 1] = {0};
int hp_buff_WR_idx = 0;
int hp_buff_RD_idx = 0;

// LP filter outputs a single point for every input point
// This goes straight to adaptive filtering for eval
float next_eval_pt = 0;

// running sums for HP and LP filters, values shifted in FILO
float hp_sum = 0;
float lp_sum = 0;

// working variables for adaptive thresholding
float treshold = 0;
boolean triggered = false;
int trig_time = 0;
float win_max = 0;
int win_idx = 0;

// number of starting iterations, used determine when moving windows are filled
int number_iter = 0;

boolean detect(float new_ecg_pt) {
  
  // copy new point into circular buffer, increment index
  ecg_buff[ecg_buff_WR_idx++] = new_ecg_pt;  
  ecg_buff_WR_idx %= (M+1);

 
  /* High pass filtering */
  
  if(number_iter < M){
    // first fill buffer with enough points for HP filter
    hp_sum += ecg_buff[ecg_buff_RD_idx];
    hp_buff[hp_buff_WR_idx] = 0;
    
  } else {
    hp_sum += ecg_buff[ecg_buff_RD_idx];
    
    tmp = ecg_buff_RD_idx - M;
    if(tmp < 0) tmp += M + 1;
    
    hp_sum -= ecg_buff[tmp];
    
    float y1 = 0;
    float y2 = 0;
    
    tmp = (ecg_buff_RD_idx - ((M+1)/2));
    if(tmp < 0) tmp += M + 1;
    
    y2 = ecg_buff[tmp];
    
    y1 = HP_CONSTANT * hp_sum;
    
    hp_buff[hp_buff_WR_idx] = y2 - y1;
  }
  
  // done reading ECG buffer, increment position
  ecg_buff_RD_idx++;
  ecg_buff_RD_idx %= (M+1);
  
  // done writing to HP buffer, increment position
  hp_buff_WR_idx++;
  hp_buff_WR_idx %= (N+1);
  

  /* Low pass filtering */
  
  // shift in new sample from high pass filter
  lp_sum += hp_buff[hp_buff_RD_idx] * hp_buff[hp_buff_RD_idx];
  
  if(number_iter < N){
    // first fill buffer with enough points for LP filter
    next_eval_pt = 0;
    
  } else {
    // shift out oldest data point
    tmp = hp_buff_RD_idx - N;
    if(tmp < 0) tmp += (N+1);
    
    lp_sum -= hp_buff[tmp] * hp_buff[tmp];
    
    next_eval_pt = lp_sum;
  }
  
  // done reading HP buffer, increment position
  hp_buff_RD_idx++;
  hp_buff_RD_idx %= (N+1);
  

  /* Adapative thresholding beat detection */
  // set initial threshold        
  if(number_iter < winSize) {
    if(next_eval_pt > treshold) {
      treshold = next_eval_pt;
    }
    // only increment number_iter iff it is less than winSize
    // if it is bigger, then the counter serves no further purpose
    number_iter++;
  }
  
  // check if detection hold off period has passed
  if(triggered == true){
    trig_time++;
    
    if(trig_time >= 100){
      triggered = false;
      trig_time = 0;
    }
  }
  
  // find if we have a new max
  if(next_eval_pt > win_max) win_max = next_eval_pt;
  
  // find if we are above adaptive threshold
  if(next_eval_pt > treshold && !triggered) {
    triggered = true;

    return true;
  }
  // else we'll finish the function before returning FALSE,
  // to potentially change threshold
          
  // adjust adaptive threshold using max of signal found 
  // in previous window            
  if(win_idx++ >= winSize){
    
    // weighting factor for determining the contribution of
    // the current peak value to the threshold adjustment
    float gamma = 0.175;
    
    // forgetting factor - rate at which we forget old observations
    // choose a random value between 0.01 and 0.1 for this, 
    float alpha = 0.01 + ( ((float) random(0, RAND_RES) / (float) (RAND_RES)) * ((0.1 - 0.01)));
    
    // compute new threshold
    treshold = alpha * gamma * win_max + (1 - alpha) * treshold;
    
    // reset current window index
    win_idx = 0;
    win_max = -10000000;
  }
      
  // return false if we didn't detect a new QRS
  return false;
    
}
