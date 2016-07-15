/*  This program obtains live ECG readings from a sensor, uses the Pan-Tompkins 
 *  QRS-detection algorithm to calculate the corresponding Beats Per Minute (BPM), 
 *  and stores the BPMs and their corresponding time stamps on the Arduino 101's 
 *  2MB external flash memory chip. When connected to the phone, it retrieves the
 *  BPMs and time stamps from the chip and sends them to the phone live via a 
 *  custom Bluetooth Low Energy service. It also sends live ECG measurements to the
 *  phone for a graph of the heart beat. When the phone is disconnected for a while
 *  and then reconnects, the device quickly sends all the accumulated BPM + time 
 *  stamp data to the phone and then resumes live updating.
 *  */

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
#include <CurieTime.h>
#include <SerialFlash.h>

#define usb  // COMMENT THIS OUT IF CONNECTED TO BATTERY INSTEAD OF COMPUTER
// #define nate // COMMENT THIS OUT IF USING AN NRF APP INSTEAD OF NATE'S APP

/* The portions of this code that implement the Pan-Tompkins QRS-detection algorithm were 
 *  modified from code taken from Blake Milner's real_time_QRS_detection GitHub repository:
 https://github.com/blakeMilner/real_time_QRS_detection/blob/master/QRS_arduino/QRS.ino */

unsigned long BPMcounter = 0; 

#define ECG_PIN A0                 // the number of the ECG pin (analog)
#define LED_PIN 13                 // indicates whether Bluetooth is connected

// pins for leads off detection
const int LEADS_OFF_PLUS_PIN  = 10;      // the number of the LO+ pin (digital)
const int LEADS_OFF_MINUS_PIN = 11;      // the number of the LO- pin (digital) 

const int frequency = 5000; // rate at which the heart rate is checked
                            // (in microseconds): works out to 200 hz

QueueArray<int> bpm_queue; // holds the BPMs to be printed

QueueArray<int> ecg_queue; // holds the ECGs to be printed

int ecg_q_count = 0; // used to space out the ECG measurements sent to

boolean timeInitiated = false; // whether time has ever been set by phone

boolean safeToFill = true; // used to prevent the ECG measurement queue
                           // from being added to too early after the 
                           // phone has recently reconnected

// Memory management stuff ------------------------

#define FSIZE 128 // the size of a file on the flash chip

#define NUM_BUFFS 5 // ***changing this changes the number of files in memory***

#define DSIZE 8 // size of each unit of data stored in memory (BPM + time stamp)

#define DATA_PER_FILE (FSIZE / DSIZE) // how many data can be stored per file

#define STORAGE_LENGTH (DATA_PER_FILE * NUM_BUFFS) // how many data can be stored total

SerialFlashFile flashFiles[NUM_BUFFS]; // holds all the files 

short readFileIndex, writeFileIndex; // keeps track of which read and write file we're on

unsigned long nextToPlace;    // next location on the chip to write BPMs
unsigned long nextToRetrieve; // next location on the chip to read BPMs
                              // from and send them to the phone

unsigned long toMemBuff[2];   // holds a BPM and time stamp on its way to the chip
unsigned long fromMemBuff[2]; // holds a BPM and time stamp on its way back
                              // the chip to be sent to the phone

const int FlashChipSelect = 21; // digital pin for flash chip CS pin

// ------------------------------------------------


// Bluetooth stuff --------------------------------

#define BYTES_PER_PCKG 5
#define NUM_PCKGS 4
#define BATCH_SIZE (NUM_PCKGS * BYTES_PER_PCKG)
#define NUM_ECGS 14

boolean ble_connected = false;  // keeps track of whether the Bluetooth is connected

BLEPeripheral blePeripheral; // BLE Peripheral Device (the board you're programming)
BLEService myService("aa7b3c40-f6ed-4ffc-bc29-5750c59e74b3"); // BLE Heart Rate Service

//// Custom BLE characteristic to hold the initial time
BLECharacteristic timeChar("95d344f4-c6ad-48d8-8877-661ab4d41e5b",  
    BLERead | BLEWrite, 8);  

// Custom BLE characteristic to hold BPM
BLECharacteristic bpmChar("b0351694-25e6-4eb5-918c-ca9403ddac47",  
    BLERead | BLENotify, BYTES_PER_PCKG);  

// Custom BLE characteristic to batches of BPMs
BLECharacteristic batchChar("3cd43730-fc61-4ea7-aa18-6e7c3d798d74",  
    BLERead | BLENotify, BATCH_SIZE); 

// Custom BLE characteristic to hold chunks of raw ECG measurements
BLECharacteristic ecgChar("1bf9168b-cae4-4143-a228-dc7850a37d98", 
    BLERead | BLENotify, NUM_ECGS); 

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

  #ifdef usb  
  Serial.begin(115200); // set up the serial monitor
  while(!Serial);     // wait for the serial monitor
  ecg_queue.setPrinter(Serial); // allows the ecg queue to print error messages
  #endif

  setUpFlash(); // sets up the flash memory chip and creates files to store BPMs

  setUpBLE(); // sets up the bluetooth services and characteristics 

  pinMode(LED_PIN, OUTPUT);

  pinMode(LEADS_OFF_PLUS_PIN, INPUT); // Setup for leads off detection LO +
  pinMode(LEADS_OFF_MINUS_PIN, INPUT); // Setup for leads off detection LO -

  CurieTimerOne.start(frequency, &updateHeartRate);  // set timer and callback
  
}

void setUpFlash() { // sets up the flash chip for memory management
  
   if (!SerialFlash.begin(FlashChipSelect)) { // make sure we successfully connect to
    #ifdef usb                                // the flash chip
    Serial.println("Unable to access SPI Flash chip");
    #endif
  }

  #ifdef usb
  Serial.println("Erasing flash chip...");
  #endif
  
  SerialFlash.eraseAll(); // erase the flash chip to start with a clean slate
  while(!SerialFlash.ready()); // don't do anything till the flash chip is erased

  #ifdef usb
  Serial.println("Flash chip erased");
  #endif

  // create NUM_BUFFS number of files and open them
  int i;
  for (i = 0; i < NUM_BUFFS; i++){
  
    String fname = "file";
    fname = fname + String(i);  // creating the file name
    char filename[6];
    fname.toCharArray(filename, 6);

    if(!create_if_not_exists(filename)){ // creating the file
      #ifdef usb
      Serial.println("Not enough space to create files");
      #endif
      return;
    }
    flashFiles[i] = SerialFlash.open(filename); // opening the file
  }
  
  writeFileIndex = 0; // we need to keep track of which files we're currently
  readFileIndex = 0; // reading and writing on

  nextToPlace = 0; // begin at the beginning of 
  nextToRetrieve = 0; // the first file 
  
}

// creates a file if a file with that same name doesn't
// exist already
boolean create_if_not_exists (const char *filename) {
  if (!SerialFlash.exists(filename)) {
    #ifdef usb
    Serial.println("Creating file " + String(filename));
    #endif
    return SerialFlash.createErasable(filename, FSIZE);
  }
  #ifdef usb
  Serial.println("File " + String(filename) + " already exists");
  #endif
  return true;
}

void setUpBLE() {
  
    /* Set a local name for the BLE device */
    blePeripheral.setLocalName("Penelope");
    blePeripheral.setAdvertisedServiceUuid(myService.uuid());  // add the service UUID
    blePeripheral.addAttribute(myService);// add the BLE service
    blePeripheral.addAttribute(timeChar); // add the time characteristic
    blePeripheral.addAttribute(bpmChar);  // add the BPM characteristic
    blePeripheral.addAttribute(batchChar);// add the BPM batch characteristic
    blePeripheral.addAttribute(ecgChar);  // add the ECG characteristic
  
      /* Now activate the BLE device.  It will start continuously transmitting BLE
       advertising packets and will be visible to remote BLE central devices
       until it receives a new connection */
    blePeripheral.begin();

    blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
    blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  
    #ifdef usb
    Serial.println("Bluetooth device active, waiting for connections...");
    #endif
  
}

void blePeripheralConnectHandler(BLECentral& central) {
  
  #ifdef usb
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
  #endif

  #ifdef nate
  obtainInitTime();
  #endif

  // until this has been set to true for the first time, no BPMs
  // can be recorded because they're time stamps would be in 1970S
  timeInitiated = true;
  
  digitalWrite(LED_PIN, HIGH); // turn on the connection light

  // wait a bit before sending data to the phone because otherwise
  // the initial batch of bluetooth packets you send will get dropped
  // on the floor
  delay(1000);

  ble_connected = true;
}

void blePeripheralDisconnectHandler(BLECentral& central) {

  safeToFill = false;
  ble_connected = false;
  
  #ifdef usb
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
  #endif
      
  digitalWrite(LED_PIN, LOW);
}

void loop() { // called continuously

  blePeripheral.poll();

  sendECG();

  if(!bpm_queue.isEmpty()) { // check if there's the BPM to print

    // remove a BPM from the queue and send it to memory
    unsigned long heartRate = (unsigned long) bpm_queue.dequeue();
    unsigned long timeStamp = now(); // get the current epoch time in seconds
    
    toMemBuff[0] = heartRate; // put the BPM and time stamp into the buffer
    toMemBuff[1] = timeStamp; //   before placing the buffer's contents in memory
    placeInMemory();
     
  }

  if(ble_connected) { 
    
    if (caughtUp()){
      liveSend();
      
    } else {   
      batchSend(); 
    }
  } 
}

void obtainInitTime() {
  // don't do anything until the phone gives you the current
  // time in epoch time
  while(!timeChar.written()){
    delay(1);
  }

  // get the time from the phone as a byte array and 
  // combine each byte into one 4-byte number
  const unsigned char *fromPhone;
  fromPhone = timeChar.value(); 
  unsigned long ts0 = ((unsigned long) fromPhone[0]);
  unsigned long ts1 = ((unsigned long) fromPhone[1]) << 8;
  unsigned long ts2 = ((unsigned long) fromPhone[2]) << 16;
  unsigned long ts3 = ((unsigned long) fromPhone[3]) << 24;
  unsigned long initTime = ts0 | ts1 | ts2 | ts3;

  setTime(initTime); // set the time using the CurieTime library

  #ifdef usb
  Serial.println("time has been set");
  #endif
}

void liveSend() {
  if (retrieveFromMemory()) { // only update the characteristic if new data was read
    unsigned long timeStamp = fromMemBuff[1]; // get the time stamp
    unsigned char ts0 = timeStamp & 0xff;      // get each byte from the time stamp separately
    unsigned char ts1 = (timeStamp >> 8) & 0xff;  // so that the time stamp can be sent in a
    unsigned char ts2 = (timeStamp >> 16) & 0xff;    // bluetooth compatible format
    unsigned char ts3 = (timeStamp >> 24) & 0xff;
  
    #ifdef usb
    Serial.print(fromMemBuff[0]);
    Serial.print(", ");
    Serial.println(timeStamp);
    #endif
    
    // package the the BPM and time stamp, switching to big-endian
    unsigned char bpmCharArray[BYTES_PER_PCKG] = { ts0, ts1, ts2, ts3, (unsigned char) fromMemBuff[0] };
    bpmChar.setValue(bpmCharArray, BYTES_PER_PCKG); // send them to the phone
  } 
}

void batchSend() {
  unsigned char batchCharArray[BATCH_SIZE];
  
  short i;
  for (i = 0; i < NUM_PCKGS; i++) {
    if (retrieveFromMemory()) {
      unsigned long timeStamp = fromMemBuff[1]; // get the time stamp
      unsigned char ts0 = timeStamp & 0xff;      // get each byte from the time stamp separately
      unsigned char ts1 = (timeStamp >> 8) & 0xff;  // so that the time stamp can be sent in a
      unsigned char ts2 = (timeStamp >> 16) & 0xff;    // bluetooth compatible format
      unsigned char ts3 = (timeStamp >> 24) & 0xff;

      short offset = i * BYTES_PER_PCKG;

      #ifdef usb
      Serial.print(fromMemBuff[0]);
      Serial.print(", ");
      Serial.print(timeStamp);
      Serial.print('\t');
      #endif
      
      batchCharArray[offset] = ts0;
      batchCharArray[offset+1] = ts1;
      batchCharArray[offset+2] = ts2;
      batchCharArray[offset+3] = ts3;
      batchCharArray[offset+4] = (unsigned char) fromMemBuff[0];

    } else {
      #ifdef usb
      Serial.println("you have made an error");
      #endif
    }
  }

  batchChar.setValue(batchCharArray, BATCH_SIZE);
  Serial.println();

}

void sendECG() {
  if (ecg_queue.count() >= NUM_ECGS) { // check if there are at least 14 ECG measurements to print
      
     // remove 14 ECG measurements from the queue and package them
     unsigned char ecgCharArray[NUM_ECGS];
     int i;
     for (i = 0; i < NUM_ECGS; i++){
       ecgCharArray[i] = ecg_queue.dequeue();
     }
     
     safeToFill = true; // allow ECG measurements to be added to the queue if
                         // they weren't allowed already    

     ecgChar.setValue(ecgCharArray, NUM_ECGS); // send the array of 14 ECG measurements to
                                          // the phone to be graphed
      
     } else {    
      safeToFill = true; // allow ECG measurements to be added to the queue if
                         // they weren't allowed already
     }
}

void placeInMemory() { // store BPMs and timestamps on the flash chip

  flashFiles[writeFileIndex].seek(nextToPlace); // move cursor
  flashFiles[writeFileIndex].write(toMemBuff, DSIZE); // write to chip
  nextToPlace += DSIZE; // increment the offset to be written to next

  // check if it's time to switch files
  if(nextToPlace >= FSIZE){
    switchWriteFiles();
  }
}

boolean retrieveFromMemory() { // retrieve BPMs and time stamps from the flash
                               //   chip to send them to the phone
                               
  if((readFileIndex == writeFileIndex) && (nextToRetrieve >= nextToPlace)){
    return false; // if the read pointer has caught up to the write pointer,
                  // don't try to read any new data yet
  }
  flashFiles[readFileIndex].seek(nextToRetrieve); // move cursor
  flashFiles[readFileIndex].read(fromMemBuff, DSIZE); // read from chip
  nextToRetrieve += DSIZE; // increment the offset to be read from next

  // check if it's time to switch files
  if(nextToRetrieve >= FSIZE){
    switchReadFiles();
  }
  return true;
}

// when one file in memory gets full, we erase the
//  next file and start writing to it
void switchWriteFiles() {

  // change the current file we're writing to
  writeFileIndex = (writeFileIndex + 1) % NUM_BUFFS;
  // erase it so we can write to it
  flashFiles[writeFileIndex].erase();
  // start writing to the beginning of this file
  nextToPlace = 0;

  #ifdef usb
  Serial.print("Switching WRITE file to ");
  Serial.println(writeFileIndex);
  #endif

  // if the file that's just been erased was the file
  // that the read pointer is currently on, evict the
  // read pointer to the next file
  if(writeFileIndex == readFileIndex){
    switchReadFiles();
  }
}

// when we've read everthing in one file, we 
//  start reading from the next one
void switchReadFiles() {

  // change the current file we're reading from
  readFileIndex = (readFileIndex + 1) % NUM_BUFFS;
  // start reading from the beginning of the this file
  nextToRetrieve = 0;

  #ifdef usb
  Serial.print("Switching READ file to ");
  Serial.println(readFileIndex);
  #endif
}

boolean caughtUp() {

  // calculate logical read and write indices 
  int logicalWriteIndex = (writeFileIndex * DATA_PER_FILE) + (nextToPlace / DSIZE);
  int logicalReadIndex = (readFileIndex * DATA_PER_FILE) + (nextToRetrieve / DSIZE);

  // figure out how many data units are between the write and read pointer
  int distance = ((logicalWriteIndex - logicalReadIndex) + STORAGE_LENGTH) % STORAGE_LENGTH;

  return (distance < NUM_PCKGS);
  
}

void updateHeartRate() { // interrupt handler
 
    boolean QRS_detected = false; // keeps track of whether it's time to update the BPM
    
    // only read data if ECG chip has detected that leads are attached to patient
    boolean leads_are_on = (digitalRead(LEADS_OFF_PLUS_PIN) == 0) && (digitalRead(LEADS_OFF_MINUS_PIN) == 0);
    if(leads_are_on){     
           
      // read next ECG data point
      int next_ecg_pt = analogRead(ECG_PIN);
      
      if (ble_connected && safeToFill){
        ecg_q_count++;
        if(ecg_q_count > 2 && ecg_queue.count() < 70){ // we only need to send every nth ECG value to the phone
                           // a lower value in this loop means a higher resolution
                           // for the graph on the phone
        
          int ecg = map(next_ecg_pt, 0, 1023, 0, 100); // scale each measurement to fit on the graph
          ecg_queue.enqueue(ecg); // add each measurement to the queue
          ecg_q_count = 0;
        }
      } else {
        safeToFill = false;
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

        if (timeInitiated){
          // bpm_queue.enqueue(bpm/BPM_BUFFER_SIZE); // sends the current average BPM to the queue to be printed
          bpm_queue.enqueue(BPMcounter++); 
        }
    
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
