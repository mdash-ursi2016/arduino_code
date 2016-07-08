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

// #define usb // COMMENT THIS OUT IF CONNECTED TO BATTERY INSTEAD OF COMPUTER

/* The portions of this code that implement the Pan-Tompkins QRS-detection algorithm were 
 *  modified from code taken from Blake Milner's real_time_QRS_detection GitHub repository:
 https://github.com/blakeMilner/real_time_QRS_detection/blob/master/QRS_arduino/QRS.ino */

#define LED_PIN 13                  // the number of the LED pin (digital)
#define ECG_PIN A0                  // the number of the ECG pin (analog)

// pins for leads off detection
const int LEADS_OFF_PLUS_PIN  = 10;      // the number of the LO+ pin (digital)
const int LEADS_OFF_MINUS_PIN = 11;      // the number of the LO- pin (digital) 

const int frequency = 1000000; // rate at which the heart rate is checked
                            // (in microseconds): works out to 200 hz

QueueArray<int> bpm_queue; // holds the BPMs to be printed

QueueArray<int> ecg_queue; // holds the ECGs to be printed

int BPMcounter = 0; 
int ECGcounter = 0; // used to space out the ECG measurements sent to

boolean safeToFill = true; // used to prevent the ECG measurement queue
                           // from being added to too early after the 
                           // phone has recently reconnected

// Memory management stuff ------------------------

#define FSIZE 2048 // size of a file in memory
#define DSIZE 8   // size of each unit of data stored in memory (BPM + time stamp)

const char *filenameA = "fileA.txt"; 
const char *filenameB = "fileB.txt";

SerialFlashFile fileA, fileB, currentFile;

boolean onFileA;              // keeps track of which file we're on

unsigned long startOfA;       // address of the start of fileA
unsigned long startOfB;       // address of the start of fileB

unsigned long nextToPlace;    // next location on the chip to write BPMs
unsigned long nextToRetrieve; // next location on the chip to read BPMs
                              // from and send them to the phone

unsigned long toMemBuff[2];   // holds a BPM and time stamp on its way to the chip
unsigned long fromMemBuff[2]; // holds a BPM and time stamp on its way back from
                              // the chip to be sent to the phone

const int FlashChipSelect = 21; // digital pin for flash chip CS pin

// ------------------------------------------------


// Bluetooth stuff --------------------------------

boolean ble_connected = false;  // keeps track of whether the Bluetooth is connected

BLEPeripheral blePeripheral; // BLE Peripheral Device (the board you're programming)
BLEService myService("aa7b3c40-f6ed-4ffc-bc29-5750c59e74b3"); // BLE Heart Rate Service

//// Custom BLE characteristic to hold the initial time
BLECharacteristic timeChar("95d344f4-c6ad-48d8-8877-661ab4d41e5b",  
    BLERead | BLEWrite, 8);  

// Custom BLE characteristic to hold BPM
BLECharacteristic bpmChar("b0351694-25e6-4eb5-918c-ca9403ddac47",  
    BLERead | BLENotify, 8);  

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

  SerialFlash.eraseAll();
  while(!SerialFlash.ready());

  // create file A and B if they don't already exist
  if ((!create_if_not_exists(filenameA)) || (!create_if_not_exists(filenameB))) {
    #ifdef usb
    Serial.println("Not enough space to create files");
    #endif
    return;
  }

  fileA = SerialFlash.open(filenameA); // open both files and assign
  fileB = SerialFlash.open(filenameB); // them to global variables

  onFileA = true; // we need to keep track of which file we're
  currentFile = fileA; // currently writing to 

  nextToPlace = currentFile.position(); // begin at the beginning of 
  nextToRetrieve = currentFile.position(); // the first file 
  
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
    blePeripheral.setLocalName("Nora");
    blePeripheral.setAdvertisedServiceUuid(myService.uuid());  // add the service UUID
    blePeripheral.addAttribute(myService);   // add the BLE service
    blePeripheral.addAttribute(timeChar); // add the time characteristic
    blePeripheral.addAttribute(bpmChar); // add the BPM characteristic
    blePeripheral.addAttribute(ecgChar); // add the ECG characteristic
  
      /* Now activate the BLE device.  It will start continuously transmitting BLE
       advertising packets and will be visible to remote BLE central devices
       until it receives a new connection */
    blePeripheral.begin();
  
    #ifdef usb
    Serial.println("Bluetooth device active, waiting for connections...");
    #endif
  
}

void loop() { // called continuously

  if(!bpm_queue.isEmpty()){ // check if there's the BPM to print

    // remove a BPM from the queue and send it to memory
    unsigned long heartRate = (unsigned long) bpm_queue.dequeue();
    unsigned long timeStamp = now();
    
    toMemBuff[0] = heartRate; // put the BPM and time stamp into the buffer
    toMemBuff[1] = timeStamp; //   before placing the buffer's contents in memory
    placeInMemory();
     
  }

  if(ble_connected){ 
    
    if(central.connected()){ // check if we're still connected to the phone

      if (retrieveFromMemory()){ // only update the characteristic if new data was read
        
        unsigned long timeStamp =  fromMemBuff[1]; // get the time stamp
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
        unsigned char bpmCharArray[8] = { (unsigned char) fromMemBuff[0], 0, 0, 0, ts0, ts1, ts2, ts3 };
        bpmChar.setValue(bpmCharArray, 8); // send them to the phone
      }

      if(ecg_queue.count() >= 14){ // check if there are at least 14 ECG measurements to print
        
        // remove 14 ECG measurements from the queue and package them
        unsigned char ecgCharArray[14];
        int i;
        for (i = 0; i < 14; i++){
          ecgCharArray[i] = ecg_queue.dequeue();
        }
        safeToFill = true; // allow ECG measurements to be added to the queue if
                           // they weren't allowed already    

        ecgChar.setValue(ecgCharArray, 14); // send the array of 14 ECG measurements to
                                            // the phone to be graphed
        
      } else {    
        safeToFill = true; // allow ECG measurements to be added to the queue if
                           // they weren't allowed already    
      }
      
    } else { // if we disconnect from the phone, we stop trying to send things
             //   to it and turn off the LED
      safeToFill = false;
      ble_connected = false;
      #ifdef usb
      Serial.print("Disconnected from central: ");
      Serial.println(central.address());
      #endif
      digitalWrite(LED_PIN, LOW);
    }
    
  } else { // If we haven't connected to the phone yet, we attempt to do so
      central = blePeripheral.central();
      
      if(central){ // when we've successfully connected to the phone

        #ifdef usb
        Serial.print("Connected to central: ");
        // print the central's MAC address:
        Serial.println(central.address());
        #endif

        digitalWrite(LED_PIN, HIGH);
        
        while(!timeChar.written()){
          delay(1);
        }
                
        const unsigned char *fromPhone;
        fromPhone = timeChar.value();
        unsigned long ts0 = ((unsigned long) fromPhone[0]);
        unsigned long ts1 = ((unsigned long) fromPhone[1]) << 8;
        unsigned long ts2 = ((unsigned long) fromPhone[2]) << 16;
        unsigned long ts3 = ((unsigned long) fromPhone[3]) << 24;
        unsigned long initTime = ts0 | ts1 | ts2 | ts3;

        setTime(initTime);

        #ifdef usb
        Serial.println("time has been set");
        #endif
        
        delay(1000);
        ble_connected = true; // recently moved from top to bottom
      }
  }
}

void placeInMemory() { // store BPMs and timestamps on the flash chip
  
  currentFile.seek(nextToPlace);
  currentFile.write(toMemBuff, DSIZE);
  nextToPlace += DSIZE;

  // check if it's time to switch files
  if(onFileA){
    if(nextToPlace + DSIZE >= startOfA + FSIZE){
      switchFiles();
    }
  } else {
    if(nextToPlace + DSIZE >= startOfB + FSIZE){
      switchFiles();
    }
  }
}

boolean retrieveFromMemory() { // retrieve BPMs and time stamps from the flash
                               //   chip to send them to the phone
  if(nextToRetrieve == nextToPlace){
    return false; // if we're caught up, don't try to read any data
  }
  currentFile.seek(nextToRetrieve);
  currentFile.read(fromMemBuff, DSIZE);
  nextToRetrieve += DSIZE;
  return true;
}

// when one file in memory gets full, we erase it
//  and start writing to the next file
void switchFiles() {

  #ifdef usb
  Serial.println("switching files");
  #endif
  
  // if we're currently on fileA, switch to fileB
  if(onFileA){
    nextToPlace = startOfB;
    nextToRetrieve = startOfB;
    currentFile = fileB;
    onFileA = false;
    fileA.erase();
    
  // if we're currently on fileB, switch to fileA  
  } else {
    nextToPlace = startOfA;
    nextToRetrieve = startOfA;
    currentFile = fileA;
    onFileA = true;
    fileB.erase();
  }
  
}

void updateHeartRate(){ // interrupt handler
  /* Read the current voltage level on the A0 analog input pin.
     This is used here to simulate the heart rate's measurement.*/

      if (ble_connected && safeToFill){
          ecg_queue.enqueue(ECGcounter++); // add each measurement to the queue
          ecg_queue.enqueue(ECGcounter++); // add each measurement to the queue
          ecg_queue.enqueue(ECGcounter++); // add each measurement to the queue
          ecg_queue.enqueue(ECGcounter++); // add each measurement to the queue
          ecg_queue.enqueue(ECGcounter++); // add each measurement to the queue
          ecg_queue.enqueue(ECGcounter++); // add each measurement to the queue
          ecg_queue.enqueue(ECGcounter++); // add each measurement to the queue
          ecg_queue.enqueue(ECGcounter++); // add each measurement to the queue
          ecg_queue.enqueue(ECGcounter++); // add each measurement to the queue
          ecg_queue.enqueue(ECGcounter++); // add each measurement to the queue
          ecg_queue.enqueue(ECGcounter++); // add each measurement to the queue
          ecg_queue.enqueue(ECGcounter++); // add each measurement to the queue
          ecg_queue.enqueue(ECGcounter++); // add each measurement to the queue
          ecg_queue.enqueue(ECGcounter++); // add each measurement to the queue       
      }

      bpm_queue.enqueue(BPMcounter++);

      if(BPMcounter > 254){
        BPMcounter = 0;
      }

      if(ECGcounter > 100){
        ECGcounter = 0; 
      }
}
