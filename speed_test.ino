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

BLEPeripheral blePeripheral; 

BLEService myService("aa7b3c40-f6ed-4ffc-bc29-5750c59e74b3"); 

BLECharacteristic bpmChar("b0351694-25e6-4eb5-918c-ca9403ddac47",  
    BLERead | BLENotify, 5);  

BLECentral central = blePeripheral.central(); // the Blutooth central device (the phone)

boolean ble_connected = false;

int counter;
unsigned long timeStamp; 

void setup() { // called when the program starts

  Serial.begin(115200); // set up the serial monitor
  while(!Serial);     // wait for the serial monitor

  setUpBLE(); // sets up the bluetooth services and characteristics 

  counter = 0; 
  timeStamp = 12345;
  
}

void setUpBLE() {
  
    /* Set a local name for the BLE device */
    blePeripheral.setLocalName("Penelope");
    blePeripheral.setAdvertisedServiceUuid(myService.uuid());  // add the service UUID
    blePeripheral.addAttribute(myService);// add the BLE service
    blePeripheral.addAttribute(bpmChar);  // add the BPM characteristic

      /* Now activate the BLE device.  It will start continuously transmitting BLE
       advertising packets and will be visible to remote BLE central devices
       until it receives a new connection */
    blePeripheral.begin();

    Serial.println("Bluetooth device active, waiting for connections...");
  
}

void loop(){

  if(ble_connected){ 
    
    if(central.connected()){ // check if we're still connected to the phone

      unsigned char ts0 = timeStamp & 0xff;      // get each byte from the time stamp separately
      unsigned char ts1 = (timeStamp >> 8) & 0xff;  // so that the time stamp can be sent in a
      unsigned char ts2 = (timeStamp >> 16) & 0xff;    // bluetooth compatible format
      unsigned char ts3 = (timeStamp >> 24) & 0xff;
      
      unsigned char bpmCharArray[5] = { ts0, ts1, ts2, ts3, (unsigned char) counter };
      bpmChar.setValue(bpmCharArray, 5); // send them to the phone

      if (counter < 300){
        Serial.print(counter);
        Serial.print(", ");
        Serial.println(timeStamp);
      }
  
      counter++;
      timeStamp += 10;
      
    } else { // if we disconnect from the phone, we stop trying to send things
             
      ble_connected = false;
      
      Serial.print("Disconnected from central: ");
      Serial.println(central.address());
    }
    
  } else { // If we haven't connected to the phone yet, we attempt to do so
      central = blePeripheral.central();
      
      if(central){ // when we've successfully connected to the phone
        
        Serial.print("Connected to central: ");
        // print the central's MAC address:
        Serial.println(central.address());

        delay(1000);
        ble_connected = true; 
        counter = 0;
        timeStamp = 12345;
      }
  } 
}


