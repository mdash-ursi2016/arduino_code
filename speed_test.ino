// This program sends the numbers 0 through 250 to a phone
// as fast as possible. It repeats this action every time
// it reconnects to the phone after a disconnect.

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

BLEPeripheral blePeripheral; // the Bluetooth peripheral device (the Arduino)

BLEService myService("aa7b3c40-f6ed-4ffc-bc29-5750c59e74b3"); // custom service

BLECharacteristic bpmChar("b0351694-25e6-4eb5-918c-ca9403ddac47", // custom characteristic
    BLERead | BLENotify, 1);  

BLECentral central = blePeripheral.central(); // the Blutooth central device (the phone)

boolean ble_connected = false; // keeps track of whether Bluetooth is connected

int counter; // the data we're sending

void setup() { // called when the program starts

  Serial.begin(115200); // set up the serial monitor
  while(!Serial);     // wait for the serial monitor

  setUpBLE(); // sets up the bluetooth services and characteristics 

  counter = 0; // reset data to 0
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

      if (counter <= 250){ // send the numbers 0-250 to the phone
        
        unsigned char bpmCharArray[1] = {(unsigned char) counter };
        bpmChar.setValue(bpmCharArray, 1); // send to phone

        Serial.println(counter);
       
        counter++; // increment data variable
      }
      
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

        delay(1000); // wait a bit before sending packets so they 
                     //   don't get dropped on the floor
        ble_connected = true; 
        counter = 0; // reset data variable
      }
  } 
}


