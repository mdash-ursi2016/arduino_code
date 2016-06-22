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

/*
   Copyright (c) 2015 Intel Corporation.  All rights reserved.

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*
   This sketch example partially implements the standard Bluetooth Low-Energy Heart Rate service.
   For more information: https://developer.bluetooth.org/gatt/services/Pages/ServicesHome.aspx
*/

BLEPeripheral blePeripheral;       // BLE Peripheral Device (the board you're programming)
BLEService heartRateService("180D"); // BLE Heart Rate Service

// BLE Heart Rate Measurement Characteristic"
BLECharacteristic heartRateChar("2A37",  // standard 16-bit characteristic UUID
    BLERead | BLENotify, 2);  // remote clients will be able to get notifications if this characteristic changes
                              // the characteristic is 2 bytes long as the first field needs to be "Flags" as per BLE specifications
                              // https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.heart_rate_measurement.xml

int oldHeartRate = 0;  // last heart rate reading from analog input
long previousMillis = 0;  // last time the heart rate was checked, in ms

// Nonsense I added for experimentation:
int heartRateMeasurement = 250;
int increment = 10;

void setup() {
  Serial.begin(9600);    // initialize serial communication
  pinMode(13, OUTPUT);   // initialize the LED on pin 13 to indicate when a central is connected

  /* Set a local name for the BLE device
     This name will appear in advertising packets
     and can be used by remote devices to identify this BLE device
     The name can be changed but maybe be truncated based on space left in advertisement packet */
  blePeripheral.setLocalName("HeartRateSketch");
  blePeripheral.setAdvertisedServiceUuid(heartRateService.uuid());  // add the service UUID
  blePeripheral.addAttribute(heartRateService);   // Add the BLE Heart Rate service
  blePeripheral.addAttribute(heartRateChar); // add the Heart Rate Measurement characteristic

  /* Now activate the BLE device.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection */
  blePeripheral.begin();
  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop() {
  // listen for BLE peripherals to connect:
  BLECentral central = blePeripheral.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(13, HIGH);

    // check the heart rate measurement every 200ms
    // as long as the central is still connected:
    while (central.connected()) {
      long currentMillis = millis();
      Serial.print(currentMillis);
      // if 200ms have passed, check the heart rate measurement:
      if (currentMillis - previousMillis >= 100) {
        previousMillis = currentMillis;
        updateHeartRate();
      }
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(13, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

void updateHeartRate() {
  /* Read the current voltage level on the A0 analog input pin.
     This is used here to simulate the heart rate's measurement.
  */
  heartRateMeasurement += increment;

  if (heartRateMeasurement > 500 || heartRateMeasurement < 100){
    increment *= -1;
  }
  
  int heartRate = map(heartRateMeasurement, 0, 1023, 0, 100); // Macall - changed last number from 100 to 0
  if (heartRate != oldHeartRate) {      // if the heart rate has changed
    Serial.print("Heart Rate is now: "); // print it
    Serial.println(heartRate);
    const unsigned char heartRateCharArray[2] = { 0, (const unsigned char)heartRate };
    heartRateChar.setValue(heartRateCharArray, 2);  // and update the heart rate measurement characteristic
    oldHeartRate = heartRate;           // save the level for next comparison
  }
} 

