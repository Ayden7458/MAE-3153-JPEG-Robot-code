/**********************************************************************
  Filename    : RF24_Remote_Car.ino
  Product     : Freenove 4WD Car for UNO
  Description : A RF24 Remote Car.
  Auther      : www.freenove.com
  Modification: 2020/11/27
**********************************************************************/
#include "Freenove_4WD_Car_for_Arduino.h"
#include "RF24_Remote.h"

//RDD Add code to use the servo
#include <Servo.h>
Servo servo1;

#define NRF_UPDATE_TIMEOUT    1000

u32 lastNrfUpdateTime = 0;
int pos = 90;  // Start servo position at 90 (neutral)

void setup() {
  pinsSetup();
  if (!nrf24L01Setup()) {
    alarm(3, 2);
  }
  alarm(2, 1);  //RDD
  pinsSetup();
  Serial.begin(9600);  // Initialize serial communication
  if (!nrf24L01Setup()) {
    alarm(3, 2);
  }
  alarm(2, 1);  //RDD
  
  // Attach the servo to pin 2
  servo1.attach(2);
  servo1.write(pos);  // Set servo to the starting position
}

void loop() {
  if (getNrf24L01Data()) {  // If radio data was received
    clearNrfFlag();
    
    // Print the received data for debugging
    Serial.print("Received Data: ");
    for (int i = 0; i < 8; i++) {
      Serial.print(nrfDataRead[i]);
      Serial.print(" ");
    }
    Serial.println();
    
    // Check if S1 or S2 is pressed, and move the servo accordingly
    if (nrfDataRead[5] == 0) {
      // Move servo clockwise while S1 is held
      pos += 3;  // Increment the servo position
      if (pos > 180) pos = 180;  // Cap the position at 180 degrees
      servo1.write(pos);  // Write new position to the servo
    } 
    else if (nrfDataRead[6] == 0) {
      // Move servo counterclockwise while S2 is held
      pos -= 3;  // Decrement the servo position
      if (pos < 0) pos = 0;  // Cap the position at 0 degrees
      servo1.write(pos);  // Write new position to the servo
    } 

    updateCarActionByNrfRemote(); 
    lastNrfUpdateTime = millis();
  }

  if (millis() - lastNrfUpdateTime > NRF_UPDATE_TIMEOUT) {
    lastNrfUpdateTime = millis();
    resetNrfDataBuf();
    updateCarActionByNrfRemote();
  }

  delay(20);  // Small delay to smooth out servo movements
}
