/**********************************************************************
  Filename    : RF24_Remote_Car.ino
  Product     : Freenove 4WD Car for UNO
  Description : A RF24 Remote Car.
  Auther      : www.freenove.com
  Modification: 2020/11/27
**********************************************************************/
#include "Freenove_4WD_Car_for_Arduino.h"
#include "RF24_Remote.h"

#include <Servo.h>
Servo servo1;

#define NRF_UPDATE_TIMEOUT    1000

u32 lastNrfUpdateTime = 0;

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
  servo1.write(90);  // Set servo to stop (neutral position)
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
      // Rotate clockwise while S1 is held
      servo1.write(180);  // Signal for continuous rotation clockwise
    } 
    else if (nrfDataRead[6] == 0) {
      // Rotate counterclockwise while S2 is held
      servo1.write(90);  // Signal for continuous rotation counterclockwise
    } 
    else {
      // Stop the servo when no buttons are pressed
      servo1.write(0);  // Neutral signal to stop the continuous rotation
    }

    updateCarActionByNrfRemote(); 
    lastNrfUpdateTime = millis();
  }

  if (millis() - lastNrfUpdateTime > NRF_UPDATE_TIMEOUT) {
    lastNrfUpdateTime = millis();
    resetNrfDataBuf();
    updateCarActionByNrfRemote();
  }

  delay(20);  // Small delay for smoother control
}
