/**********************************************************************
  Filename    : RF24_Remote_Car.ino
  Product     : Freenove 4WD Car for UNO
  Description : new code 10.31.20241
  Auther      : www.freenove.com
  Modification: 2024/10/31
**********************************************************************/
#include "Freenove_4WD_Car_for_Arduino.h"
#include "RF24_Remote.h"

//RDD Add small display screen and code to set it up and use it
#include <Wire.h>
#include "SSD1306AsciiWire.h"
#include "Dist_Sensors.h"

#define I2C_ADDRESS 0x3C
SSD1306AsciiWire oled;

void setupScreen() {
  Wire.begin();
  Wire.setClock(400000L);
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.clear();
  oled.setFont(fixed_bold10x15);
}

void clearScreen() {
  I2C_select(3);
  oled.clear();
}

void screenPrint(int row, int col, char* value) {
  I2C_select(3);
  oled.clearField((col - 1) *11, (row - 1) * 2, 12);    
  oled.print(value);
}

void screenPrintNum(int row,int col, int width, int places, float val) {
  I2C_select(3);
  char buffer[6];
  dtostrf(val,width,places,buffer);
  screenPrint(row,col,buffer);  
}




#include <Servo.h>
Servo servo1;

#define NRF_UPDATE_TIMEOUT    1000

#define MAX_AUTO_TIME 15000
#define MODE_STOP 0
#define MODE_MANUAL 1
#define MODE_AUTO 2

#define AUTO_MODE_STOP 0
#define AUTO_MODE_FORWARD 1
#define AUTO_MODE_BACKWARD 2
#define AUTO_MODE_RIGHTTURN 3
#define AUTO_MODE_LEFTTURN 4
#define AUTO_MODE_SERVO_POS 5
#define AUTO_MODE_SERVO_NEG 6
#define AUTO_MODE_FORWARD_SLOW 7
#define AUTO_MODE_READ_OFFSETS 8
#define AUTO_MODE_READ_DISTANCE 9
#define AUTO_MODE_GO_STRAIGHT_DIST 10
#define AUTO_MODE_SHAKE 11
#define AUTO_MODE_END_AUTO 12

#define SERVO_STOP 91

int mode, automode, autokey, start_time, current_time, delta_time;


u32 lastNrfUpdateTime = 0;

void setup() {
  Serial.begin(115200);  // to print to the serial monitor
  Serial.println("Starting");

  pinsSetup();
  servo1.attach(PIN_SERVO);  //RDD
  if (!nrf24L01Setup()) {
    alarm(3, 2);
  }
  alarm(2, 1);  //RDD
  mode = MODE_MANUAL;  //RDD

  setupScreen();   //RDD  Setup screen and print the group number and battery voltage
  setup_distance_sensors(); 
  setupScreen();
  clearScreen();
  screenPrint(1, 1, "Sensors");
  setup_distance_sensors(); 
}

int counter = 0;
void loop() {
  int i;

  // Read and print distance sensor data
  read_distance_sensors(); 

  // Display sensor data on OLED every few loops to reduce screen flicker
  counter++;
  if (counter == 5) {
    counter = 0;
    for (i = 0; i < N_DIST_SENSORS; i++){
      screenPrintNum(i+2, 2, 6, 0, distances[i]); // Update display with sensor distances
    }
  }

  // Check for RF data and control car movement
  if (getNrf24L01Data()) {
    clearNrfFlag();

    if (mode == MODE_MANUAL) {
      manual(); // Calls function to control the car
    } else if (mode == MODE_STOP) {
      servo1.write(SERVO_STOP);
      motorRun(0,0);  // Stop motors
    }

    lastNrfUpdateTime = millis(); // Reset timer for data update check
  }
}




void manual() {
  updateCarActionByNrfRemote();  //make the car go

  //handle the servo
  float servospeed = nrfDataRead[0] /1023.0;  // 1023.0 not 1023 
  //Serial.println(servospeed);
  if (nrfDataRead[5] == 0 ) {
    servo1.write( int(91 + servospeed * 36));  // servo reaches max  positive speed at 127
  }  else if (nrfDataRead[6] == 0 ) {
    servo1.write(int(91 - servospeed * 32));  // servo reaches max  negative speed at 63
  } else {
    servo1.write(91); //servo stops at 91
  }
}  




