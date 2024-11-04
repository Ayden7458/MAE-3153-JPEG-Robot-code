
#include "Freenove_4WD_Car_for_Arduino.h"
#include "RF24_Remote.h"

//RDD Add small display screen and code to set it up and use it
#include <Wire.h>
#include "SSD1306AsciiWire.h"
#include "Dist_Sensors.h"

#define I2C_ADDRESS 0x3C
#define ALIGNMENT_TOLERANCE 5       // Tolerance level for front sensor alignment (adjust as needed)
#define TURN_SPEED 150              // Speed for turning adjustments (adjust as needed)
#define SET_DISTANCE_RIGHT_WALL 30  // Desired distance from the right wall (adjust as needed)
#define FORWARD_SPEED 200           // Speed for forward movement (adjust as needed)
#define SET_DISTANCE_SIDE_SENSOR_TO_OPPOSING_WALL 30 // Replace with the desired distance threshold for the side sensor
#define SET_DISTANCE_FROM_PEG_WALL 20 // Replace with the desired distance threshold for the front sensors to the peg wall



// Update the logic to use these constants

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

void loop() {
  // Autonomous mode behavior
  if (mode == MODE_AUTO) {
    bool frontEqual = (distances[0] == distances[1]);  // Assuming front left sensor is distances[0], front right is distances[1]
    bool sideAtSetDistance = (distances[2] == SET_DISTANCE_SIDE_SENSOR_TO_OPPOSING_WALL); // Using defined constant
    bool frontAtPegWall = (distances[0] == SET_DISTANCE_FROM_PEG_WALL);  // Using defined constant

    if (frontEqual) {
      if (sideAtSetDistance) {
        if (frontAtPegWall) {
          // Move forward along the peg wall
          motorRun(255, 255);
        } else {
          // Turn based on peg wall distance
          motorRun(distances[0] > SET_DISTANCE_FROM_PEG_WALL ? 255 : -255, 
                   distances[0] > SET_DISTANCE_FROM_PEG_WALL ? -255 : 255);
        }
      } else {
        // Sharper turn based on side sensor distance
        motorRun(distances[2] > SET_DISTANCE_SIDE_SENSOR_TO_OPPOSING_WALL ? 255 : -255, 
                 distances[2] > SET_DISTANCE_SIDE_SENSOR_TO_OPPOSING_WALL ? -255 : 255);
      }
    } else {
      // Adjust to equalize front sensor readings
      motorRun(distances[0] > distances[1] ? 150 : -150, 
               distances[0] > distances[1] ? -150 : 150);
    }
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
