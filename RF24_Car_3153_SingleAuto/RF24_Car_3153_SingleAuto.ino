/**********************************************************************
  Filename    : RF24_Remote_Car.ino
  Product     : Freenove 4WD Car for UNO
  Description : A RF24 Remote Car.
  Auther      : www.freenove.com
  Modification: 2020/11/27
**********************************************************************/
#include "Freenove_4WD_Car_for_Arduino.h"
#include "RF24_Remote.h"

//RDD Add small display screen and code to set it up and use it
#include <Wire.h>
#include "SSD1306AsciiWire.h"
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
  oled.clear();
}

void screenPrint(int row, int col, char* value) {
  oled.clearField((col - 1) *11, (row - 1) * 2, 12);    
  oled.print(value);
}

void printVoltage(int row,int col) {
  getBatteryVoltage();  
  char buffer[6];
  dtostrf(batteryVoltage,4,2,buffer);
  screenPrint(row,col,buffer);  
  screenPrint(row,col+5,"Volts");
}


//RDD Add code to use the servo
#include <Servo.h>
Servo servo1;

#define NRF_UPDATE_TIMEOUT    1000

//RDD add modes
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

#define SERVO_STOP 91

int mode, automode, start_time, current_time, delta_time;


u32 lastNrfUpdateTime = 0;

void setup() {
  pinsSetup();
  servo1.attach(PIN_SERVO);  //RDD
  if (!nrf24L01Setup()) {
    alarm(3, 2);
  }
  alarm(2, 1);  //RDD
  mode = MODE_MANUAL;  //RDD

  setupScreen();   //RDD  Setup screen and print the group number and battery voltage
  screenPrint(1,3,"Group 11");
  printVoltage(2,1);
  screenPrint(3,1,"Mode: Manual");
}


void loop() {

  if (getNrf24L01Data()) {  //if radio data was received
    clearNrfFlag();
  
    if (mode == MODE_STOP  ) {
      if (nrfDataRead[7] == 0) {
        mode = MODE_AUTO;
        printVoltage(2,1);
        screenPrint(3,1,"Mode: Auto");        
              
        automode = AUTO_MODE_STOP;
        alarm(1, 1);     
        start_time = millis();
        //Serial.println(start_time);
        servo1.write(SERVO_STOP);
      }  
      if (nrfDataRead[4] == 0) {
        mode = MODE_MANUAL;
         printVoltage(2,1);
        screenPrint(3,1,"Mode: Manual"); 
        screenPrint(4,1," ");   
        alarm(2, 1);  
        servo1.write(SERVO_STOP);
      }  
    }

    if (mode == MODE_AUTO){
      current_time = millis();
      delta_time = current_time - start_time;

      if (delta_time > 15000) {
        mode = MODE_MANUAL;
        printVoltage(2,1);
        screenPrint(3,1,"Mode: Manual"); 
        screenPrint(4,1," ");          
        alarm(2, 1);            
      } else {
        autonomous(delta_time);
      }
    }

    if (mode == MODE_MANUAL  ) {
      //the joystick controls the servo positions
      if (nrfDataRead[4] == 0) {
        mode = MODE_STOP;
        printVoltage(2,1);
        screenPrint(3,1,"Mode: Stop"); 
        screenPrint(4,1," ");   
        alarm(3, 1);  
        servo1.write(SERVO_STOP);
      }
      manual();
    }

    lastNrfUpdateTime = millis();
  }

  if (millis() - lastNrfUpdateTime > NRF_UPDATE_TIMEOUT) {
    lastNrfUpdateTime = millis();
    resetNrfDataBuf();
    updateCarActionByNrfRemote();
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


void autonomous(int now) {
  int endtime_prev = 0;
  //                    mode        current_time  start   duration
  endtime_prev = do_auto(AUTO_MODE_FORWARD,   now,    0,      1500);
  endtime_prev = do_auto(AUTO_MODE_STOP,      now,  endtime_prev, 1000);
  endtime_prev = do_auto(AUTO_MODE_BACKWARD,  now,  endtime_prev, 1500);
  endtime_prev = do_auto(AUTO_MODE_STOP,      now,  endtime_prev, 1000);
  endtime_prev = do_auto(AUTO_MODE_RIGHTTURN, now,  endtime_prev, 1500);
  endtime_prev = do_auto(AUTO_MODE_STOP,      now,  endtime_prev, 1000);
  endtime_prev = do_auto(AUTO_MODE_LEFTTURN,  now,  endtime_prev, 1500);
  endtime_prev = do_auto(AUTO_MODE_STOP,      now,  endtime_prev, 1000);
  endtime_prev = do_auto(AUTO_MODE_SERVO_POS, now,  endtime_prev, 1500);
  endtime_prev = do_auto(AUTO_MODE_STOP,      now,  endtime_prev, 1000);
  endtime_prev = do_auto(AUTO_MODE_SERVO_NEG, now,  endtime_prev, 1500);
  endtime_prev = do_auto(AUTO_MODE_STOP,      now,  endtime_prev, 500);
}

int do_auto(int automode, int current_time,  int start_time, int duration) {

  int stop_time = start_time + duration;

  if (current_time > start_time  &&   current_time < stop_time)

  switch (automode) {
  case AUTO_MODE_STOP:
    motorRun(0, 0);
    servo1.write(90);
      break;
  case AUTO_MODE_FORWARD:
    motorRun(150, 150);
      break;
  case AUTO_MODE_BACKWARD:
    motorRun(-150, -150);
      break;
  case AUTO_MODE_RIGHTTURN:
      motorRun(200, -200);
      break;
  case AUTO_MODE_LEFTTURN:
      motorRun(-185, 185);
      break;
  case AUTO_MODE_SERVO_POS:
      motorRun(0, 0);
      servo1.write(180);
      break;
  case AUTO_MODE_SERVO_NEG:
      motorRun(0, 0);
      servo1.write(0);
      break;
  }  
  return stop_time;
}

