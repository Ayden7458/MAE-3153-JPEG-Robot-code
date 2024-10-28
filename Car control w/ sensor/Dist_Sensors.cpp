#include <VL53L0X.h>
#include <Wire.h>
#include "Freenove_4WD_Car_for_Arduino.h"
#include "Dist_Sensors.h"



VL53L0X dist_sensors[N_DIST_SENSORS];  // create the distance sensors
float distances[N_DIST_SENSORS];   // an array to hold the distance measurements

float calibration_offset = 0;
extern float error_integral = 0;
extern int error_sign = 1;
extern int prev_time = 0;



void I2C_select(int i) {
  if (i > 7) return;
  Wire.beginTransmission(0x70);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

//void setup_distance_sensors(VL53L0X* sensors, int n_sensors) {
void setup_distance_sensors() {

  uint8_t i;
  for (i = 0; i < N_DIST_SENSORS; i++) {
    I2C_select(i);
    dist_sensors[i].setTimeout(100);
    if (!dist_sensors[i].init())
    {
      Serial.print("Failed to detect and initialize sensor!  ");
      Serial.println(i);
    }    

    dist_sensors[i].setMeasurementTimingBudget(25000);
    dist_sensors[i].startContinuous(25);
    error_integral = 0;
 
  }
} 

void read_distance_sensors() {
  uint8_t i;
  int val;
  for (i = 0; i < N_DIST_SENSORS; i++) {
    I2C_select(i);
    //val = sensors[i].read();
    distances[i] = distances[i]*FILTER_STRENGTH + (1.0-FILTER_STRENGTH)*dist_sensors[i].readRangeSingleMillimeters();
   }
}

void startControl(void){
  error_integral = 0;
  error_sign = +1;
  prev_time = 0;

}


void gostraightP(int forward_speed, int setpoint) {
  int i, diff, error, correction, leftspeed, rightspeed;
  read_distance_sensors(); 
  diff = distances[SIDE_FRONT]- distances[SIDE_REAR]; // error means not parallel to the wall
  diff = diff - calibration_offset; //  installed calibration offset
  error = setpoint - diff;  // heading LEFT of the setpoint produces a POSITIVE error value
  correction = -error * K_PROPORTIONAL;

  leftspeed = forward_speed + correction; 
  rightspeed = forward_speed - correction;

  motorRun(leftspeed, rightspeed); 
  Serial.print(correction);  
  Serial.print("   ");
  Serial.print(leftspeed);
  Serial.print("   ");
  Serial.print(rightspeed);
  Serial.println("   -300   300");

}


void gostraightDistance(int forward_speed, int setpoint) {
  int i, diff, error, correction, leftspeed, rightspeed;
  float theta,dwall;
  read_distance_sensors(); 
  diff = distances[SIDE_FRONT]- distances[SIDE_REAR]; // error means not parallel to the wall
  //diff = diff - calibration_offset; //  installed calibration offset
  dwall = distances[SIDE_FRONT] * (1- pow(diff/102.0,2)/2.0);

  error = setpoint - dwall;  // heading LEFT of the setpoint produces a POSITIVE error value
  correction = -error * K_PROPORTIONAL;

  leftspeed = forward_speed + correction; 
  rightspeed = forward_speed - correction;

  motorRun(leftspeed, rightspeed); 
  Serial.print(correction);  
  Serial.print("   ");
  Serial.print(leftspeed);
  Serial.print("   ");
  Serial.print(rightspeed);
  Serial.println("   -300   300");

}


void gostraightPI(int forward_speed, int setpoint, int currenttime) {
  int i, diff, error, correction, leftspeed, rightspeed;
  int deltatime, eisign;
  read_distance_sensors(); 
  diff = distances[SIDE_FRONT]- distances[SIDE_REAR]; // error means not parallel to the wall
  diff = diff - calibration_offset; //  installed calibration offset
  error = setpoint - diff;  // heading LEFT of the setpoint produces a POSITIVE error value

  correction = -error * K_PROPORTIONAL;

  deltatime = currenttime - prev_time;
  prev_time = currenttime;
  error_integral += error*deltatime/1000.0;
  eisign = abs(error)/error;
  if (eisign  !=  error_sign) {
    error_sign *= -1;
    error_integral = -error_integral/5;
  }


  if (abs(error_integral) > 25) {
    error_integral = 25 * eisign;
  }

  correction -= error_integral*K_INTEGRAL;

  leftspeed = forward_speed + correction; 
  rightspeed = forward_speed - correction;

  motorRun(leftspeed, rightspeed); 

  Serial.print(error);  
  Serial.print("   ");
  Serial.print(leftspeed);
  Serial.print("   ");
  Serial.print(rightspeed);
  Serial.print("   ");
  Serial.print(error_integral);
  Serial.print("   ");
  Serial.print(deltatime);
  
  Serial.println("   -300   300");

}


void read_calibration_offset(int durationMS) {
  unsigned long starttime;
  bool  finished = false;
  starttime = millis();
  while (finished == false) {
    read_distance_sensors(); 
    if (millis() - starttime > durationMS) {
      finished = true;
    }
  }
  calibration_offset = (distances[SIDE_FRONT]- distances[SIDE_REAR]);
  Serial.print("calibration_offset ");
  Serial.println(calibration_offset);
 }

 
