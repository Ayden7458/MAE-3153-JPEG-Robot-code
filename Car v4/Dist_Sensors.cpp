  #include <VL53L0X.h>
#include <Wire.h>
#include "Freenove_4WD_Car_for_Arduino.h"
#include "Dist_Sensors.h"



VL53L0X dist_sensors[N_DIST_SENSORS];  // create the distance sensors
float distances[N_DIST_SENSORS];   // an array to hold the distance measurements

float calibration_offset = 0;
float dstart = 250;
float rearstart;


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


void gostraightDistance(float setpoint) {
  int i, diff, error, Pcorrection, Icorrection,correction,leftspeed, rightspeed,sign_C;
  int deltatime, eisign;
  float theta,dwall;
  read_distance_sensors(); 

  // if (distances[REAR] > rearstart) {
  //   motorRun(0,0);
  //   shake();
  //   shake();
  //   shake();
  //   return;
  // }

  diff = distances[SIDE_FRONT]- distances[SIDE_REAR]; // error means not parallel to the wall
  diff = diff - calibration_offset; //  installed calibration offset
  dwall = distances[SIDE_FRONT] * (1- pow(diff/102.0,2)/2.0);

  error = setpoint - dwall;  // heading LEFT of the setpoint produces a POSITIVE error value
 if (error > 2) {
    motorRun(LEFT_SLOW, RIGHT_SLOW+BANG);
  } else if (error < -2) {
    motorRun(LEFT_SLOW+BANG, RIGHT_SLOW);
  } else {
    motorRun(LEFT_SLOW, RIGHT_SLOW); 
  }
  Serial.println("error  setpt    dwall  ");
  Serial.print(error);  
  Serial.print("         ");
  Serial.print(setpoint);  
  Serial.print("         ");
  Serial.print(dwall);  
  Serial.print("         ");
  Serial.println("            -300   300");
}


void shake() {
  motorRun(SHAKE_MAG, -SHAKE_MAG);
  delay(SHAKE_TIME);
  motorRun(-SHAKE_MAG, SHAKE_MAG);
  delay(SHAKE_TIME);
  motorRun(0, 0);

}


void read_calibration_offset() {
  unsigned long starttime;
  bool  finished = false;
  read_distance_sensors(); 
  calibration_offset = (distances[SIDE_FRONT]- distances[SIDE_REAR]);
  dstart = distances[SIDE_FRONT];
  rearstart = distances[REAR];
 }

 
