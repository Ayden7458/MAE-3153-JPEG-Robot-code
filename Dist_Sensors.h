#include <VL53L0X.h>
#include <Wire.h>

#define N_DIST_SENSORS 3  // number of distance sensors attached

//Sensor locations
#define SIDE_FRONT  0
#define SIDE_REAR   1
#define REAR  2


#define K_PROPORTIONAL    3.0   //Kp - proportional gain
#define K_INTEGRAL    4.0   //Kp - proportional gain
#define FILTER_STRENGTH  0.80

extern VL53L0X dist_sensors[N_DIST_SENSORS];  // create the distance sensors
extern  float distances[N_DIST_SENSORS];   // an array to hold the distance measurements
extern float calibration_offset;
extern float error_integral;
extern int error_sign;
extern int prev_time;


void I2C_select(int);
 
//void setup_distance_sensors(VL53L0X* , int );
void setup_distance_sensors();
 
void read_distance_sensors();

void startControl(void);

void gostraightP(int forward_speed, int setpoint);

void gostraightPI(int forward_speed, int setpoint, int deltatime);

void gostraightDistance(int forward_speed, int setpoint);

void read_calibration_offset(int durationMS);

