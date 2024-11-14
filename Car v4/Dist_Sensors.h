#include <VL53L0X.h>
#include <Wire.h>

#define LEFT_SLOW  60
#define RIGHT_SLOW  39
#define BANG  47
#define SHAKE_MAG  75
#define SHAKE_TIME  100
#define FILTER_STRENGTH  0.70


#define N_DIST_SENSORS 3  // number of distance sensors attached

//Sensor locations
#define SIDE_FRONT  0
#define SIDE_REAR   1
#define REAR  2


extern VL53L0X dist_sensors[N_DIST_SENSORS];  // create the distance sensors
extern  float distances[N_DIST_SENSORS];   // an array to hold the distance measurements
extern float calibration_offset;
extern float dstart, rearstart;


void I2C_select(int);
 
//void setup_distance_sensors(VL53L0X* , int );
void setup_distance_sensors();
 
void read_distance_sensors();

void gostraightDistance(float setpoint);

void read_calibration_offset();

void shake();

