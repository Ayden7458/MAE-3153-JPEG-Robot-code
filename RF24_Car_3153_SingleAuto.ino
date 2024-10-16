/**********************************************************************
  Filename    : RF24_Remote_Car.ino
  Product     : Freenove 4WD Car for UNO
  Description : A RF24 Remote Car.
  Auther      : www.freenove.com
  Modification: 2020/11/27
**********************************************************************/

#include "Freenove_4WD_Car_for_Arduino.h"  // Include the main library for controlling the 4WD car
#include "RF24_Remote.h"  // Include the library to enable RF24 remote communication

// RDD Add small display screen and code to set it up and use it
#include <Wire.h>  // Include the Wire library for I2C communication
#include "SSD1306AsciiWire.h"  // Include the library for the OLED display

#define I2C_ADDRESS 0x3C  // Define the I2C address for the OLED display
SSD1306AsciiWire oled;  // Initialize an instance of the OLED display object

// Function to set up the OLED screen
void setupScreen() {
  Wire.begin();  // Start I2C communication
  Wire.setClock(400000L);  // Set the I2C clock speed to 400kHz
  oled.begin(&Adafruit128x64, I2C_ADDRESS);  // Initialize the OLED with the correct display model
  oled.clear();  // Clear the display
  oled.setFont(fixed_bold10x15);  // Set the display font to a bold fixed-size font
}

// Function to clear the OLED screen
void clearScreen() {
  oled.clear();  // Clear the screen
}

// Function to print a value on the OLED screen at a specific row and column
void screenPrint(int row, int col, char* value) {
  oled.clearField((col - 1) * 11, (row - 1) * 2, 12);  // Clear the current field where the new value will be printed
  oled.print(value);  // Print the new value
}

// Function to get and display the battery voltage on the OLED screen
void printVoltage(int row,int col) {
  getBatteryVoltage();  // Retrieve the battery voltage
  char buffer[6];  // Create a buffer to store the voltage as a string
  dtostrf(batteryVoltage,4,2,buffer);  // Convert the voltage to a formatted string
  screenPrint(row,col,buffer);  // Print the voltage value on the screen
  screenPrint(row,col+5,"Volts");  // Append the word 'Volts' to the displayed voltage
}

// RDD Add code to use the servo
#include <Servo.h>  // Include the Servo library
Servo servo1;  // Create a Servo object to control the servo motor

// Define timeout for updating RF24 data
#define NRF_UPDATE_TIMEOUT    1000

// Define operation modes
#define MODE_STOP 0  // Stopped mode
#define MODE_MANUAL 1  // Manual driving mode
#define MODE_AUTO 2  // Autonomous driving mode

// Define autonomous sub-modes
#define AUTO_MODE_STOP 0  // Stop action in autonomous mode
#define AUTO_MODE_FORWARD 1  // Move forward in autonomous mode
#define AUTO_MODE_BACKWARD 2  // Move backward in autonomous mode
#define AUTO_MODE_RIGHTTURN 3  // Turn right in autonomous mode
#define AUTO_MODE_LEFTTURN 4  // Turn left in autonomous mode
#define AUTO_MODE_SERVO_POS 5  // Rotate servo to positive direction in autonomous mode
#define AUTO_MODE_SERVO_NEG 6  // Rotate servo to negative direction in autonomous mode

#define SERVO_STOP 91  // Servo stop position

int mode, automode, start_time, current_time, delta_time;  // Variables to track modes, time, and timing intervals

u32 lastNrfUpdateTime = 0;  // Variable to track the last time RF24 data was received

// Setup function, runs once when the car starts
void setup() {
  pinsSetup();  // Initialize pin configurations for the car
  servo1.attach(PIN_SERVO);  // Attach the servo motor to its control pin
  if (!nrf24L01Setup()) {  // Set up the RF24 communication; alarm if it fails
    alarm(3, 2);  // Trigger alarm if RF24 setup fails
  }
  alarm(2, 1);  // Trigger a success alarm for RF24 setup

  mode = MODE_MANUAL;  // Set initial mode to manual control

  setupScreen();   // Initialize the OLED screen
  screenPrint(1,3,"Group 11");  // Print group number
  printVoltage(2,1);  // Display the initial battery voltage
  screenPrint(3,1,"Mode: Manual");  // Display the initial mode
}

// Loop function, runs continuously while the car is powered
void loop() {

  // If RF24 data was received
  if (getNrf24L01Data()) {
    clearNrfFlag();  // Clear the flag indicating data reception

    // Mode transition logic based on the received data
    if (mode == MODE_STOP) {
      if (nrfDataRead[7] == 0) {
        mode = MODE_AUTO;  // Switch to autonomous mode
        printVoltage(2,1);  // Update the voltage on the screen
        screenPrint(3,1,"Mode: Auto");  // Indicate the current mode on the screen

        automode = AUTO_MODE_STOP;  // Start autonomous mode in a stopped state
        alarm(1, 1);  // Trigger an alarm to indicate mode change
        start_time = millis();  // Record the start time for autonomous operations
        servo1.write(SERVO_STOP);  // Stop the servo motor initially
      }
      if (nrfDataRead[4] == 0) {
        mode = MODE_MANUAL;  // Switch back to manual mode
        printVoltage(2,1);  // Update the voltage on the screen
        screenPrint(3,1,"Mode: Manual");  // Indicate the current mode
        screenPrint(4,1," ");  // Clear additional screen rows
        alarm(2, 1);  // Trigger alarm for manual mode
        servo1.write(SERVO_STOP);  // Stop the servo motor
      }
    }

    // Autonomous mode behavior
    if (mode == MODE_AUTO) {
      current_time = millis();  // Get the current time
      delta_time = current_time - start_time;  // Calculate elapsed time since starting autonomous mode

      // If 15 seconds have passed, switch back to manual mode
      if (delta_time > 15000) {
        mode = MODE_MANUAL;  // Revert to manual mode
        printVoltage(2,1);  // Update voltage
        screenPrint(3,1,"Mode: Manual");  // Display manual mode
        screenPrint(4,1," ");  // Clear additional screen rows
        alarm(2, 1);  // Trigger alarm
      } else {
        autonomous(delta_time);  // Continue autonomous operations
      }
    }

    // Manual mode behavior
    if (mode == MODE_MANUAL) {
      if (nrfDataRead[4] == 0) {  // If stop command is received
        mode = MODE_STOP;  // Switch to stop mode
        printVoltage(2,1);  // Update voltage
        screenPrint(3,1,"Mode: Stop");  // Indicate stop mode on the screen
        screenPrint(4,1," ");  // Clear additional rows
        alarm(3, 1);  // Trigger stop alarm
        servo1.write(SERVO_STOP);  // Stop the servo
      }
      manual();  // Execute manual controls
    }

    lastNrfUpdateTime = millis();  // Update the timestamp for the last RF24 data reception
  }

  // Check if RF24 communication has timed out
  if (millis() - lastNrfUpdateTime > NRF_UPDATE_TIMEOUT) {
    lastNrfUpdateTime = millis();  // Reset the last update time
    resetNrfDataBuf();  // Reset the RF24 data buffer
    updateCarActionByNrfRemote();  // Update the car's action based on the RF24 data
  }
}

// Manual control function
void manual() {
  updateCarActionByNrfRemote();  // Update the car's movement based on remote commands

  // Control the servo motor based on joystick input
  float servospeed = nrfDataRead[0] / 1023.0;  // Calculate servo speed from joystick data
  if (nrfDataRead[5] == 0) {
    servo1.write(int(91 + servospeed * 36));  // Move servo to positive direction
  } else if (nrfDataRead[6] == 0) {
    servo1.write(int(91 - servospeed * 32));  // Move servo to negative direction
  } else {
    servo1.write(91);  // Stop the servo if no input is given
  }
}

// Autonomous mode function
void autonomous(int now) {
  int endtime_prev = 0;
  // Sequence of actions for autonomous mode
  endtime_prev = do_auto(AUTO_MODE_FORWARD, now, 0, 1500);
  endtime_prev = do_auto(AUTO_MODE_STOP, now, endtime_prev, 1000);
  endtime_prev = do_auto(AUTO_MODE_BACKWARD, now
