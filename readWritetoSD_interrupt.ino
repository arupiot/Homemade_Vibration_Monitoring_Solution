/* Acceleration data logger
  Script to read data from the accelerometer and write to the SD card
  Data is recorded for a set time period once acceleration exceeds a specfied value
  Uses Sparkfun MPU-9250 (MEMS accelerometer)
  Created: 14/09/2017
*/

// Define libaries
#include <Wire.h>
// accelerometer libraries:
#include "quaternionFilters.h"
#include "MPU9250.h"
// #include "DS3231.h"   // RTC
#include <SPI.h>
#include <SD.h>

// Set the pin used for SD card
#define cardSelect 4
File logfile;   // file object for writing data to

// Declare accelerometer
MPU9250 myIMU;

// Variables for setting sampling frequency
long prevTime = 0;          // stores when last reading was taken
long interval = 10;         // interval at which to take readings (milliseconds) 10ms --> 100Hz
long currentTime = 0;

// Trigger acceleration
float trigger = 1.5; // in g (incl gravity)

void setup() {  // runs once when sketch is uploaded

  Wire.begin();       // For communicating with accelerometer
  Serial.begin(9600); // Start serial output - speed is in bits/second (for writing to serial monitor)

  // Initialize digital pin 13 as an output (for switching LED on when data is recording)
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);    // turn the LED off initially

  ///// ACCELEROMETER SETUP /////

  // Calibrate gyro and accelerometer, load biases in bias registers
  myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
  Serial.println("MPU9250 calibrated");

  // Initialize device for active mode read of acclerometer
  myIMU.initMPU9250();

  Serial.println("MPU9250 initialized for active data mode....");

}

void loop() { // runs repeatedly

  myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values  (accel count is a vector with three values)
  myIMU.getAres();                        // Get x/y/z resolution

  // Calculate z axis acceleration in g
  myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes - myIMU.accelBias[2];

  // Check if value exceeds trigger acceleration
  if (myIMU.az > trigger || myIMU.az < -trigger) {

    digitalWrite(13, HIGH);   // turn the LED on to indicate data is being recorded

    // see if the SD card is present and can be initialized:
    if (!SD.begin(cardSelect)) {
      Serial.println("Card init. failed!");
    }

    // create a file on SD card to save the data in
    char filename[15];
    strcpy(filename, "DATA0000.csv");
    for (uint8_t i = 0; i < 100; i++) {
      filename[6] = '0' + i / 10;
      Serial.print("Filename6 = "); Serial.println(filename[6]);
      filename[7] = '0' + i % 10;
      Serial.print("Filename7 = "); Serial.println(filename[7]);
      // create if does not exist, do not open existing, write, sync after write
      if (! SD.exists(filename)) {
        break;
      }
    }


    // open file on SD card
    logfile = SD.open(filename, FILE_WRITE);
    if ( ! logfile ) {
      Serial.print("Couldnt create ");
      Serial.println(filename);
    }
    Serial.print("Writing to ");
    Serial.println(filename);

    // Record data
    int num = 1;
    while (num <= 1000)  {  // run for 30s at 100Hz
      currentTime = millis();
      if (currentTime - prevTime > interval)   {

        myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values  (accel count is a vector with three values)
        myIMU.getAres();                        // Get the resolution
        myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes - myIMU.accelBias[2];
        logfile.print(myIMU.az, 8);      // print z value to SD card (to 8dp)
        logfile.print(",");              // print comma on SD card in between each reading
        prevTime = currentTime;          // save last time reading was taken
        num = num + 1;

      }
    }

    logfile.flush();    // save data to SD card
    digitalWrite(13, LOW);   // turn the LED off to indicate data is no longer being recorded

  }
}

