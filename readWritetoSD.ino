/* Acceleration data logger
  Script to read data from the accelerometer and write to the SD card
  Uses Sparkfun MPU-9250
*/

#include <Wire.h>   // do we need this?
#include "quaternionFilters.h"
#include "MPU9250.h"  // accelerometer
// #include "DS3231.h"   // board RTC
#include <SPI.h>
#include <SD.h>

// Set the pin used for SD card
#define cardSelect 4
File logfile;   // file for storing accelerometer data

// declare accelerometer
MPU9250 myIMU;

long prevTime = 0;        // will store last time reading was taken
long interval = 10;         // interval at which to taken readings (milliseconds) 10ms = 100Hz

long runTime = 10;     // time to run in s

int switchPin = 11;   // pin for button
int buttonState;
int State;

void setup() {

  Wire.begin();       // for communicating with accelerometer
  Serial.begin(9600); // start serial output - speed is in bits/second (for writing to serial monitor)

  pinMode(switchPin, INPUT);
  State = 0;

  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW

  unsigned long prevTime = 0;

  // Set the time and date
  // RTC.setTime(hours, minutes, seconds);
  // RTC.setDate(day, month, year);

  ///// ACCELEROMETER /////

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);

  // Calibrate gyro and accelerometers, load biases in bias registers
  myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
  Serial.println("MPU9250 calibrated");

  // Initialize device for active mode read of acclerometer, gyroscope, and temperature
  myIMU.initMPU9250();

  Serial.println("MPU9250 initialized for active data mode....");

}

void loop() {

  buttonState = digitalRead(switchPin);
  Serial.print("button state = ");
  Serial.println(buttonState);
  Serial.print("state = ");
  Serial.println(State);

  if (buttonState == 0)
  {
    delay(1000);  // wait one second
    Serial.print("button pressed");

    if (State == 0)
    {
      Serial.print("changing state to 255");
      State = 255;
      ///// SD CARD /////

      // see if the SD card is present and can be initialized:
      if (!SD.begin(cardSelect)) {
        Serial.println("Card init. failed!");
      }

      // create a file on SD card to save the data in
      char filename[15];
      strcpy(filename, "ANALOG00.csv");
      for (uint8_t i = 0; i < 100; i++) {
        filename[6] = '0' + i / 10;
        filename[7] = '0' + i % 10;
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

    }
    else
    {
      Serial.print("changing state to 0");
      digitalWrite(13, LOW);   // turn the LED on (HIGH is the voltage level)
      State = 0;
    }
  }
  else
  {
    Serial.println("Not pressed");
  }


  if (State > 0)
  {

    Serial.println("Recording");
    unsigned long currentTime = millis();
    Serial.print("Current time = "); Serial.println(currentTime);
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)

    // check if its time to take a reading
    if (currentTime - prevTime > interval)   {

      myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values  (accel count is a vector with three values)
      myIMU.getAres();                        // get sensor resolution (reference with myIMU.aRes)
      Serial.println("Sensor resolution: ");   // print to serial terminal
      Serial.println(myIMU.aRes, 5);
      Serial.print("X bias = "); Serial.println(myIMU.accelBias[0], 5);
      Serial.print("Y bias = "); Serial.println(myIMU.accelBias[1], 5);
      Serial.print("Z bias = "); Serial.println(myIMU.accelBias[2], 5);

      Serial.print("X original = "); Serial.println(myIMU.accelCount[0]);
      Serial.print("Y original = "); Serial.println(myIMU.accelCount[1]);
      Serial.print("Z original = "); Serial.println(myIMU.accelCount[2]);

      myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes - myIMU.accelBias[0]; //
      myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes - myIMU.accelBias[1]; //
      myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes - myIMU.accelBias[2]; //

      logfile.print(myIMU.ax, 8);     // print x value to SD card
      logfile.print(",");   // print comma in between each reading
      logfile.print(myIMU.ay, 8);      // print y value to SD card
      logfile.print(",");   // print comma in between each reading
      logfile.print(myIMU.az, 8);      // print z value to SD card
      logfile.print(",");   // print comma in between each reading
      logfile.flush();

      // print values to serial window
      Serial.print("X value = "); Serial.println(myIMU.ax, 5);
      Serial.print("Y value = "); Serial.println(myIMU.ay, 5);
      Serial.print("Z value = "); Serial.println(myIMU.az, 5);
      Serial.println(",");

      prevTime = currentTime;   // save last time reading was taken
      Serial.print("Previous time = "); Serial.println(prevTime);
    }

  }

}

