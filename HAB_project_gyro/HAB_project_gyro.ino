/*  Cameron and Dustin's ECE 198 Camera payload stabilization code for the Sparkfun MPU9250 Inertial Measurement Unit.
 *  This code only uses the gyro sensor in the IMU.
 *  To stabilize the payload, we read the angular velocity in degrees per second, and integrate to obtain heading in degrees.
 *  With the heading, we use an algorithm to set the servo to the proper angle to rotate the box back to its initial heading.
 *  At the time of the end of ECE 198, this code works, but is untested in terms of aerodynamic functionality.
 *  More work is required to make a functional camera stabilizer.
 *  
 *  Note that the sensitivity of the Gyro is set in Arduino/libraries/SparkFun_MPU9250/src/MPU9250.h
 *  The current sensitivity value is 500 degrees per second.
 */
 
// Initial IMU code obtained from:
/* by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you
 find it useful you can buy me a beer some time.
 Modified by Brent Wilkins July 19, 2016

 Demonstrate basic MPU-9250 functionality including parameterizing the register
 addresses, initializing the sensor, getting properly scaled accelerometer,
 gyroscope, and magnetometer data out. Addition of 9 DoF sensor fusion using open source
 Madgwick and Mahony filter algorithms.

 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4 (with 3.3k pull-up to 3.3V)
 SCL ----------------------- A5 (with 3.3k pull-up to 3.3V)
 GND ---------------------- GND
 AD0 ---- GND
 */

#include "MPU9250.h"
#include <Servo.h>
#define AHRS false         // Set to false for basic data read
Servo servo;
MPU9250 myIMU;

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins.
int myLed  = 13;  // Set up pin 13 led for toggling.

// Boolean for when the system is first powered on. The payload will start horizontal and then become vertical.
bool payloadHorizontal = true;
// Boolean used to set previousHeading before stabilizing the first time.
bool firstTimeStabilize = true;

// Variables used in getHeading().
float prev_rate_x = 0;
float heading_x = 0;
unsigned long time_x = millis();
float rate_x;
float prev_rate_z = 0;
float heading_z = 0;
unsigned long time_z = millis();
float rate_z;

// Variables used in stabilize().
// Desired heading is 0 for gyro.
float desiredHeading = 0;
bool jumpedRTL = false;
bool jumpedLTR = false;
float previousHeading_x;
float previousHeading_z;



void setup()
{
  servo.attach(3);
  initIMU();
}


// Globals for loop().
bool baselineTesting = true;
float dc_offset_x = 0;
float dc_offset_z = 0;
float noise_x = 0;
float noise_z = 0;
float noiseTotal_x = 0;
float noiseTotal_z = 0;
// X-axis heading (for horizontal-to-vertical).
float xHeading;
// Z-axis heading (for stabilization).
float zHeading;

void loop()
{ 
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    // Calculate the gyro angular velocity into degrees per second. This depends on scale being set.
    // SCALE IS 500 DPS.
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;
  }

  // Calculate DC offset and noise level of gyro.
  if (baselineTesting) {
    // Take 500 samples.
    for(int n = 0; n < 500; n++) {
      myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
      myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
      myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
      myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

      dc_offset_x += myIMU.gy;
      dc_offset_z += myIMU.gz;
    }
    dc_offset_x /= 500;
    dc_offset_z /= 500;
  
    //print dc offset and noise level
    Serial.print("X DC Offset: "); Serial.println(dc_offset_x, 4);
    Serial.print("Z DC Offset: "); Serial.println(dc_offset_z, 4);

    // Take 500 more samples to get noise values.
    for (int i = 1; i <= 500; i++) {
      myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
      myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
      myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
      myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

      // Add the absolute value of the noise to the total noise.
      if (myIMU.gx - dc_offset_x > 0) {
        noiseTotal_x += (myIMU.gx - dc_offset_x);
      }
      else if (myIMU.gx - dc_offset_x < 0) {
        noiseTotal_x += -(myIMU.gx - dc_offset_x);
      }
    
      if (myIMU.gz - dc_offset_z > 0) {
        noiseTotal_z += (myIMU.gz - dc_offset_z);
      }
      else if (myIMU.gz - dc_offset_z < 0) {
        noiseTotal_z += -(myIMU.gz - dc_offset_z);
      }
    }
    
    noise_x = noiseTotal_x / 500;
    noise_z = noiseTotal_z / 500;
    // Add .07 to x noise and .05 to z noise to keep things static when the box is not moving. This is just manual tuning based on noise we see.
    noise_x += 0.07;
    noise_z += 0.05;
    Serial.print("X Noise: "); Serial.println(noise_x, 4);
    Serial.print("Z Noise: "); Serial.println(noise_z, 4); Serial.println(" ");

    baselineTesting = false;
  }

  // Must be called before updating quaternions.
  myIMU.updateTime();

  // INITIAL: WAIT FOR BOX TO BE VERTICAL, THEN START STABILIZING.
  int count = 0;
  previousHeading_x = getXHeading();
  // While the box is horizontal, get the x-heading and wait for it to be rotated 80 degrees.
  while (payloadHorizontal) {
    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    xHeading = getXHeading();

    if ((count % 1000) == 0) {
      Serial.println("Waiting for the box to be vertical...");
      Serial.print("X-axis heading: "); Serial.println(xHeading);
      Serial.print("X-headingDiff: "); Serial.println(getHeadingDiff(xHeading));
    }

    // Turn off the LED to indicate that baseline testing is done and it's waiting to be vertical.
    digitalWrite(myLed, 0);

    // If the box has rotated 80+ degrees, it's now vertical.
    if (getHeadingDiff(xHeading) > 80 || getHeadingDiff(xHeading) < -80) {
      previousHeading_x = getXHeading();
      payloadHorizontal = false;
      Serial.println("PAYLOAD IS VERTICAL"); Serial.println(" ");
      // Toggle the LED 10 times to indicate that the payload is vertical.
      for (int i = 0; i < 10; i++) {
        digitalWrite(myLed, !digitalRead(myLed));  // toggle led
        delay(100);
        digitalWrite(myLed, !digitalRead(myLed));
        delay(100);
      }
      // Wait 5 seconds before getting a z-heading and beginning stabilization.
      delay(5000);
      break;
    }
    count++;
  }
  
  // INTEGRATE TO GET HEADING on the z axis (yaw).
  zHeading = getZHeading();

  // STABILIZE
  if (firstTimeStabilize) {
    previousHeading_z = zHeading;
    firstTimeStabilize = false;
  }
  // Set the servo to stabilize the payload.
  stabilize(zHeading);
  previousHeading_z = zHeading;

  // Print diagnostics.
  if (!AHRS)
  {
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 1000)
    {
        // Print gyro values in degree/sec
//        Serial.print("GYRO- X: "); Serial.print(myIMU.gx, 3);
//        Serial.print(" ");
//        Serial.print("x: "); Serial.print(myIMU.gy, 3);
//        Serial.print(" ");
        Serial.print("Z: "); Serial.print(myIMU.gz, 3);
        Serial.println("   deg/s");
        Serial.print("Desired Heading: "); Serial.println(desiredHeading);
        Serial.print("Heading: "); Serial.println(zHeading);
        Serial.println(" ");

        myIMU.tempCount = myIMU.readTempData();  // Read the adc values
        // Temperature in degrees Centigrade
        myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
        // Print temperature in degrees Centigrade
//        Serial.print("Temperature is ");  Serial.print(myIMU.temperature, 1);

      // Reset IMU count to current time.
      myIMU.count = millis();
      digitalWrite(myLed, !digitalRead(myLed));  // toggle led
    }
  }
}



// prev_rate_x, heading, time, rate are globals defined above.
// NOTE that variable heading_x is initialized as 0.
float getXHeading(void) {
  // Every 5 ms, take a gyro sample.
  float changeInTime = millis() - time_x;
  if(changeInTime > 5) {
    time_x = millis(); // Update time for the next sample.

    // Make rate negative for clockwise to be positive values.
    rate_x = -(myIMU.gx - dc_offset_x);

    // If the rate of motion is above the noise values, the payload is actually moving.
    if (rate_x >= noise_x || rate_x <= -noise_x) {
      // Trapezoidal method of integration:
      // Average of the two values * time between samples.
      heading_x += ((prev_rate_x + rate_x)/ 2.0) * (changeInTime / 1000);
    }
    prev_rate_x = rate_x;
    
    // Keep the angle between 0-359 degrees:
    if(heading_x < 0){
       heading_x += 360;
    }
    else if(heading_x >= 360){
     heading_x -= 360;
    }
  }
  return heading_x;
}



// prev_rate, heading, time, rate_z are globals defined above.
// NOTE that variable heading is initialized as 0.
float getZHeading(void) {
  // Every 5 ms, take a gyro sample.
  float changeInTime = millis() - time_z;
  if(changeInTime > 5) {
    time_z = millis(); // Update time for the next sample.

    // Make rate negative for clockwise to be positive values.
    rate_z = -(myIMU.gz - dc_offset_z);

    // If the rate of motion is above the noise values, the payload is actually moving.
    if (rate_z >= noise_z || rate_z <= -noise_z) {
      // Trapezoidal method of integration:
      // Average of the two values * time between samples.
      heading_z += ((prev_rate_z + rate_z)/ 2.0) * (changeInTime / 1000);
    }
    
    prev_rate_z = rate_z;
    
    // Keep the angle between 0-359 degrees:
    if(heading_z < 0){
        heading_z += 360;
    }
    else if(heading_z >= 360){
      heading_z -= 360;
    }
  }
  return heading_z;
}



// This is a simplified version of stabilize(), used for getting the change in x-heading.
// This is used to prevent the "jump" from 0-359 issue.
float getHeadingDiff(float heading) {
    
  float headingDiff;

  if (heading < 180) {
    headingDiff = heading;
  }
  else {
    headingDiff = 360 - heading;
  }

  return headingDiff;
}


// Set the servo to stabilize the payload based on the z-heading of the gyro.
void stabilize(float heading) {
  
  float headingDiff = heading - desiredHeading;
  float change = heading - previousHeading_z;

  // Cases:
  // jumpedRTL && desiredHeading < 180
  // jumpedRTL && desiredHeading > 180
  // jumpedLTR && desiredHeading < 180
  // jumpedLTR && desiredHeading > 180

  if (change < -350) {
    Serial.println("JUMPED LTR");
    jumpedLTR = true;
    jumpedRTL = false;
  }
  if (change > 350) {
    Serial.println("JUMPED RTL");
    jumpedRTL = true;
    jumpedLTR = false;
  }

  if (jumpedRTL) {
    if (desiredHeading < 180) {
      headingDiff = -((360 - heading) + desiredHeading);
    }
  }

  if (jumpedLTR) {
    if (desiredHeading < 180) {
      headingDiff = heading - desiredHeading;
    }
    // desiredHeading > 180
    else {
      headingDiff = 360 - desiredHeading + heading;
      
    }
  }

    if (headingDiff >= -90 && headingDiff <= 90) {
    // If the heading is to the right (between 1 and 90 degrees), move to the left.
    if (headingDiff > 0) {
      servo.write(90 - (headingDiff / 2));
    }
    // If the heading is to the left (between -90 and 0 degrees), move to the right.
    else {
      servo.write(90 + ((-headingDiff) / 2));
    }
  }

  // After 1.5 hours, stop stabilizing and make the flaps vertical for the rest of the flight.
  // This is for the descent.
  if (millis() > 5400000) {
    while (1) {
      servo.write(90);
    }
  }
}



void initIMU(void) {
    Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(38400);
  servo.attach(3);

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print(F("MPU9250 I AM 0x"));
  Serial.print(c, HEX);
  Serial.print(F(" I should be 0x"));
  Serial.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    Serial.println(F("MPU9250 is online..."));

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.selfTest);
    Serial.print(F("x-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[3],1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[4],1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 ");
    Serial.print("I AM 0x");
    Serial.print(d, HEX);
    Serial.print(" I should be 0x");
    Serial.println(0x48, HEX);

// For some reason, this if statement causes the program to stop working. My thought is that this statement is for SPI.
//    if (d != 0x48)
//    {
//      // Communication failed, stop here
//      Serial.println(F("Communication failed, abort!"));
//      Serial.flush();
//      abort();
//    }

    // Get sensor resolutions, only need to do this once
    myIMU.getGres();

    delay(2000); // Add delay to see results before serial spew of data

  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }
  Serial.println(""); Serial.println("STARTING LOOP()"); Serial.println("");
}

