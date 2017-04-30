/*  Cameron and Dustin's ECE 198 Camera payload stabilization code for the Sparkfun MPU9250 Inertial Measurement Unit
 *  This code only uses the gyro sensor in the IMU.
 *  To stabilize the payload, we read the angular velocity in degrees per second, and integrate to obtain heading it degrees.
 *  With the heading, we use an algorithm to set the servo to the proper angle to rotate the box back to its initial heading.
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

#include "quaternionFilters.h"
#include "MPU9250.h"
#include <Servo.h>
Servo servo;

#define AHRS false         // Set to false for basic data read

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 13;  // Set up pin 13 led for toggling

// Global vars used in loop() and stabilize().
int count = 0;
int firstTime = 1;
float desiredHeading;
bool jumpedRTL = false;
bool jumpedLTR = false;
float previousHeading;

MPU9250 myIMU;



void setup()
{
  servo.attach(3);
  initIMU();
}



void loop()
{
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions.
  myIMU.updateTime();

  // INTEGRATE TO GET HEADING.

  // STABILIZE


  if (!AHRS)
  {
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 1000)
    {

        // Print gyro values in degree/sec
        Serial.print("GYRO- X: "); Serial.print(myIMU.gx, 3);
        Serial.print(" ");
        Serial.print("Y: "); Serial.print(myIMU.gy, 3);
        Serial.print(" ");
        Serial.print("Z: "); Serial.print(myIMU.gz, 3);
        Serial.println("   deg/s");

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



void stabilize(float heading) {
    int servoReading;
  
  float headingDiff = heading - desiredHeading;
  float change = heading - previousHeading;

  // Cases:
  // JumpedRTL && desiredHeading < 180
  // JumpedRTL && desiredHeading > 180
  // JumpedLTR && desiredHeading < 180
  // JumpedLTR && desiredHeading > 180

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
    // desiredHeading > 180
//    else {
//      headingDiff = desiredHeading - headingDegrees;
//    }
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
      servo.write(90 - headingDiff);
    }
    // If the heading is to the left (between -90 and 0 degrees), move to the right.
    else {
      servo.write(90 + (-headingDiff));
    }
    servoReading = servo.read();
  }

    // Print every second (count is global).
  if ((count % 100) == 0) {
    Serial.print("Heading: "); Serial.println(heading);
    if (jumpedRTL) {
      Serial.println("jumpedRTL TRUE");
    }
    if (jumpedLTR) {
      Serial.println("jumpedLTR TRUE");
    }
//    Serial.print("Heading difference: "); Serial.println(headingDiff);
//    Serial.print("Servo angle: "); Serial.println(servoReading);
    Serial.println("");
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
}

