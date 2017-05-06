/*   Cameron and Dustin's code to stabilize a HAB payload using an Adafruit HMC5883 magnetometer to get the heading of the box.
 *   NOTE: This code was not replaced by HAB_project_gyro because the strong magnetic field created by the servo prevented us from measuring Earth's magnetic field.
 *   This code contains a more advanced stabilization algorithm from our inital code, because we set the servo to return to the initial heading, no matter what that initial heading was.
 *   
 *   Wiring:
 *   5v - VIN
 *   GND - GND
 *   SDA - A4
 *   SCL - A5
 */

/***************************************************************************
  Arduino code to run a servo based on the HMC5883 magnentometer/compass.
  Note that some of this code is copied from the Adafruit library.

  These displays use I2C to communicate, 2 pins are required to interface.
 ***************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Servo.h>

// Servo setup.
int servoPin = 3;
Servo servo;
// Global vars used in loop() and stabilize().
int count = 0;
int firstTime = 1;
float desiredHeading;
bool jumpedRTL = false;
bool jumpedLTR = false;
float previousHeading;

// Assign a unique ID to this sensor at the same time
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);



void setup(void) 
{
  Serial.begin(9600);
  servo.attach(servoPin);
  
  // Initialise the sensor
  if(!mag.begin())
  {
    // There was a problem detecting the HMC5883, print a message.
    Serial.println("No HMC5883 detected - Check wiring!");
    while(1);
  }
  
  // Display some basic information on this sensor
  displaySensorDetails();

  // Set the servo to 90 degrees (in the middle).
  Serial.println("Setting servo");
  servo.write(0);
  delay(1000);
  servo.write(90);
  Serial.println("Servo set");
}



void loop(void) 
{
  // Get a new sensor event
  sensors_event_t event; 
  mag.getEvent(&event);
  
 
 // Print output every second.
 if ((count % 200) == 0) {
    // Display the results (magnetic vector values are in micro-Tesla (uT))
//    Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
//    Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
//    Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");
 }

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);   // inverseTan(y / x)
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Convert the declination degrees to radians.
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = -0.279;     // RADIANS (Set for 16 degrees, Bngr ME).
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI;

// The first time the loop runs, get the desired heading.
  if (firstTime) {
    desiredHeading = headingDegrees;
    previousHeading = headingDegrees;
    Serial.print("INITIAL HEADING: "); Serial.println(desiredHeading);
    firstTime = 0;
    delay(2000);
  }

  // Stabilize the payload based on the current heading and desired heading.
  stabilize(headingDegrees);

  // Get the previous heading to tell how the heading is changing.
  previousHeading = headingDegrees;
  
  delay(5);
  count++;
}


// A complex stabilization algorithm.
// The complication is caused by "jumping" from 359 degrees to 0 degrees, which would make our payload think it changed direction by 359 degrees, when it only moved by 1 degree.
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

// If we've jumped from right to left and our desiredHeading is on the right, calculate the difference in heading using the "slicing" method.
  if (jumpedRTL) {
    if (desiredHeading < 180) {
      headingDiff = -((360 - heading) + desiredHeading);
    }
    // If desiredHeading >= 180, use the original headingDiff.
  }

// If we've jumped from left to right and our desiredHeading is on the left, calculate the difference in heading using the "slicing" method.
  if (jumpedLTR) {
    if (desiredHeading >= 180) {
      headingDiff = 360 - desiredHeading + heading;
    }
    // If desiredHeading < 180, use the original headingDiff.
  }

    // If our heading is within 90 degrees of the desired heading, set the servo appropriately.
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
    // Diagnostic jump prints.
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

void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

