
#include <Stepper.h>
#include <Servo.h>
#include "LIDARLite.h"

int reading;
int prev_v_angle = 0;
int v_angle = 0;
int prev_h_angle = 0;
float h_angle = 0;

bool clockwise = false;
bool ROS = false;
bool new_data = false;
int i;
int v_angle_start;

int minPosX = 0;
int maxPosX = 360;
int minPosY = 0;
int maxPosY = 180;
int radius, posX, posY;
float pi = 3.14159265;
float deg2rad = pi / 180.0;
int servomax = 100;
int servomin = 15;

int stepperSpeed = 100;
float motorSteps = 456.7; // change this to number of steps per revolution

#define motorPin1 8
#define motorPin2 9
#define motorPin3 10
#define motorPin4 11

float degperstep = 360 / motorSteps;//0.77419354838;

Stepper mystepper(motorSteps, motorPin1, motorPin2, motorPin3, motorPin4);
Servo myservo;
LIDARLite myLidarLite;

/*
 * Testing the motors
 */ 
void test_motors()
{
  /*
  for (i = 0; i <= 465; i++)
  {
    mystepper.step(-1);
//    delay(10);
  }
  
  mystepper.step(-1*motorSteps);    //is this the right way to make 360 rotation
  */
  for (i = servomin; i <= servomax; i++)
  {
    myservo.write(i);
    delay(10);
  }
  for (i; i >= servomin; i--)
  {
    myservo.write(i);
    delay(10);
  }
  myservo.write(servomax);
  v_angle = v_angle_start;
  delay(10);
}

/*
 * Increase horizontal angle
 * If that is 360 increase vertical angle by 1
 */
void motors_move()
{
  mystepper.step(-1);
  h_angle+=degperstep;
  if (h_angle >= 360)
  {
    h_angle = h_angle - 360.0;
    if (clockwise == true)
    {
      v_angle++;
      if (v_angle >= servomax)
      {
        v_angle = servomax;
        clockwise = false;
      }
    }
    else
    {
      v_angle--;
      if (v_angle <= servomin)
      {
        v_angle = servomin;
        clockwise = true;
      }
    }
  }
  myservo.write(v_angle);
}

void setup()
{

  mystepper.setSpeed(stepperSpeed);         //Speed of stepper-needs optimizing
  myservo.attach(5);

  myLidarLite.begin(1, true);
  myLidarLite.beginContinuous();

  Serial.begin(115200);

  test_motors();

  v_angle_start = servomax;

}

void loop()
{

  
//  Serial.print("                                           ");
//  Serial.println(reading);
  
  if (prev_h_angle != h_angle)
  {
    prev_h_angle = h_angle;
    new_data = true;
  }
  else if (prev_v_angle != v_angle)
  {
    prev_v_angle = v_angle;
    new_data = true;
  }

  if (new_data == true)
  {
    reading = myLidarLite.distanceContinuous();
    new_data = false;
    if (ROS == false)
    {
      radius = reading;
      posX = h_angle;
      posY = v_angle;

      float azimuth = posX * deg2rad;
//      float elevation = (180 - maxPosY + posY) * deg2rad;
      float elevation = (posY) * deg2rad;
      double x = radius * sin(elevation) * cos(azimuth);
      double y = radius * sin(elevation) * sin(azimuth);
      double z = radius * cos(elevation);
      Serial.print('\n');
      Serial.print(" "+String(x, 5) + " " + String(y, 5) + " " + String(z, 5)+" ");
      Serial.print('\n');
    }
    else
    {
      serialPrintRange(v_angle, h_angle, reading);
    }
  }
  motors_move();
}

/*
void loop()
{

}
*/

void serialPrintRange(int v_angle, int h_angle, int dist)
{
  {
    Serial.print(v_angle);
    Serial.print(",");
    Serial.print(h_angle);
    Serial.print(",");
    Serial.print(dist);
    Serial.println();
  }
}


/*
  TODO

  Stepper Motor Vibrations... WHY??
  STart condition  for motor
  Set the no of steps per revolution
  Find the optimum speed
  Change the method to do parallel move and measure
  Use Spherical Coordinate System for ROS as well
  Use xbee.h

*/


