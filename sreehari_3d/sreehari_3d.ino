
#include <Stepper.h>
#include <Servo.h>
#include "LIDARLite.h"

int reading;
int prev_v_angle = 0;
int v_angle = 0;
int prev_h_angle = 0;
int h_angle = 0;

bool clockwise = false;
bool ROS = false;
int i, new_data;

int minPosX = 0;
int maxPosX = 360;
int minPosY = 0;
int maxPosY = 180;
int radius, posX, posY;
float pi = 3.14159265;
float deg2rad = pi / 180.0;

#define motorSteps 200 // change this to number of steps per revolution
#define motorPin1 8
#define motorPin2 9

Stepper mystepper(motorSteps, motorPin1, motorPin2);
Servo myservo;
LIDARLite myLidarLite;

void test_motors()
{
  for (i = 0; i <= 180; i++)
  {
    myservo.write(i);
  }
  for (i; i >= 0; i--)
  {
    myservo.write(i);
  }
  myservo.write(0);

  for (i = 0; i <= 360; i++)
    mystepper.step(1);

  mystepper.step(360);    //is this the right way to make 360 rotation


}

void motors_move()
{
  mystepper.step(1);
  h_angle++;
  if (h_angle >= 360)
  {
    h_angle = h_angle % 360;
    if (clockwise == false)
    {
      v_angle++;
      if (v_angle >= 180)
      {
        v_angle = 180;
        clockwise = 1;
      }
    }
    else
    {
      v_angle--;
      if (v_angle <= 0)
      {
        v_angle = 0;
        clockwise = 0;
      }
    }
  }
  myservo.write(v_angle);
}

void setup()
{

  mystepper.setSpeed(60);         //Speed of stepper-needs optimizing
  myservo.attach(9);

  myLidarLite.begin(1, true);
  myLidarLite.beginContinuous();

  Serial.begin(115200);

  test_motors();

}

void loop()
{

  reading = myLidarLite.distanceContinuous();
  
  if (prev_h_angle != h_angle)
  {
    prev_h_angle = h_angle;
    new_data = 1;
  }
  else if (prev_v_angle != v_angle)
  {
    prev_h_angle = h_angle;
    new_data = 1;
  }

  if (new_data == 1)
  {
    new_data = 0;
    if (ROS == false)
    {
      radius = reading;
      posX = h_angle;
      posY = v_angle;

      float azimuth = posX * deg2rad;
      float elevation = (180 - maxPosY + posY) * deg2rad;
      double x = radius * sin(elevation) * cos(azimuth);
      double y = radius * sin(elevation) * sin(azimuth);
      double z = radius * cos(elevation);
      Serial.println(String(-x, 5) + " " + String(y, 5) + " " + String(-z, 5));
    }
    else
    {
      serialPrintRange(v_angle, h_angle, reading);
    }
  }
  motors_move();
}

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

  Set the no of steps per revolution
  Find the optimum speed
  Change the method to do parallel move and measure
  Use Spherical Coordinate System for ROS as well
  Use xbee.h

*/
