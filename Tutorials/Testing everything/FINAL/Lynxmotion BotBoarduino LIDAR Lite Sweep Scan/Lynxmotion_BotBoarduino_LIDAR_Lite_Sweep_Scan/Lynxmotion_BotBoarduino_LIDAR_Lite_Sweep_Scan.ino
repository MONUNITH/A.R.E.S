/***********************************************************************************************************
/* Lynxmotion BotBoarduino LIDAR Lite Sweep Scan
/* Version: 1.0.0
/* By: scharette
/* Date: 2015-03-31
/* 
/* Short example based on the Servo library example "Sweep" example and the Pulsed Light 3D examples.
/* The servo motor* is attached to pin 5 and rotates from 0-180 degrees continuously by 1 degree increments.
/* The LIDAR lite is attached to VCC (+5 V DC), GND, SDA and SCL.
/* 
/* This examples uses the Wire library.
/*
/* Hardware used:
/* BotBoarduino: http://www.robotshop.com/en/lynxmotion-botboarduino-robot-controller.html?product-code=RB-Lyn-363
/* LIDAR-Lite Laser Rangefinder: http://www.robotshop.com/en/lidar-lite-laser-rangefinder-pulsedlight.html?product-code=RB-Pli-01
/* HS-645MG servo motor: http://www.robotshop.com/en/hitec-hs645mg-servo-motor.html?product-code=RB-Hit-29
/* (alternative) HS-422 servo motor: http://www.robotshop.com/en/hitec-hs422-servo-motor.html?product-code=RB-Hit-27
/* Bracket to hold the LIDAR: http://www.robotshop.com/en/aluminum-l-connector-bracket-asb-06.html?product-code=RB-Lyn-83
/* or this one: http://www.robotshop.com/en/lynxmotion-multi-purpose-sensor-housing.html?product-code=RB-Lyn-75
/* Bracket to hold the servo motor: http://www.robotshop.com/en/lynxmotion-aluminum-multi-purpose-servo.html?product-code=RB-Lyn-81
/*
/* *: Please note that while we used a HS-645MG in our example, the HS-422 and almost any other standard RC servo motor could perform this task.
/*
/**********************************************************************************************************/

#include <Wire.h>

#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

#include <Servo.h>

Servo myservo;

      // Position of the servo (degress, [0, 180])
int distance = 0;    // Distance measured
int reading = 0;
long dT;
long tNew, tOld;
int motorPos;
volatile int motorStepCnt = 0;
int stepsPerRevolution = 1599;
 
void setup()
{
  // Serial output
  Serial.begin(115200);
  Serial.println("< START >");
  
  // Servo control
  myservo.attach(9);
  
  // LIDAR control
  Wire.begin(); // join i2c bus
}

// Get a measurement from the LIDAR Lite

float secondsPerDegree = 0;
int lastMotorPos = -1;
int lastSpeed = 0;
void calculate_speed( int p_position, int p_interval )
{
  int degreesTraveled;
  if ( p_position > lastMotorPos )
  {
   degreesTraveled = p_position - lastMotorPos; 
  }
  else
  {
    degreesTraveled = p_position - lastMotorPos + 360;
  }
  lastMotorPos = p_position;
  float secsPerDeg = ( p_interval / 1000.0/*milliseconds*/ ) / degreesTraveled;
  secondsPerDegree = secsPerDeg;//( 0.1 * secsPerDeg ) ;//+ ( 0.9 * secondsPerDegree );
}

int lidarGetRange(void)
{
  int val = -1;
  
  Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
  Wire.write((int)RegisterMeasure); // sets register pointer to  (0x00)  
  Wire.write((int)MeasureValue); // sets register pointer to  (0x00)  
  Wire.endTransmission(); // stop transmitting

  delay(5); // Wait 20ms for transmit

  Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
  Wire.write((int)RegisterHighLowB); // sets register pointer to (0x8f)
  Wire.endTransmission(); // stop transmitting

  delay(5); // Wait 20ms for transmit
  
  Wire.requestFrom((int)LIDARLite_ADDRESS, 2); // request 2 bytes from LIDAR-Lite

  if(2 <= Wire.available()) // if two bytes were received
  {
    val = Wire.read(); // receive high byte (overwrites previous reading)
    val = val << 8; // shift high byte to be high 8 bits
    val |= Wire.read(); // receive low byte as lower 8 bits
  }
  
  return val;
}

void serialPrintRange(int motorPos, int reading)
{
    tNew = millis();
    dT = (tNew - tOld);
    if ( reading < 1024 && reading > 0 )
    {
        //motorPos = (360.0 / stepsPerRevolution) * motorStepCnt;
        
        calculate_speed( motorPos, dT );

        Serial.print(motorPos);
        Serial.print(",");
        Serial.print(reading);
        Serial.print(",");
        Serial.print(secondsPerDegree,3);
        Serial.print(",");
        Serial.print(dT);
        Serial.println();
    }
    tOld=tNew;
}

void loop()
{
  for(motorPos = 0; motorPos <= 361; motorPos += 1)
  {
    motorPos%=360;
    myservo.write(motorPos);
    distance = lidarGetRange();
    serialPrintRange(motorPos, distance);
    delay(5);
  }
  for(motorPos = 180; motorPos >= 0; motorPos -= 1)
  {
    myservo.write(motorPos);
    distance = lidarGetRange();
    serialPrintRange(motorPos, distance);
    delay(5);
  }
}

