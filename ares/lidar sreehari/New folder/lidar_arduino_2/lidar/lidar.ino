/* LidarLite v2 laser mounted on 3d printed chassis

  Copyright (c) 2015 Alexander Grau
  Private-use only! (you need to ask for a commercial-use)

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  Private-use only! (you need to ask for a commercial-use)
*/

//
// serial output:
// [0] angle (degree) high byte
// [1] angle (degree) low byte
// [3] distance (cm) high byte
// [4] distance (cm) low byte
// ...repeats
//
// ...until zero pos indication:
// 0xCC, 0xDD, 0xEE, 0xFF


// Arduino Nano wiring:
// A5  - Lidar SCL
// A4  - Lidar SDA
// D4  - Lidar Mode (Add Pull-Down!)  NOT USED!
// D3  - disc encoder pos
// D5  - disc encoder LED (Active Pull-Down )  NOT USED!


// diagnostic mode: open Arduino IDE serial console (CLTR+SHIFT+M), and send 'd' to the Lidar
// diagnostic output should be like this:
// time=239  freq=3  angleMin=0  angleMax=363
// time=240  freq=3  angleMin=1  angleMax=363
// time=241  freq=3  angleMin=1  angleMax=364

#include <Arduino.h>
#include <Wire.h>
#include "LIDARLite.h"
#include "TimerOne.h"
//#include <MsTimer2.h>
//#include <Servo.h>


int pinLidarMode   = 4;
int pinEncoderPos  = 3;
int pinEncoderLED  = 5;
int pinLED  = 13;


#define MODE_NORMAL  0
#define MODE_DEBUG   1
#define MODE_LASER   2

char mode = MODE_NORMAL;
LIDARLite myLidarLite;
volatile int angle = 0;
volatile float ratio = 0;
volatile bool zeroPos = false;
volatile int tickLowDuration = 0;
volatile int tickHighDuration = 0;
volatile int tickAngle = 0;
volatile unsigned long tickLastTime = 0;
int measurements = 0;
unsigned long nextSerialTime = 0;
unsigned long nextInfoTime = 0;
unsigned long rotationCounter = 0;
int angleMax = 0;
int angleMin = 30000;

volatile int tthresh = 500;
volatile int t = 0;
volatile int tickDuration = 0;
volatile unsigned long ms = 0;
volatile float ratiothresh = 5;
volatile int prems = 0;
volatile int pre_trip = 0;
volatile int zeromil = 0;
volatile int zeromilmax = 3000;
volatile int zeromilmin = 1000;
int prev_angle = 0;
bool pk = 1;
//Servo myservo;

int distance = 0;    // Distance measured
int reading = 0;
long dT;
long tNew, tOld;
int motorPos;
volatile int motorStepCnt = 0;
int steps;
float secondsPerDegree = 0;
int lastMotorPos = -1;
int lastSpeed = 0;
volatile float x = 0.0;

void serialPrintRange(int motorPos, int reading);
void calculate_speed( int p_position, int p_interval);

//
//void timer(){ //for 180
//  //angle++; // set to next angle step
//  if(pk==1)
//  {
//    angle++;
//    if(angle>=179)
//    {
//      pk=!pk;
//    }
//  }
//  else
//  {
//    angle--;
//    if(angle<=1)
//    {
//      pk=!pk;
//      zeroPos=true;
//    }
//  }
//  {
//    myservo.write(angle);
//  }
//}


void timer() { // for 360
  angle++; // set to next angle step

  if (angle >= 359)
  {
    angle = 0;
    //zeroPos=true;
  }
  //  myservo.write(angle);
}


//void encoderTick(){
//  volatile int state = digitalRead(pinEncoderPos);
//  volatile unsigned long ms = millis();
//  //digitalWrite(pinLED, state);
//  volatile int tickDuration = ms-tickLastTime;   // compute transition time
//  if (tickDuration == 0) tickDuration = 1;
//  tickAngle+=12;
//  if (state == LOW){
//     high->low transition
//    tickHighDuration = tickDuration;   // remember high time
//    ratio = ((float)tickLowDuration)/((float)tickHighDuration);
//    if (ratio < 0.66){
//      // zero pos
//      zeroPos = true;
//      tickAngle = 0;     // reset angle to zero
//      angle = tickAngle;
//    } else {
//      angle = tickAngle;
//      Timer1.setPeriod(tickDuration*1000/12); // set degree timer based on transition time
//    }
//  } else {
//    // low->high transition
//    tickLowDuration = tickDuration;   // remember low time
//  }
//  tickLastTime = ms; // remember transition time
//}


//void encoderTickRise()
//{
//  digitalWrite(pinLED, HIGH);
//  ms = millis();
//  tickDuration = ms - tickLastTime;
//  if (tickDuration == 0) tickDuration = 1;
//  tickLowDuration = tickDuration;
//  Serial.print("tickLowDuration - ");
//  Serial.println(tickHighDuration);
//  tickLastTime = ms;
//  digitalWrite(pinLED, LOW);
//  attachInterrupt(digitalPinToInterrupt(pinEncoderPos), encoderTickFall, FALLING);
//}
//void encoderTickFall()
//{
//  digitalWrite(pinLED, HIGH);
//  ms = millis();
//  tickDuration = ms - tickLastTime;
//  if (tickDuration == 0) tickDuration = 1;
//  tickHighDuration = tickDuration;
//  Serial.print("tickHighDuration - ");
//  Serial.println(tickHighDuration);
//  ratio = ((float)tickLowDuration) / ((float)tickHighDuration);
//  Serial.print("ratio - ");
//  Serial.println(ratio);
//  if (ratio < ratiothresh)
//  {
//    zeromil = ms - prems;
//    if (zeromil == 0) zeromil = 1;
//    prems = ms;
//    tickAngle = 0;
//    angle = tickAngle;
//    pre_trip = 0;
//    if ( (zeromil < zeromilmax) && (zeromil > zeromilmin) )
//    {
//      Timer1.setPeriod((float)zeromil * 1000.0 / 360.0 / 1.0);
//      if (mode == MODE_DEBUG)
//      {
//        x = (float)zeromil * 1000.0 / 360.0 / 1.0 ;
//        Serial.println();
//        Serial.println();
//        Serial.print((float)zeromil * 1000.0 / 360.0 / 1.0);
//        Serial.print("    -    ");
//        Serial.print(zeromil);
//        Serial.print("    -    ");
//        Serial.print(360.0 / (float)zeromil);
//        Serial.println();
//        Serial.println();
//      }
//    }
//  }
//  else
//  {
//    //angle = tickAngle;
//    pre_trip += 90;
//    angle = pre_trip;
//  }
//  tickLastTime = ms;
//  digitalWrite(pinLED, LOW);
//  attachInterrupt(digitalPinToInterrupt(pinEncoderPos), encoderTickRise, RISING);
//}


void encoderTick() {
  volatile int state = digitalRead(pinEncoderPos);
  volatile unsigned long ms = millis();
  digitalWrite(pinLED, HIGH);
  volatile int tickDuration = ms - tickLastTime;
  if (tickDuration == 0) tickDuration = 1;
  if (state == LOW)
  {
    //    Serial.println("h 2 low");
    //     high->low transition
    //    Serial.println();
    //    Serial.println("*******************************************************************************************************************");
    //tickAngle += 90;
    tickHighDuration = tickDuration;
    ratio = ((float)tickLowDuration) / ((float)tickHighDuration);
    //    Serial.print("ratio - ");
    //    Serial.println(ratio);
    //    Serial.println();
    //    Serial.print("ratio = ");
    //    Serial.println(ratio);
    if (ratio < ratiothresh)
    {
      //zeroPos = false;
      //      Serial.println();
      //      Serial.println("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
      zeromil = ms - prems;
      if (zeromil == 0) zeromil = 1;
      prems = ms;
      tickAngle = 0;
      angle = tickAngle;
      pre_trip = 0;
      if ( (zeromil < zeromilmax) && (zeromil > zeromilmin) )
      {
        Timer1.setPeriod((float)zeromil * 1000.0 / 360.0 / 1.0);
        if (mode == MODE_DEBUG)
        {
          Serial.println();
          Serial.println();
          Serial.print((float)zeromil * 1000.0 / 360.0 / 1.0);
          Serial.print("    -    ");
          Serial.print(zeromil);
          Serial.print("    -    ");
          Serial.print(360.0 / (float)zeromil);
          Serial.println();
          Serial.println();
        }
      }
      //      Serial.print("zeromil - ");
      //      Serial.println(zeromil);

    }
    else
    {
      //angle = tickAngle;
      pre_trip += 90;
      angle = pre_trip;
    }
  }
  else
  {

    //    Serial.println("l 2 high");
    //    low->high transition

    //    Serial.println();
    //    Serial.println("---------------------------------------------------------------------------------------------------------------");
    tickLowDuration = tickDuration;
  }
  tickLastTime = ms;
  digitalWrite(pinLED, LOW);
}

void setup() {

  //myLidarLite.begin();
  pinMode(pinLidarMode, INPUT);
  pinMode(pinEncoderPos, INPUT);
  pinMode(pinEncoderLED, INPUT);
  pinMode(pinLED, OUTPUT);

  // begin(int configuration, bool fasti2c, bool showErrorReporting, char LidarLiteI2cAddress)
  myLidarLite.begin(1, true);
  myLidarLite.beginContinuous();

  Serial.begin(115200);

  Timer1.initialize(1000000); // 1 second
  Timer1.attachInterrupt(timer);

  //MsTimer2::set(5, timer); // 500ms period
  //MsTimer2::start();
  //myservo.attach(5);
  //delay(100);
  //myservo.write(90);

  Serial.println("Ready");
  attachInterrupt(digitalPinToInterrupt(pinEncoderPos), encoderTick, CHANGE);
  pinMode(4, OUTPUT);
}



void loop() {

  t = analogRead(A0);
  if (t > tthresh)
  {
    digitalWrite(4, HIGH);
  }
  else
  {
    digitalWrite(4, LOW);
  }
  //  Serial.println(t);

  if (millis() >= nextSerialTime) {
    nextSerialTime = millis() + 500;
    if (Serial.available()) {
      char ch = Serial.read();
      if (ch == 'd') mode = MODE_DEBUG;
      if (ch == 'n') mode = MODE_NORMAL;
      if (ch == 'l') mode = MODE_LASER;
    }
  }

  int d = 0;
  //d = myLidarLite.distance();
  //while (digitalRead(pinLidarMode) == HIGH);
  d = myLidarLite.distanceContinuous();

  //  int s = myLidarLite.signalStrength();
  /*if (v == 0){
    // take 1 reading with preamp stabilization and reference pulse (these default to true)
    d = myLidarLite.distance();
    } else {
    // take reading without preamp stabilization and reference pulse (these read about 0.5-0.75ms faster than with)
    d = myLidarLite.distance(false,false);
    } */


  int v = angle;
  //  Serial.println(v);
  //  if (zeroPos) {
  //    // zero point
  //    zeroPos = false;
  //    if (mode == MODE_NORMAL) {
  //      /*// fill-up to 360 measurements
  //        while (measurements < 360){
  //        Serial.write(0);
  //        Serial.write(0);
  //        Serial.write(0);
  //        Serial.write(0);
  //        Serial.flush();
  //        measurements++;
  //        }*/
  //      // send zero sync
  //      Serial.write(0xCC);
  //      Serial.write(0xDD);
  //      Serial.write(0xEE);
  //      Serial.write(0xFF);
  //      Serial.flush();
  //    } else {
  //      //Serial.println(rotationCounter);
  //      //Serial.flush();
  //    }
  //    measurements = 0;
  //    rotationCounter++;
  //  }

  if (mode == MODE_NORMAL) {
    if (prev_angle != angle) {
      /*
        Serial.write(v >> 8);
        Serial.write(v & 0xFF);
        Serial.write(d >> 8);
        Serial.write(d & 0xFF);
        Serial.flush();
      */
      serialPrintRange(v, d);
      prev_angle = angle;
    }
  }
  else if (mode == MODE_DEBUG) {

    //    if (angle > angleMax) angleMax = angle;
    //    if (angle < angleMin) angleMin = angle;
    //
    //    if (millis() >= nextInfoTime) {
    //      nextInfoTime = millis() + 1000;
    //      Serial.print("time=");
    //      Serial.print(millis() / 1000);
    //      Serial.print("  freq=");
    //      Serial.print(rotationCounter);
    //      Serial.print("  angleMin=");
    //      Serial.print(angleMin);
    //      Serial.print("  angleMax=");
    //      Serial.print(angleMax);
    //      Serial.println();
    //      angleMax = 0;
    //      angleMin = 30000;
    //      rotationCounter = 0;

    /*Serial.print("idx=");
      Serial.print(measurements);
      Serial.print("  angle=");
      Serial.print(v);
      Serial.print("  distance(cm)=");
      Serial.println(d);  */
    //Serial.print("  strength=");
    //Serial.println(s);  */

    //Serial.println();
  }
  else if (mode == MODE_LASER)
  {
    detachInterrupt(digitalPinToInterrupt(pinEncoderPos));
    Serial.println(digitalRead(pinEncoderPos));
  }
  measurements++;
}


void serialPrintRange(int motorPos, int reading)
{
  //tNew = millis();
  //dT = (tNew - tOld);
  //if ( reading < 1024 && reading > 0 )
  {
    //motorPos = (360.0 / stepsPerRevolution) * motorStepCnt;

    //calculate_speed( motorPos, dT );

    Serial.print(motorPos);
    Serial.print(",");
    Serial.print(reading);
    //    Serial.print(",");
    //    Serial.print(secondsPerDegree, 3);
    //    Serial.print(",");
    //    Serial.print(dT);
    Serial.println();
  }
  //tOld = tNew;
}

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



