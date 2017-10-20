

#include <SoftwareServo.h>
#include <TimerOne.h>
#include <Wire.h>
#include <LIDARLite.h>
 
#define DIR_PIN 2
#define STEP_PIN 3
 
LIDARLite myLidarLite;
SoftwareServo servo;
 
volatile int stepCount = 0;
volatile int roundCount = 0;
 
int height = 18;
 
void setup() {
    Serial.begin(115200);
 
    myLidarLite.begin(0, true);
    myLidarLite.configure(0);
 
    servo.attach(5);
    servo.write(height);
 
    pinMode(DIR_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
 
    digitalWrite(DIR_PIN, HIGH);
 
    Timer1.initialize();
    Timer1.attachInterrupt(tick, 1000000.0f / (3200 * 1));
}
 
void loop() {
    receive(true);
 
    for(int i = 0; i < 99; i++)
        receive(false);
}
 
void receive(bool bias)
{
    float angle = ((float)stepCount / 3200.0f) * 360.0f;
    Serial.print(108 - height);
    Serial.print(",");
    Serial.print(angle);
    Serial.print(",");
    Serial.println(myLidarLite.distance(bias));
 
    if(roundCount >= 2)
    {
        height++;
 
        if(height >= 140)
            height = 18;
 
        servo.write(height);
        roundCount = 0;
    }
 
    servo.refresh();
}
 
void tick()
{
    digitalWrite(STEP_PIN, HIGH);
    //delay(1);
    digitalWrite(STEP_PIN, LOW);
 
    stepCount++;
    if(stepCount >= 3200)
    {
        stepCount = 0;
        roundCount++;
    }
}
