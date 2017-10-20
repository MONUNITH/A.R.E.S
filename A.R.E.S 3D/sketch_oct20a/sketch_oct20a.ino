#include <CustomStepper.h>

//Full constructor, just the first 4 parameters are necessary, they are the pins connected to the motor,
//the others are optional, and default to the following below
//the 5th paramater is the steps sequence, where the 1st element of the array is the number of steps
//it can have a maximum of 8 steps
//the 6th parameter is the SPR (Steps Per Rotation)
//the 7th parameter is the RPM
//the 8th parameter is the rotation orientation
CustomStepper stepper(2, 3, 4, 5, (byte[]){8, B1000, B1100, B0100, B0110, B0010, B0011, B0001, B1001}, 4075.7728395, 12, CW);
boolean rotate1 = false;
boolean rotatedeg = false;
boolean crotate = false;

void setup()
{
 //sets the RPM
 stepper.setRPM(12);
 //sets the Steps Per Rotation, in this case it is 64 * the 283712/4455 annoying ger ratio
 //for my motor (it works with float to be able to deal with these non-integer gear ratios)
 stepper.setSPR(4075.7728395);
}

void loop()
{
 //when a command is finished it the isDone will return true, it is important to notice that
 //you can't give one command just after another since they don't block the execution,
 //which allows the program to control multiple motors
 if (stepper.isDone() &&  rotate1 == false)
 {
   //this method sets the direction of rotation, has 3 allowed values (CW, CCW, and STOP)
   //clockwise and counterclockwise for the first 2
   stepper.setDirection(CCW);
   //this method sets the motor to rotate a given number of times, if you don't specify it,
   //the motor will rotate untilyou send another command or set the direction to STOP
   stepper.rotate(2);
   rotate1 = true;
 }
 if (stepper.isDone() && rotate1 == true && rotatedeg == false)
 {
   stepper.setDirection(CW);
   //this method makes the motor rotate a given number of degrees, it works with float
   //you can give angles like 90.5, but you can't give negative values, it rotates to the direction currently set
   stepper.rotateDegrees(90);
   rotatedeg = true;
 }
 if (stepper.isDone() && rotatedeg == true && crotate == false)
 {
   stepper.setDirection(CCW);
