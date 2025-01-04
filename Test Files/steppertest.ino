/* Example sketch to control a stepper motor with 
   DRV8825 stepper motor driver, AccelStepper library 
   and Arduino: continuous rotation. 
   More info: https://www.makerguides.com */

#include "AccelStepper.h"

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
#define dirPin 8
#define stepPin 9
#define motorInterfaceType 1

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

void setup() {
  // Set the maximum speed in steps per second:
  stepper.setMaxSpeed(2000);
  stepper.setSpeed(500);
  stepper.setCurrentPosition(0);
  stepper.setAcceleration(1000);
}

void loop() {
  // Set the speed in steps per second:

  stepper.move(500);
  // Step the motor with a constant speed as set by setSpeed():
  stepper.runToPosition();

  stepper.moveTo(-500);
  // Step the motor with a constant speed as set by setSpeed():
  stepper.runToPosition();

}