#include "autonomousRoutine.h"

autonomousRoutine::autonomousRoutine(autonomousControl *autoControl) {
  control = autoControl;
  control->setPIDConstants( 700, 55, 0, 200, 
                            100, 10, 10, 500);
}
//PID constants
//kP,kI,kD, driveCap
//kP,kI,kD, turnCap

void autonomousRoutine::run(int autoSelection) {
  switch(autoSelection){
    case 0:
      test();
      break;
    case 1:
      odometryOnlyAuto();
      break;
    case 2:
      odometryVisionAuto();
      break;
    default:
      printf("No Auto Selected!");
      break;
  }
}

void autonomousRoutine::test(){
  //where you run the program
  control->updateTargetPos(0, 6);
  //control->updateOrientation(90);
  wait(1000, msec);
  control->waitUntilSettled();
}

void autonomousRoutine::odometryOnlyAuto(){
  // First Goal
  /*control->updateIntakePct(-100);
  wait(1000, msec);
  control->updateTargetPos(0, 9);
  control->updateIntakePct(-100);
  control->waitUntilDistance(1);
  control->updateTargetPos(0, 9);
  control->waitUntilDistance(2);
  control->updateIntakePct(0);
  control->updateTargetPos(16, 10);
  control->waitUntilDistance(2);
  control->updateTargetPos(18, 8);
  control->waitUntilDeg(5);
  control->waitUntilDistance(2);
  wait(1250, msec);
  */
}

void autonomousRoutine::odometryVisionAuto(){
  
}