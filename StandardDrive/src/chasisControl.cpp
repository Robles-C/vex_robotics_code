#include "chasisControl.h"

autonomousControl::autonomousControl(robotChasis *robot, odometry *tr){
  robot1 = robot;
  tracking = tr;
}

void autonomousControl::setPIDConstants(float drivekP, float drivekI, float drivekD, int driveCap,
                                        float turnkP, float turnkI, float turnkD, int turnCap){
  drivePID.kP = drivekP; drivePID.kI = drivekI; drivePID.kD = drivekD; drivePID.cap = driveCap;
  turnPID.kP = turnkP; turnPID.kI = turnkI; turnPID.kD = turnkD; turnPID.cap = turnCap;                         
}

void autonomousControl::moveDrive(float vDrive, float vTurn){ // Hint use spin
}

float autonomousControl::averageRPM(){
  return (fabs(robot1->frontRight.velocity(rpm)) + fabs(robot1->frontLeft.velocity(rpm)) + fabs(robot1->backRight.velocity(rpm)) + fabs(robot1->backLeft.velocity(rpm)))/4;
}

float autonomousControl::updatePID(PIDSettings *good){
  good->error = good->curr - good->target;
  good->derivative = good->error - good->prevError;
  good->totalError = good->totalError + good->error;

  if((good->totalError*good->kI)>good->cap) good->totalError = good->cap/good->kI;
  else if((good->totalError*good->kI)<-good->cap)good->totalError = -good->cap/good->kI;

  if(std::signbit(good->error) != std::signbit(good->prevError)) good->totalError = 0;

  good->prevError = good->error;
  return -(good->kP*good->error + good->kD*good->derivative + good->kI*good->totalError);
}

int autonomousControl::turnCap(float distanceMag){
  if(distanceMag>30.0) return 2000;
  else if(distanceMag>10.0) return 6000;
  else return 10000;
}

void autonomousControl::movAB(){
  updateCurrPos(); 
  if(fabs(turnPID.curr - turnPID.target) > 5){
  
  } else {

  }

}

void autonomousControl::updateTargetPos(float x, float y){
  rightEncoder = robot1->rightTracker.position(deg);
  leftEncoder = robot1->leftTracker.position(deg);
  turnPID.target = atan((y-tracking->getYPos())/(x-tracking->getXPos()));
}


void autonomousControl::updateOrientation(float degTarget){
  turnPID.target = degTarget;
}

void autonomousControl::updateIntakePct(int pow){ intakePct = pow; }


void autonomousControl::intakeMove(){
  robot1->leftIntake.spin(fwd, intakePct, pct);
  robot1->rightIntake.spin(fwd, -intakePct, pct);
}


void autonomousControl::waitUntilSettled(){
  wait(100, msec);
  while(averageRPM() != 0){
    wait(20, msec);
  }
}

void autonomousControl::waitUntilDistance(float dis){
  wait(100, msec);
  while(dis < vMag){
    wait(20, msec);
  }
}


void autonomousControl::waitUntilDeg(float deg){
  wait(100, msec);
  while(deg < fabs(turnPID.curr - turnPID.target) ){
    wait(20, msec);
  }
}

void autonomousControl::strafeVision(){}

void autonomousControl::autoMain(){
  robot1->set_drive_break_type(coast);

  while(true){
    movAB();
    intakeMove();
    task::sleep(20);
  }
}

void autonomousControl::updateCurrPos(){
  drivePID.curr = ((rightEncoder - robot1->rightTracker.position(deg)) + (leftEncoder - robot1->leftIntake.position(deg))/2) * (robot1->getWheelCir()/360);
  turnPID.curr = tracking->getangleD();
}
