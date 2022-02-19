#include "chasisControl.h"

autonomousControl::autonomousControl(robotChasis *robot, odometry *tr){
  robot1 = robot;
  tracking = tr;
}

void autonomousControl::setPIDConstants(float drivekP, float drivekI, float drivekD, int driveCap,
                                        float turnkP, float turnkI, float turnkD, int turnCap){
  drivePID.kP = drivekP; drivePID.kI = drivekI; drivePID.kD = drivekD; drivePID.cap = driveCap;
  turnPID.kP = turnkP; turnPID.kI = turnkI; turnPID.kD = turnkD; turnPID.cap = turnCap;    
  //declaring pid constants                     
}

void autonomousControl::moveDrive(float vDrive, float vTurn){ 
  robot1->LeftRear.spin(fwd, vDrive + vTurn, voltageUnits::mV);
  robot1->LeftFront.spin(fwd, vDrive + vTurn, voltageUnits::mV); 
  robot1->LeftMid.spin(fwd, vDrive + vTurn, voltageUnits::mV);
  robot1->RightRear.spin(fwd, vDrive - vTurn, voltageUnits::mV);
  robot1->RightFront.spin(fwd, vDrive - vTurn, voltageUnits::mV);
  robot1->RightMid.spin(fwd, vDrive - vTurn,voltageUnits::mV);
  //setting power to each motor based on value returned
}

float autonomousControl::averageRPM(){
  return (fabs(robot1->RightFront.velocity(rpm)) + fabs(robot1->LeftFront.velocity(rpm)) + fabs(robot1->RightMid.velocity(rpm)) + fabs(robot1->LeftMid.velocity(rpm))+fabs(robot1->RightRear.velocity(rpm)) + fabs(robot1->LeftRear.velocity(rpm)))/4;
}

float autonomousControl::updatePID(PIDSettings *good){
  good->error = good->curr - good->target; //good-error = E(s) good-curr = Y*(s) good-target(Y(s)
  good->derivative = good->error - good->prevError; //prevError = -e(t)
  good->totalError = good->totalError + good->error;// totalError = H(s) 


  if((good->totalError*good->kI)>good->cap) good->totalError = good->cap/good->kI;
  else if((good->totalError*good->kI)<-good->cap)good->totalError = -good->cap/good->kI;

  if(std::signbit(good->error) != std::signbit(good->prevError)) good->totalError = 0;

  good->prevError = good->error;
  return -(good->kP*good->error + good->kD*good->derivative + good->kI*good->totalError);
}

/*int autonomousControl::turnCapp(float distanceMag){
  if(distanceMag>30.0) return 2000;
  else if(distanceMag>10.0) return 6000;
  else return 10000;
}*/

void autonomousControl::movAB(){
  updateCurrPos(); 

  float turnVoltage, moveVoltage;

  switch(mState) {
  case 1:
    // orinetation
    // have: targetAngle, and currentAngle
    posUpdated = false;
    turnVoltage = updatePID(&turnPID);
    moveVoltage = 0;
    moveDrive(moveVoltage, -turnVoltage);
    if((fabs(tracking->getangleD() - turnPID.target) < 3) && !isMovingD()) mState = 2;

    
    break;
  case 2:
    // calculate and update drive target
    turnVoltage = 0;
    moveVoltage = 0;
    drivePID.target = sqrt(pow(tracking->getXPos() - xTarget,2)+pow(tracking->getYPos() - yTarget, 2));
    mState = 3;
    //
    break;  
  case 3:
    //applying power to the drive by updating move/tunrVoltage
    moveVoltage = updatePID(&drivePID);
    turnVoltage = (turnPID.curr - turnPID.target);
    //calling move drive function with updated volatage values
    moveDrive(moveVoltage, turnVoltage);
    //when position is within .5 inches stop moving 
    if((fabs(drivePID.target - drivePID.curr) < .5) && !isMovingD()) mState = 0;
    // go back to default state
    break;
  default:
    //stay in position. keep orientation and left and right encoders
    moveVoltage = updatePID(&drivePID);
    turnVoltage = updatePID(&turnPID);
    //to fix turn while drive change 0 to compare and turn one side more
    moveDrive(moveVoltage, -turnVoltage);
    if(posUpdated) mState = 1;
    //keep position
    // compare encoder value if different go back. could be done in 3
    break;
}

//printf("TurnVoltage: %f, moveVolatage: %f,  mState: %d, pidTarget: %f, angleD: %f\n", turnVoltage, moveVoltage, mState, turnPID.target, tracking->getangleD());
printf("%f   %f\n", drivePID.target, drivePID.curr);

}

bool autonomousControl::isMovingD(){
  float rightSide, leftSide;
  rightSide = (robot1->RightMid.velocity(rpm) + robot1->RightFront.velocity(rpm)+robot1->RightRear.velocity(rpm))/3;
  leftSide = (robot1->LeftMid.velocity(rpm) + robot1->LeftFront.velocity(rpm)+robot1->LeftRear.velocity(rpm))/3;

  if((fabs(rightSide) + fabs(leftSide)) < 1.5){
    //no longer moving
    return false;
  }
  return true;
}

void autonomousControl::updateTargetPos(float x, float y){
  rightEncoder = robot1->rightTracker.position(deg);
  leftEncoder = robot1->leftTracker.position(deg);
  if(x-tracking->getXPos() >= 0 ){
    turnPID.target = (atan((y-tracking->getYPos())/(x-tracking->getXPos()))*180/robot1->getPI())-94.160;
  }else{
    turnPID.target = (atan((y-tracking->getYPos())/(x-tracking->getXPos()))*180/robot1->getPI())+94.160;
  }
  // x and y
  xTarget = x;
  yTarget = y;
  posUpdated = true;
}

void autonomousControl::updateOrientation(float degTarget){
  turnPID.target = degTarget;
}

void autonomousControl::updateIntakePct(int pow){ intakePct = pow; }


void autonomousControl::intakeMove(){
  //robot1->leftIntake.spin(fwd, intakePct, pct);
  //robot1->rightIntake.spin(fwd, -intakePct, pct);
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
