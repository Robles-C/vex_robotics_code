#include "usercontrol.h"
#include "visionUno.h"

userControl::userControl(robotChasis *robot, bool dM) {
  robot1 = robot;
  robot1->set_drive_break_type(coast);
}

void userControl::setBrakeMode() {
  if (robot1->Controller1.ButtonB.pressing())
    robot1->set_drive_break_type(hold);
  else if (robot1->Controller1.ButtonA.pressing())
    robot1->set_drive_break_type(coast);
}

void userControl::driveB() {
  robot1->LeftRear.spin(fwd,robot1->Controller1.Axis3.value(), velocityUnits::pct);
  robot1->LeftFront.spin(fwd, (robot1->Controller1.Axis3.value()), velocityUnits::pct); 
  robot1->LeftMid.spin(fwd, (robot1->Controller1.Axis3.value()), velocityUnits::pct);
  robot1->RightRear.spin(fwd, robot1->Controller1.Axis2.value(), velocityUnits::pct);
  robot1->RightFront.spin(fwd, (robot1->Controller1.Axis2.value()), velocityUnits::pct);
  robot1->RightMid.spin(fwd, (robot1->Controller1.Axis2.value()),velocityUnits::pct);
}

void userControl::turnUntil() {
int x = 0;
int center = 158;// The x coordinate for the center of the vision sensor
int OKError = 10; //how close to center you want it

while (true)
{
VisionSenso.takeSnapshot(BLUEMG);
if (VisionSenso.largestObject.exists){
    //Not supposed to happen
}else if(!VisionSenso.largestObject.exists){ 
    VisionSenso.takeSnapshot(BLUEMG);
    while(!VisionSenso.largestObject.exists){
      robot1->LeftFront.spin(fwd,5,pct);
      robot1->LeftMid.spin(fwd,5,pct);
      robot1->LeftRear.spin(fwd,5,pct);
      robot1->RightFront.spin(reverse,5,pct);
      robot1->RightMid.spin(reverse,5,pct);
      robot1->RightRear.spin(reverse,5,pct);
      wait(40, msec);
      VisionSenso.takeSnapshot(BLUEMG);
      if(VisionSenso.largestObject.exists){
        break;
      }
    }
    x = VisionSenso.largestObject.centerX; 
    if(x< (center-OKError)){ 
    //If the object is to the right of center
      robot1->LeftFront.spin(reverse,5,pct);
      robot1->LeftMid.spin(reverse,5,pct);
      robot1->LeftRear.spin(reverse,5,pct);
      robot1->RightFront.spin(fwd,5,pct);
      robot1->RightMid.spin(fwd,5,pct);
      robot1->RightRear.spin(fwd,5,pct);
    } 
    else if (x> center + OKError){
      //If the object is to the left of center
      robot1->LeftFront.spin(fwd,5,pct);
      robot1->LeftMid.spin(fwd,5,pct);
      robot1->LeftRear.spin(fwd,5,pct);
      robot1->RightFront.spin(reverse,5,pct);
      robot1->RightMid.spin(reverse,5,pct);
      robot1->RightRear.spin(reverse,5,pct);
    }
    else{ 
      //The object is not to the right of center and not to the left of center
      robot1->LeftFront.spin(reverse,0,pct);
      robot1->LeftMid.spin(reverse,0,pct);
      robot1->LeftRear.spin(reverse,0,pct);
      robot1->RightFront.spin(fwd,0,pct);
      robot1->RightMid.spin(fwd,0,pct);
      robot1->RightRear.spin(fwd,0,pct);
    }
  }
  break;
task::sleep(100);
}
}


void userControl::driveLoop() {
  while (true) {
    setBrakeMode();
    if(robot1->Controller1.ButtonUp.pressing()){
      turnUntil();
    }else {
    driveB();
    }
    wait(20, msec);
  }
}