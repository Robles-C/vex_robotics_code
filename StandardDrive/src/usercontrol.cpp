#include "usercontrol.h"

userControl::userControl(robotChasis *robot, bool dM){
  robot1 = robot;
  robot1->set_drive_break_type(coast);
}

void userControl::setBrakeMode(){
  if(robot1->Controller1.ButtonB.pressing()) robot1->set_drive_break_type(hold);
  else if(robot1->Controller1.ButtonA.pressing()) robot1->set_drive_break_type(coast);
}

void userControl::driveB(){ // Hint use spin
}

void userControl::driveLoop(){
  while(true){
    setBrakeMode();
    driveB();
    wait(20, msec);
  }
}