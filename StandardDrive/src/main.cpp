/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Jose Martinez                                             */
/*    Created:      Sat Jul 04 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;
robotChasis robot1 = robotChasis(2.75, 5.65, 5.65, 5.25);
odometry tracker = odometry(&robot1, 0, 0, 0);
autonomousControl autoChasis = autonomousControl(&robot1, &tracker);
autonomousRoutine autoRoutine = autonomousRoutine(&autoChasis);

int trackerWrapper(){
  tracker.updatePosition();
  return 0;
}

int printerWrapper(){
  tracker.updateScreen();
  return 0;
}

int autoWrapper(){
  autoChasis.autoMain();
  return 0;
}

task startTracking(trackerWrapper);
task startPrinting(printerWrapper);
task startAuto(autoWrapper);

void opControl(){                             
  startAuto.stop();
  userControl driveJose = userControl(&robot1, true);
  driveJose.driveLoop();
}

void autonM(){
  robot1.gyroM.resetRotation();
  autoRoutine.run(ODOMETRYONLY);
}

void disabledR(){
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit(&robot1);
  robot1.Comp.drivercontrol(opControl);
  robot1.Comp.autonomous(autonM);

  while(1){
    if(!robot1.Comp.isEnabled()) disabledR();
    wait(100, msec);
  }
}
