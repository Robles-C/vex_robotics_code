#pragma once
#include "robot-config.h"
#include <math.h>
#include <cstdlib>

class userControl{
  public:
    void driveLoop();
    userControl(robotChasis *robot, bool dM);

  private:
    robotChasis *robot1;

    void driveB();
    void setBrakeMode();
};

