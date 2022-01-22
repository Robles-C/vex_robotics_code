#pragma once

#include "robot-config.h"
#include "tracking.h"
#include <cmath>

class autonomousControl{
  public:
    void updateTargetPos(float x, float y);
    void updateOrientation(float degTarget);
    void updateIntakePct(int pow);

    void waitUntilDistance(float dis);
    void waitUntilSettled();
    void waitUntilDeg(float deg);

    void autoMain();
    void setPIDConstants(float drivekP, float drivekI, float drivekD, int driveCap,
                         float turnkP, float turnkI, float turnkD, int turnCap);

    autonomousControl(robotChasis *robot, odometry *tr);

  private:
    struct PIDSettings {
      float target; float curr; float error; float prevError;
            float derivative; float totalError;
      float kP, kI, kD;
      int cap;
    };

    PIDSettings drivePID;
    PIDSettings turnPID;
    
    robotChasis *robot1;
    odometry *tracking;

    int intakePct = 0;
    double rightEncoder;
    double leftEncoder;
    double backEncoder;
    double vectorD[2];
    float vMag;
    double angleVoltage;
    

    void moveDrive(float vDrive, float vTurn);
    void odometryMove(bool oMove);
    void turnVision();
    void forwardVision();
    void strafeVision();
    void updateCurrPos();
    float averageRPM();
    float updatePID(PIDSettings *good);
    int turnCap(float distanceMag);
    void movAB();
    void intakeMove();
};
