#include "tracking.h"


odometry::odometry(robotChasis *robot, double x, double y, double deg){
  robot1 = robot;
  xPos = x;
  yPos = y;
  angleD = deg;
  angleR = 0;
}

double odometry::getangleD(){ return angleD; }
double odometry::getangleR(){ return angleR; }
double odometry::getXPos(){ return xPos; }
double odometry::getYPos(){ return yPos; }

int odometry::updatePosition(){
  double loopTime;
  double deltaL;
  double deltaR;
  double deltaS;
  double prevLeftEnc = 0;
  double prevRightEnc = 0;
  double prevBackEnc = 0;
  double currLeftEnc = 0;
  double currRightEnc = 0;
  double currBackEnc = 0;
  

  // This loop will update the x and y position and angle the bot is facing
  while(true){

    //Current time at start of loop
    loopTime = robot1->Brain.Timer.time(msec);

    currLeftEnc = robot1->leftTracker.position(deg);
    currRightEnc = robot1->rightTracker.position(deg);
    currBackEnc = robot1->backTracker.position(deg);

    //The change in encoder values since last cycle in inches
    deltaL = (currLeftEnc - prevLeftEnc) * robot1->getWheelCir()/360;
    deltaR = (currRightEnc - prevRightEnc)* robot1->getWheelCir()/360;
    deltaS = (currBackEnc - prevBackEnc)* robot1->getWheelCir()/360;

    //Update previous value of the encoders
    prevLeftEnc = currLeftEnc;
    prevRightEnc = currRightEnc;
    prevBackEnc = currBackEnc;

    double h;
    double i;
    double h2;

   
    double a = (deltaL - deltaR)/(robot1->getsL() + robot1->getsR());
    //double a = (angleD - robot1->gyroM.rotation(deg)) * (robot1->getPI()/180);

    if(a){
      double r = deltaR/a;
      i = a / 2.0;
      double sinI = sin(i);
      h = (r + robot1->getsR()) * sinI * 2.0;

      double r2 = deltaS/a;
      h2 = (r2 + robot1->getsS()) * sinI * 2.0;
    } else {
      h = deltaR;
      i = 0;
      h2 = deltaS;
    }
    double p = i + angleR;
    double cosP = cos(p);
    double sinP = sin(p);

    yPos += h*cosP;
    xPos += h*sinP;

    yPos += h2*(-sinP);
    xPos += h2*cosP;
    angleR += a;
    angleD = angleR * (180/robot1->getPI());
    //Delays task so it does not hog all resources
    task::sleep(10 - (robot1->Brain.Timer.time(msec)-loopTime));
  }
  return 1;
}

int odometry::updateScreen(){
    // Clears controller the screen.
  robot1->Controller1.Screen.clearScreen();
  double loopTime;

  while(true){
    loopTime = robot1->Brain.Timer.time(msec);

    // Prints the x and y coordinates and angle the bot is facing to the Controller.
    robot1->Controller1.Screen.setCursor(0, 0);
    robot1->Controller1.Screen.print("x: %.1fin y: %.1fin     ", xPos, yPos);
    //robot1->Controller1.Screen.print("right: %.1lf     ", robot1->rightTracker.rotation(deg));
    robot1->Controller1.Screen.newLine();
    robot1->Controller1.Screen.print("Angle: %.1f°    ", angleD);
    //robot1->Controller1.Screen.print("left: %.1lf     ", robot1->leftTracker.rotation(deg));
    robot1->Controller1.Screen.newLine();
    //robot1->Controller1.Screen.print("back: %.1lf     ", robot1->backTracker.rotation(deg));
    // Controller1.Screen.print("Drive mV: %.0lf");

    // Prints information about the bot to the console
    //printf("Distance: %.2lf Y Voltage: %.0f X Voltage: %.0f\n", vMag, yVoltage, xVoltage);
    //printf("Tracking Wheels Angle: %0.f   IMU angle: %0.lf\n", angleD, robot1->gyroM.rotation(deg));
    //printf("rightTW: %.0lf, leftTW: %0.lf, backTW: %.0lf\n", robot1->rightTracker.position(deg), robot1->leftTracker.position(deg), robot1->backTracker.position(deg));
    //printf("Flywheel RPM: %.1lf, Flywheel Voltage: %.0lf\n\n\n", robot1->flyOuttake.velocity(rpm), robot1->flyOuttake.voltage(voltageUnits::mV));
    //printf("%.0lf, %.0lf, %.0lf \n", Brain.Timer.time(msec), flyOuttake.velocity(rpm), flyOuttake.voltage(voltageUnits::mV));

    //Delays task so it does not hog all resources
    task::sleep(200 - (robot1->Brain.Timer.time(msec)-loopTime));
  }

  return 1;
}