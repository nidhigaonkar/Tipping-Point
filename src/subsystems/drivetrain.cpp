#include "main.h"

using namespace okapi;
//done

Motor rightFront(rightFrontPort, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightBack(rightBackPort, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftFront(leftFrontPort, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftBack(leftBackPort, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

IMU inertial_sensor(4, IMUAxes::z);


double Sl = 5.125; //distance from tracking center to middle of left wheel        ///get correct value
double  Sr = 5.125; //distance from tracking center to middle of right wheel       //get correct value
double Ss = 7.75; //distance from tracking center to middle of the tracking wheel  //get correct value
double wheelDiameter = 4.125;   //get correct value
double trackingDiameter = 3;

double x; //x coordinate returned
double y; //y coordinate  returned
double angle = 0; //angle  returned

double prevLeftEncoderVals;
double prevRightEncoderVals;
double prevSidePos;

double changeTheta;
double newTheta;
double thetaM;


double leftEncoderVals = 0;
double rightEncoderVals = 0;
double currentSide = 0;   ///encoder

double leftAtReset = 0;
double rightAtReset = 0;
double thetaReset = 0;

double deltaLeft = 0;
double deltaRight = 0;
double deltaSide = 0;

double deltaLr = 0;
double deltaRr = 0;

double deltaX;
double deltaY;

double theta;
double radius;

//
// double leftEncoderVals = drive -> getModel() -> getSensorVals()[0];  //left side of drive encoder values  (averaging values)
// double rightEncoderVals = drive -> getModel() -> getSensorVals()[1];  //right side of drive encoder values



std::shared_ptr<ChassisController> drive =
  ChassisControllerBuilder()
  .withMotors({leftFront, leftBack}, {rightFront, rightBack})   //MotorGroups for left and right side
  .withDimensions(AbstractMotor::gearset::blue, {{4_in, 10_in}, imev5BlueTPR})		  //Blue gearset(100 rpm) and wheel dimensions
  .build();


//added:
  //odom no encoder and tracking wheels:
  std::shared_ptr<ChassisController> chassis =
    ChassisControllerBuilder()
    .withMotors({leftFront, leftBack}, {rightFront, rightBack})   //MotorGroups for left and right side
    .withDimensions(AbstractMotor::gearset::blue, {{4_in, 10_in}, imev5BlueTPR})		  //Blue gearset(100 rpm) and wheel dimensions
    .withOdometry()
    .buildOdometry();


void updateDrive()
{
  drive -> getModel() -> arcade(controller.getAnalog(ControllerAnalog::leftY),controller.getAnalog(ControllerAnalog::rightX));
}


float modulo(float a, float b) {
  while (a>b) {
    a-=b;
  }
  return a;
}

void odometry(){

  //store current encoder values in local variables:
  leftEncoderVals = (leftBack.getPosition() + leftFront.getPosition())/2; //averaging the values, can update later
  leftEncoderVals = (rightBack.getPosition() + rightFront.getPosition())/2; //averaging the vlaues, can update later

  //currentSide = sideEnc.get_value; //need to get value from side encoder

  deltaLeft = (leftEncoderVals - prevLeftEncoderVals)*2*(M_PI/360)*(wheelDiameter/2); //in inches
  deltaRight = (rightEncoderVals - prevRightEncoderVals)*2*(M_PI/360)*(wheelDiameter/2); //in inches
  deltaSide = (currentSide - prevSidePos)*2*(M_PI/360)*(trackingDiameter/2); //in inches

  prevLeftEncoderVals = leftEncoderVals;
  prevRightEncoderVals = rightEncoderVals;
  prevSidePos = currentSide;

  //Calculate the total change in the left and right encoder values since the last reset, and convert to distance of wheel travel;

  deltaLr = (leftEncoderVals - leftAtReset)*2*(M_PI/360)*(wheelDiameter/2);
  deltaRr = (rightEncoderVals - rightAtReset)*2*(M_PI/360)*(wheelDiameter/2); //in inches

  newTheta = (thetaReset + (deltaLr -deltaRr) / (Sl+Sr));  //step 5


  changeTheta = newTheta - angle;

  deltaSide = deltaSide - Ss*changeTheta;

  if(changeTheta ==0){
    deltaX = deltaSide; //step 7
    deltaY = deltaRight;
  }
  else {
    deltaX = (2*sin(changeTheta/2))*(deltaSide/changeTheta + Ss); //step 8
    deltaY = (2*sin(changeTheta/2))*(deltaRight/changeTheta +Sr);
  }


  thetaM = angle + changeTheta/2;


  theta = atan2f(deltaY, deltaX);
  radius = sqrt(deltaX*deltaX + deltaY*deltaY);
  theta = theta - thetaM;         //step 10
  deltaX = radius*cos(theta);
  deltaY = radius*sin(theta);


  newTheta +=M_PI;
   while (newTheta <= 0) {
     newTheta+=2*M_PI;
   }
   newTheta = modulo(newTheta, 2*M_PI);
   newTheta-= M_PI;

   angle = newTheta;
   x = x - deltaX; //step 11
   y = y + deltaY;

}

double getX(){
  return x;
}

double getY(){
  return y;
}

double getAngleDegrees(){
  return angle*180/M_PI;
}

double getAngle() {
  return angle;
}
