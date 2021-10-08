#include "main.h"

using namespace okapi;

Motor rightFront(rightFrontPort, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightBack(rightBackPort, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftFront(leftFrontPort, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftBack(leftBackPort, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
ADIEncoder encoder(encoderTop, encoderBottom, false);

typedef struct PID pid;

pid translate;
pid turnAngle;
pid rotate;

double Sl = 6.25;    //distance from tracking center to middle of left wheel
double Sr = 6.25;    //distance from tracking center to middle of right wheel
double Ss = 7.75;     //distance from tracking center to middle of the tracking wheel
double wheelDiameter = 4;   //get correct value
double trackingDiameter = 3.25;

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
double currentSide = 0;

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

double distanceX;
double distanceY;
double distanceAngle;

std::shared_ptr<ChassisController> drive =
  ChassisControllerBuilder()
  .withMotors({leftFront, leftBack}, {rightFront, rightBack})   //MotorGroups for left and right side
  .withDimensions(AbstractMotor::gearset::blue, {{4_in, 10_in}, imev5BlueTPR})		  //Blue gearset(100 rpm) and wheel dimensions
  .build();


void updateDrive()
{
  drive -> getModel() -> tank(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightY));
}

float modulo(float a, float b)
{
  while (a > b)
  {
    a -= - b;
  }
  return a;
}

double odometry(char choice)
{

  //store current encoder values in local variables:
  leftEncoderVals = (leftBack.getPosition() + leftFront.getPosition())/2;     //average left encoder values
  rightEncoderVals = (rightBack.getPosition() + rightFront.getPosition())/2;  //average right encoder values

  //currentSide = sideEnc.get_value; //need to get value from side encoder

  deltaLeft = (leftEncoderVals - prevLeftEncoderVals) * 2 * (M_PI / 360) * (wheelDiameter / 2);     //distance travelled since last loop
  deltaRight = (rightEncoderVals - prevRightEncoderVals) * 2 * (M_PI / 360) * (wheelDiameter / 2);  //distance travelled since last loop
  deltaSide = (currentSide - prevSidePos) * 2 * (M_PI / 360) * (trackingDiameter / 2); //in inches

  prevLeftEncoderVals = leftEncoderVals;
  prevRightEncoderVals = rightEncoderVals;
  prevSidePos = currentSide;

  //Calculate the total change in the left and right encoder values since the last reset, and convert to distance of wheel travel;

  deltaLr = (leftEncoderVals - leftAtReset) * 2 * (M_PI / 360) * (wheelDiameter / 2);
  deltaRr = (rightEncoderVals - rightAtReset) * 2 * (M_PI / 360) * (wheelDiameter / 2); //in inches

  newTheta = (thetaReset + (deltaLr - deltaRr) / (Sl + Sr));  //step 5


  changeTheta = newTheta - angle;

  deltaSide = deltaSide - Ss * changeTheta;

  if(changeTheta == 0)
  {
    deltaX = deltaSide; //step 7
    deltaY = deltaRight;
  }
  else
  {
    deltaX = (2 * sin(changeTheta / 2)) * (deltaSide / changeTheta + Ss); //step 8
    deltaY = (2 * sin(changeTheta / 2)) * (deltaRight / changeTheta + Sr);
  }


  thetaM = angle + changeTheta / 2;


  theta = atan2f(deltaY, deltaX);
  radius = sqrt(deltaX * deltaX + deltaY * deltaY);
  theta = theta - thetaM;         //step 10
  deltaX = radius * cos(theta);
  deltaY = radius * sin(theta);


  newTheta += M_PI;
   while (newTheta <= 0)
   {
     newTheta += 2 * M_PI;
   }
   newTheta = modulo(newTheta, 2 * M_PI);
   newTheta -= M_PI;

   angle = newTheta;
   x += deltaX; //step 11
   y += deltaY;

   switch (choice)
   {
    case 'x':
      return x;
      break;
    case 'y':
      return y;
      break;
    case 'a':
      return angle;
      break;
   }

   return 0;

}

void odom_move(double x_pos, double y_pos)
{
  //PID constants
	translate.kP = 0.001575;
	translate.kI = 0.0005;
	translate.kD = 0.00015;

	auto moveController = IterativeControllerFactory::posPID(translate.kP, translate.kI, translate.kD);
  auto turnController = IterativeControllerFactory::posPID(translate.kP, translate.kI, translate.kD);

  while (true)
  {
    distanceX = x_pos - odometry(x);
    distanceY = y_pos - odometry(y);

    if (distanceX < 0)
    {
      distanceAngle = 3 * (M_PI/2) - atan2f(distanceY, distanceX) - odometry(angle);
    }
    else if (distanceX > 0)
    {
      distanceAngle = M_PI/2 - atan2f(distanceY, distanceX) - odometry(angle);
    }

    turnAngle.error = distanceAngle;
    turnAngle.power = turnController.step(turnAngle.error);

    translate.error = sqrt(distanceX * distanceX + distanceY * distanceY);
    translate.power = moveController.step(translate.error);

    drive -> getModel() -> tank(translate.power - turnAngle.power, translate.power + turnAngle.power);

    if (abs(translate.error) < 5 && abs(turnAngle.error) < 1)
    {
      break;
    }

    pros::delay(10);
  }

  drive -> getModel() -> tank(0, 0);
}

void odom_rotate(double x_pos, double y_pos)
{
  rotate.kP = 0.001575;
	rotate.kI = 0.0005;
	rotate.kD = 0.00015;

	auto angleController = IterativeControllerFactory::posPID(rotate.kP, rotate.kI, rotate.kD);

  while (true)
  {
    distanceX = x_pos - odometry(x);
    distanceY = y_pos - odometry(y);

    if (distanceX < 0)
    {
      distanceAngle = 3 * (M_PI/2) - atan2f(distanceY, distanceX) - odometry(angle);
    }
    else if (distanceX > 0)
    {
      distanceAngle = M_PI/2 - atan2f(distanceY, distanceX) - odometry(angle);
    }

    rotate.error = distanceAngle;
    rotate.power = angleController.step(rotate.error);

    drive -> getModel() -> tank(rotate.power, -rotate.power);

    if (abs(rotate.error) < 1)
    {
      break;
    }

    pros::delay(10);
  }

  drive -> getModel() -> tank(0, 0);

}
