#include "main.h"

using namespace okapi;

Motor rightFront(rightFrontPort, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightBack(rightBackPort, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftFront(leftFrontPort, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftBack(leftBackPort, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
ADIEncoder encoder(encoderTop, encoderBottom, true);

typedef struct PID pid;

pid translate;
pid drift;
pid rotate;

double Sl = 6;      //distance from tracking center to middle of left wheel
double Sr = 6;      //distance from tracking center to middle of right wheel
double Ss = 0.5;    //distance from tracking center to middle of the tracking wheel
double wheelDiameter = 4;
double trackingDiameter = 2.75;

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
double targetDistance;
double targetAngle;


bool turnComplete = false;
bool routeComplete = false;

std::shared_ptr<ChassisController> drive =
  ChassisControllerBuilder()
  .withMotors({leftFront, leftBack}, {rightFront, rightBack})   //MotorGroups for left and right side
  .withDimensions(AbstractMotor::gearset::blue, {{4_in, 10_in}, imev5BlueTPR})		  //Blue gearset(100 rpm) and wheel dimensions
  .build();


void updateDrive()
{
  drive -> getModel() -> tank(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightY));
}


void odometry(double x_pos, double y_pos)
{
  //drive -> getModel() -> resetSensors();

  leftBack.tarePosition();
  leftFront.tarePosition();
  rightBack.tarePosition();
  rightFront.tarePosition();

  translate.kP = 0.015;
  translate.kI = 0.0015;
  translate.kD = 0.0085; //was 0.007

  drift.kP = 0.001;
  //drift.kI = 0.0;
  //drift.kD = 0;


	auto moveController = IterativeControllerFactory::posPID(translate.kP, translate.kI, translate.kD);
  //auto driftController = IterativeControllerFactory::posPID(drift.kP, drift.kI, drift.kD);
  auto driftController = IterativeControllerFactory::posPID(drift.kP, 0.01, 0.01);
  auto rotateController = IterativeControllerFactory::posPID(translate.kP, translate.kI, translate.kD);



  if (atan2f(distanceY, distanceX) == 0)
  {
    targetAngle = 0;
  }
  else
  {
    targetAngle = -1 * (atan2f(distanceY, distanceX) - M_PI/2);
  }
  pros::lcd::set_text(1, std::to_string(atan2f(distanceY, distanceX)));

  while (routeComplete == false)
  {
    leftEncoderVals = ((leftBack.getPosition() * 6 / 5) / 7) * 3;
    rightEncoderVals = ((rightBack.getPosition() * 6 / 5) / 7) * 3;
    currentSide = encoder.get();

    deltaLeft = (leftEncoderVals - prevLeftEncoderVals) * 2 * (M_PI / 360) * (wheelDiameter / 2);
    deltaRight = (rightEncoderVals - prevRightEncoderVals) * 2 * (M_PI / 360) * (wheelDiameter / 2);
    deltaSide = (currentSide - prevSidePos) * 2 * (M_PI / 360) * (trackingDiameter / 2);

    prevLeftEncoderVals = leftEncoderVals;
    prevRightEncoderVals = rightEncoderVals;
    prevSidePos = currentSide;

     //Calculate the total change in the left and right encoder values since the last reset, and convert to distance of wheel travel;

    deltaLr = ((leftEncoderVals - leftAtReset) * M_PI * wheelDiameter) / 360;
    deltaRr = ((rightEncoderVals - rightAtReset) * M_PI * wheelDiameter) / 360;

    newTheta = (thetaReset + (deltaLr - deltaRr) / (Sl + Sr));

    changeTheta = newTheta - angle;

    deltaSide = deltaSide - Ss * changeTheta;

    if(changeTheta == 0)
    {
      deltaX = deltaSide;
      deltaY = deltaRight;
    }
    else
    {
      deltaX = (2 * sin(changeTheta / 2)) * (deltaSide / changeTheta + Ss);
      deltaY = (2 * sin(changeTheta / 2)) * (deltaRight / changeTheta + Sr);
    }

    thetaM = angle + changeTheta / 2;

    theta = atan2f(deltaY, deltaX);
    radius = sqrt(deltaX * deltaX + deltaY * deltaY);
    theta = theta - thetaM;
    deltaX = radius * cos(theta);
    deltaY = radius * sin(theta);

    newTheta += M_PI;

    while (newTheta <= 0)
    {
      newTheta += 2 * M_PI;
    }

    newTheta = fmod(newTheta, 2 * M_PI);
    newTheta -= M_PI;

    angle = newTheta;
    x += deltaX; //current x position
    y += deltaY;  //current y position

    //  right is negative x left is negative x
    pros::lcd::set_text(3, std::to_string(y));



    //////////////// Move Functions Start Here ////////////////
    if (turnComplete  == true)
    {
      distanceX = x_pos - x;
      distanceY = y_pos - y;
      pros::lcd::set_text(4, std::to_string(distanceY));
      targetDistance = sqrt(distanceX * distanceX + distanceY * distanceY);

      drift.error = targetAngle - angle;
      drift.power = driftController.step(drift.error);

      // for drift constant * angle + power

      translate.error = targetDistance;
      translate.power = moveController.step(translate.error);

      //pros::lcd::set_text(3, std::to_string(translate.power));  //accurate values

      drive -> getModel() -> tank(-translate.power + drift.power, -translate.power - drift.power); //add drift to power


      if (abs(translate.error) < 0.5 && abs(drift.error) < 1)
      {
        routeComplete = true;
      }

    }
    else
    {
      rotate.error = targetAngle - angle;
      rotate.power = rotateController.step(rotate.error);

      pros::lcd::set_text(5, std::to_string(rotate.error));
      //drive -> getModel() -> tank(rotate.power, -rotate.power);

      if (abs(rotate.error) < M_PI/180)
      {
        turnComplete = true;
      }
    }
    //pros::lcd::set_text(2, std::to_string(translate.error));
    //pros::lcd::set_text(5, std::to_string(turnComplete));
    pros::delay(10);

    // switch (choice) {
    //   case 'x':
    //     return x;
    //     break;
    //   case 'y':
    //     return y;
    //     break;
    //   case 'a':
    //     return angle;
    //     break;
    //   }



  }

  drive -> getModel() -> tank(0, 0);
}
