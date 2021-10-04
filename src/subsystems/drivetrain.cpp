#include "main.h"

using namespace okapi;

Motor rightFront(rightFrontPort, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightBack(rightBackPort, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftFront(leftFrontPort, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftBack(leftBackPort, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
IMU inertial_sensor(imuPort, IMUAxes::z);

std::shared_ptr<ChassisController> drive =
  ChassisControllerBuilder()
  .withMotors({leftFront, leftBack}, {rightFront, rightBack})   //MotorGroups for left and right side
  .withDimensions(AbstractMotor::gearset::blue, {{4_in, 10_in}, imev5BlueTPR})		  //Blue gearset(100 rpm) and wheel dimensions
  .build();

typedef struct PID pid;

pid translate;
pid turnAngle;

pid left_drive_PID;
pid right_drive_PID;

double inertial_values = inertial_sensor.get();

double Sl = 5.125;    //distance from tracking center to middle of left wheel        ///get correct value
double  Sr = 5.125;   //distance from tracking center to middle of right wheel       //get correct value
double Ss = 7.75;     //distance from tracking center to middle of the tracking wheel  //get correct value
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

double distanceX;
double distanceY;
double distanceAngle;


void updateDrive()
{
  drive -> getModel() -> tank(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightY));
}

void rotate_PID(double turn_degrees)
{
    drive -> getModel() -> setEncoderUnits(AbstractMotor::encoderUnits::degrees);

    inertial_values = 0;

    //PID constants
    left_drive_PID.kP = 0.010150;
    left_drive_PID.kI = 0.0080;
    left_drive_PID.kD = 0.000005;

    right_drive_PID.kP = 0.010150;
    right_drive_PID.kI = 0.0080;
    right_drive_PID.kD = 0.000005;

    //initializes left and right side drivetrain PID controllers
    auto left_pid_controller = IterativeControllerFactory::posPID(left_drive_PID.kP,
                                                                                                                                left_drive_PID.kI,
                                                                                                                                left_drive_PID.kD);

    auto right_pid_controller = IterativeControllerFactory::posPID(right_drive_PID.kP,
                                                                                                                                 right_drive_PID.kI,
                                                                                                                                 right_drive_PID.kD);

    drive -> getModel() -> resetSensors();            //reset drivetrain motor sensor values before use
    inertial_sensor.reset();                                        //reset inertial sensor values before use

    while (true)
    {

        inertial_values = inertial_sensor.get();
        pros::lcd::set_text(1, std::to_string(inertial_values));

        //pros::lcd::set_text(2, std::to_string(drive -> getModel() -> getSensorVals()[0]));
        left_drive_PID.error = turn_degrees - inertial_values;
        left_drive_PID.speed = left_pid_controller.step(left_drive_PID.error);                             //returns speed for left side

        right_drive_PID.error = turn_degrees - inertial_values;
        right_drive_PID.speed = right_pid_controller.step(right_drive_PID.error);                     //returns speed for right side
        //pros::lcd::set_text(3, std::to_string(leftFront.getPosition()));
        //pros::lcd::set_text(4, std::to_string(rightFront.getPosition()));
        //pros::lcd::set_text(2, std::to_string(left_drive_PID.error));
        //pros::lcd::set_text(3, std::to_string(right_drive_PID.error));

        drive -> getModel() -> arcade(-left_drive_PID.speed, right_drive_PID.speed);

        if ((abs(left_drive_PID.error) < 0.5) && (abs(right_drive_PID.error) < 0.5))
        {
            break;
        }

        pros::delay(20);
    }

    drive -> getModel() -> tank(0, 0);                                 //brakes drivetrain right after PId movement
}

float modulo(float a, float b)
{
  while (a>b)
  {
    a-=b;
  }
  return a;
}

double odometry(char choice)
{

  //store current encoder values in local variables:
  leftEncoderVals = (leftBack.getPosition() + leftFront.getPosition())/2;     //average left encoder values
  rightEncoderVals = (rightBack.getPosition() + rightFront.getPosition())/2;  //average right encoder values

  //currentSide = sideEnc.get_value; //need to get value from side encoder

  deltaLeft = (leftEncoderVals - prevLeftEncoderVals)*2*(M_PI/360)*(wheelDiameter/2);     //distance travelled since last loop
  deltaRight = (rightEncoderVals - prevRightEncoderVals)*2*(M_PI/360)*(wheelDiameter/2);  //distance travelled since last loop
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

  if(changeTheta ==0)
  {
    deltaX = deltaSide; //step 7
    deltaY = deltaRight;
  }
  else
  {
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
   while (newTheta <= 0)
   {
     newTheta+=2*M_PI;
   }
   newTheta = modulo(newTheta, 2*M_PI);
   newTheta-= M_PI;

   angle = newTheta;
   x = x - deltaX; //step 11
   y = y + deltaY;

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
