#include "main.h"

using namespace okapi;

Motor rightFront(rightFrontPort, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightBack(rightBackPort, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftFront(leftFrontPort, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftBack(leftBackPort, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

std::shared_ptr<ChassisController> drive =
  ChassisControllerBuilder()
  .withMotors({leftFront, leftBack}, {rightFront, rightBack})   //MotorGroups for left and right side
  .withDimensions(AbstractMotor::gearset::blue, {{4_in, 10_in}, imev5BlueTPR})		  //Blue gearset(100 rpm) and wheel dimensions
  .build();

typedef struct PID pid;

pid left;
pid right;


void updateDrive()
{
  drive -> getModel() -> tank(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightY));
}


void translatePID(double leftDistance, double rightDistance)
{
	//inertial_values = 0;
	drive -> getModel() -> setEncoderUnits(AbstractMotor::encoderUnits::degrees);

	left.target = leftDistance * (360 / (2 * 3.1415 * (4 / 2)));				// calculates left side motors target distance in wheel degrees
	right.target = rightDistance * (360 / (2 * 3.1415 * (4 / 2)));			// calculates right side motors target distance in wheel degrees

	//PID constants
	left.kP = 0.001575;
	left.kI = 0.0005;
	left.kD = 0.00015;

	right.kP = 0.001575;
	right.kI = 0.0005;
	right.kD = 0.00015;

	//initializes left and right side drivetrain PID controllers
	auto leftController = IterativeControllerFactory::posPID(left.kP, left.kI, left.kD);

	auto rightController = IterativeControllerFactory::posPID(right.kP, right.kI, right.kD);

	drive -> getModel() -> resetSensors();						//reset drivetrain motor sensor values before use
	//nertial_sensor.reset();													//reset inertial sensor values before use

	while (true)
	{

		//pros::lcd::set_text(2, std::to_string(drive -> getModel() -> getSensorVals()[0]));
		left.error = left.target - (drive -> getModel() -> getSensorVals()[0]);
		left.power = leftController.step(left.error); 							//returns speed for left side
		//pros::lcd::set_text(1, std::to_string(left_drive_PID.error));

		right.error = right.target - drive -> getModel() -> getSensorVals()[1];
		right.power = rightController.step(right.error); 					//returns speed for right side
		//pros::lcd::set_text(2, std::to_string(right_drive_PID.error));
		//pros::lcd::set_text(3, std::to_string(leftFront.getPosition()));
		//pros::lcd::set_text(4, std::to_string(rightFront.getPosition()));
/*
		inertial_values = inertial_sensor.get();
		pros::lcd::set_text(4, std::to_string(inertial_values));
		if (inertial_values < 0)
		{
			right_drive_PID.speed += abs(inertial_values) * 0.01;
			left_drive_PID.speed -= abs(inertial_values) * 0.01;
		}
		else if (inertial_values > 0)
		{
			left_drive_PID.speed += abs(inertial_values) * 0.01;
			right_drive_PID.speed -= abs(inertial_values) * 0.01;
		}
    **/
		pros::lcd::set_text(2, std::to_string(left.power));
		pros::lcd::set_text(3, std::to_string(right.power));

		drive -> getModel() -> tank(-left.power, -right.power);

		if ((abs(left.error) < 5) && (abs(right.error) < 5))
		{
			break;
		}

		pros::delay(20);
	}

	drive -> getModel() -> tank(0, 0); 								//brakes drivetrain right after PID movement

}
