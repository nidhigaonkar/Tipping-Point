#include "main.h"

using namespace okapi;

//Lift Motors
Motor itz_lift(5, false, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees);

/*//Drivetrain Motors
Motor leftBackDrive(6, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftFrontDrive(17, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightBackDrive(7, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightFrontDrive(10, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
**/
//Controller
Controller controller;
