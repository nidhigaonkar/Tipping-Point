#include "main.h"

using namespace okapi;

Motor leftBackDrive(2, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftFrontDrive(17, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightBackDrive(8, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightFrontDrive(10, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

Controller controller;

void joystickDrive();
