#include "main.h";

uisng namespace okapi;

Motor leftBackDrive(2, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftFrontDrive(17, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightBackDrive(8, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightFrontDrive(10, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

void joystickDrive();
