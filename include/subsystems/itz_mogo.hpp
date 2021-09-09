#include "main.h"

using namespace okapi;

Motor itz_lift(5, false, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees);

void itz_lift_control();
