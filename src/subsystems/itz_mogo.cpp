#include "main.h"

using namespace okapi;

Motor itz_lift(itzLiftPort, false, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees);

void updateItzLift()
{
  itz_lift.moveVelocity(100*(controller.getDigital(ControllerDigital::down) - controller.getDigital(ControllerDigital::up)));
}
