#include "main.h"

using namespace okapi;

Motor fourBarLift(fourBarLiftPort, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

void updateFourBarLift()
{
  fourBarLift.moveVelocity(100 * (controller.getDigital(ControllerDigital::L1) - controller.getDigital(ControllerDigital::L2)));
}
