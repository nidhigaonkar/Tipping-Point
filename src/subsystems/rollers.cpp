#include "main.h"

using namespace okapi;

Motor rollers(rollerPort, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

void updateRollers()
{
  rollers.moveVelocity(200 * (controller.getDigital(ControllerDigital::R1) - controller.getDigital(ControllerDigital::R2)));
  rollers.setBrakeMode(AbstractMotor::brakeMode::coast);
}
