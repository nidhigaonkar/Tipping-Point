#include "main.h"

using namespace okapi;

void itz_lift_control()
{
  itz_lift.moveVelocity(100*(controller.getDigital(ControllerDigital::R2) - controller.getDigital(ControllerDigital::R1)));
}
