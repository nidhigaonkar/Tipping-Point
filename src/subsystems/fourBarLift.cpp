#include "main.h"

using namespace okapi;

Motor fourBarLift(fourBarLiftPort, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

double setpoint;

typedef struct PID pid;

pid FB;

double PIDFourBar(double setpoint)
{
  FB.kP = 0.5;
  FB.kI = 0;
  FB.kD = 0.1;

  FB.target = setpoint;
  FB.error = FB.target - fourBarLift.getPosition();
  FB.integral += FB.error;
  FB.derivative = FB.error - FB.prevError;

  FB.power = FB.kP * FB.error + FB.kI * FB.integral + FB.kD * FB.derivative;

  return FB.power;
}


void updateFourBarLift()
{
  if (controller.getDigital(ControllerDigital::R1) == 0 && controller.getDigital(ControllerDigital::R2) == 0)
  {
    setpoint = fourBarLift.getPosition();
    fourBarLift.moveVelocity(PIDFourBar(setpoint));
  }
  else
  {
    fourBarLift.moveVelocity(100 * (controller.getDigital(ControllerDigital::R1) - controller.getDigital(ControllerDigital::R2)));
  }
}
