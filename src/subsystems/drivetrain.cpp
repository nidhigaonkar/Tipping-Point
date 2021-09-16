#include "main.h"

using namespace okapi;

Motor fourBarLift(fourBarLiftPort, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

double setpoint = 0;
double integral = 0;
double prevError;
double kP;
double kI;
double kD;
double target;
double error;
double derivative;
double power;

void updateFourBarLift()
{
  fourBarLift.moveVelocity(100 * (controller.getDigital(ControllerDigital::L1) - controller.getDigital(ControllerDigital::L2)));

  if (controller.getDigital(ControllerDigital::L1) == 0 && controller.getDigital(ControllerDigital::L2) == 0)
  {
    setpoint = fourBarLift.getPosition();
    fourBarLift.moveVelocity(PIDFourBar(setpoint));
  }
}

double PIDFourBar(double setpoint)
{
  kP = 0.5;
  kI = 0;
  kD = 0.1;

  target = setpoint;
  error = setpoint - fourBarLift.getPosition();
  integral += error;
  derivative = error - prevError;

  power = kP * error + kI * integral + kD * derivative;

  return power;
}
