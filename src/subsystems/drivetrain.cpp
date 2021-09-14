#include "main.h"

using namespace okapi;

Motor rightFront(rightFrontPort, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightBack(rightBackPort, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftFront(leftFrontPort, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftBack(leftBackPort, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

std::shared_ptr<ChassisController> drive =
  ChassisControllerBuilder()
  .withMotors({leftFront, leftBack}, {rightFront, rightBack})   //MotorGroups for left and right side
  .withDimensions(AbstractMotor::gearset::blue, {{4_in, 10_in}, imev5BlueTPR})		  //Blue gearset(100 rpm) and wheel dimensions
  .build();

void updateDrive()
{
  drive -> getModel() -> arcade(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightX));
}
