#include "main.h"

void updatePneumatics()
{
   if (controller.getDigital(ControllerDigital::A) == 1)
   {
     pros::c::adi_digital_write(pneumaticLeftPort, HIGH);
   }
   else if (controller.getDigital(ControllerDigital::B) == 1)
   {
     pros::c::adi_digital_write(pneumaticLeftPort, LOW);
   }
}
