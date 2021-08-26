#include "main.h"

//auton code


using namespace okapi;
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		// pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

//Drivetrain Motor initialization
Motor rightBack(8, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor rightFront(6, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor leftFront(2, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor leftBack(1, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

//Controller Initialization
Controller controller;

//Misc Motor Initialization
Motor four_bar_lift(10, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor chain_bar(9, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor mogo_lift(7, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor mogo_spinner(3, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

//Sensor Initialization
IMU inertial_sensor(4, IMUAxes::z);

//Variable Initialization
int A_buttonPress = 0;
int lift_macro_level = 0;
int mogo_macro_level = 2;

double inertial_values = 0;

double chain_bar_speed = 0;
double chain_bar_PID_speed = 0;

double four_bar_speed = 0;
double four_bar_PID_speed = 0;

double mogo_lift_speed = 0;

struct PID
{
	float kP;
	float kI;
	float kD;
	float integral;
	float derivative;
	float error;
	float prev_error;
	float speed;
	float target;
	float sensor_value;
};

typedef struct PID pid;

pid CB; 								//Chain bar PID
pid FB; 								//Four bar PID
pid ML;
pid left_drive_PID; 		//PID for left side of drivetrain
pid right_drive_PID;		//PID for right side of drivetrain


//Drivetrain Controller Initialization
std::shared_ptr<ChassisController> drive =
  ChassisControllerBuilder()
  .withMotors({leftBack, leftFront}, {rightFront, rightBack}) 												//MotorGroups for left and right side
  .withDimensions(AbstractMotor::gearset::green, {{4_in, 10_in}, imev5GreenTPR})		  //Gearset(rpm) and wheel dimensions
  .build();


double chain_bar_PID(double chain_bar_set) 							//PID for chain bar
{
	CB.kP = 0.3;
	CB.kI = 0.0002;
	CB.kD = 0.01;
	CB.target = chain_bar_set;
	//pros::lcd::set_text(4, std::to_string(chain_bar.getPosition()));
	CB.error = CB.target - chain_bar.getPosition();
	CB.derivative = CB.error - CB.prev_error;
	CB.integral += CB.error;
	CB.prev_error = CB.error;
	CB.speed = CB.error * CB.kP + CB.integral * CB.kI + CB.derivative * CB.kD;
	//pros::lcd::set_text(2, std::to_string(CB.speed));
	return CB.speed;
	pros::delay(10);
}

double four_bar_PID(double four_bar_setpoint) 					//PID for four bar
{
	FB.kP = 0.2; //need tuning
	FB.kI = 0.001; //need tuning
	FB.kD = 0.007; //need tuning
	FB.target = four_bar_setpoint;
	FB.error = FB.target - four_bar_lift.getPosition();
	FB.derivative = FB.error - FB.prev_error;
	FB.integral += FB.error;
	FB.prev_error = FB.error;
	FB.speed = FB.kP * FB.error + FB.kI * FB.integral + FB.kD * FB.derivative;
	//pros::lcd::set_text(3, std::to_string(FB.speed));
	return FB.speed;
	pros::delay(10);
}

double mogo_lift_PID(double mogo_lift_setpoint)					//PID for mogo lift
{
	ML.kP = 0.2; //need tuning
	ML.kI = 0.001; //need tuning
	ML.kD = 0.005; //need tuning
	ML.target = mogo_lift_setpoint;
	ML.error = ML.target - mogo_lift.getPosition();
	ML.derivative = ML.error - ML.prev_error;
	ML.integral += ML.error;
	ML.prev_error = ML.error;
	ML.speed = ML.error * ML.kP + ML.integral * ML.kI + ML.derivative * ML.kD;
	pros::lcd::set_text(2, std::to_string(ML.speed));
	pros::lcd::set_text(4, std::to_string(mogo_lift.getPosition()));
	return ML.speed;
	pros::delay(10);
}

void translate_PID(double left_distance, double right_distance)
{

	inertial_values = 0;
	drive -> getModel() -> setEncoderUnits(AbstractMotor::encoderUnits::degrees);

	double left_target = left_distance * (360 / (2 * 3.1415 * (4 / 2)));				// calculates left side motors target distance in wheel degrees
	double right_target = right_distance * (360 / (2 * 3.1415 * (4 / 2)));			// calculates right side motors target distance in wheel degrees

	//PID constants
	left_drive_PID.kP = 0.001575;
	left_drive_PID.kI = 0.0005;
	left_drive_PID.kD = 0.00015;

	right_drive_PID.kP = 0.001575;
	right_drive_PID.kI = 0.0005;
	right_drive_PID.kD = 0.00015;

	//initializes left and right side drivetrain PID controllers
	auto left_pid_controller = IterativeControllerFactory::posPID(left_drive_PID.kP,
																																left_drive_PID.kI,
																																left_drive_PID.kD);

	auto right_pid_controller = IterativeControllerFactory::posPID(right_drive_PID.kP,
																																 right_drive_PID.kI,
																																 right_drive_PID.kD);

	drive -> getModel() -> resetSensors();						//reset drivetrain motor sensor values before use
	inertial_sensor.reset();													//reset inertial sensor values before use

	while (true)
	{

		//pros::lcd::set_text(2, std::to_string(drive -> getModel() -> getSensorVals()[0]));
		left_drive_PID.error = left_target - drive -> getModel() -> getSensorVals()[0];
		left_drive_PID.speed = left_pid_controller.step(left_drive_PID.error); 							//returns speed for left side
		//pros::lcd::set_text(1, std::to_string(left_drive_PID.error));

		right_drive_PID.error = right_target - drive -> getModel() -> getSensorVals()[1];
		right_drive_PID.speed = right_pid_controller.step(right_drive_PID.error); 					//returns speed for right side
		//pros::lcd::set_text(2, std::to_string(right_drive_PID.error));
		//pros::lcd::set_text(3, std::to_string(leftFront.getPosition()));
		//pros::lcd::set_text(4, std::to_string(rightFront.getPosition()));

		inertial_values = inertial_sensor.get();
		pros::lcd::set_text(4, std::to_string(inertial_values));
		if (inertial_values < 0)
		{
			right_drive_PID.speed += abs(inertial_values) * 0.01;
			left_drive_PID.speed -= abs(inertial_values) * 0.01;
		}
		else if (inertial_values > 0)
		{
			left_drive_PID.speed += abs(inertial_values) * 0.01;
			right_drive_PID.speed -= abs(inertial_values) * 0.01;
		}
		pros::lcd::set_text(2, std::to_string(left_drive_PID.speed));
		pros::lcd::set_text(3, std::to_string(right_drive_PID.speed));

		drive -> getModel() -> tank(-left_drive_PID.speed, -right_drive_PID.speed);

		if ((abs(left_drive_PID.error) < 5) && (abs(right_drive_PID.error) < 5))
		{
			break;
		}

		pros::delay(20);
	}

	drive -> getModel() -> tank(0, 0); 								//brakes drivetrain right after PID movement

}

void rotate_PID(double turn_degrees)
{
	drive -> getModel() -> setEncoderUnits(AbstractMotor::encoderUnits::degrees);

	inertial_values = 0;

	//PID constants
	left_drive_PID.kP = 0.010150;
	left_drive_PID.kI = 0.0080;
	left_drive_PID.kD = 0.000005;

	right_drive_PID.kP = 0.010150;
	right_drive_PID.kI = 0.0080;
	right_drive_PID.kD = 0.000005;

	//initializes left and right side drivetrain PID controllers
	auto left_pid_controller = IterativeControllerFactory::posPID(left_drive_PID.kP,
																																left_drive_PID.kI,
																																left_drive_PID.kD);

	auto right_pid_controller = IterativeControllerFactory::posPID(right_drive_PID.kP,
																																 right_drive_PID.kI,
																																 right_drive_PID.kD);

	drive -> getModel() -> resetSensors();			//reset drivetrain motor sensor values before use
	inertial_sensor.reset();										//reset inertial sensor values before use

	while (true)
	{

		inertial_values = inertial_sensor.get();
		pros::lcd::set_text(1, std::to_string(inertial_values));

		//pros::lcd::set_text(2, std::to_string(drive -> getModel() -> getSensorVals()[0]));
		left_drive_PID.error = turn_degrees - inertial_values;
		left_drive_PID.speed = left_pid_controller.step(left_drive_PID.error); 							//returns speed for left side

		right_drive_PID.error = turn_degrees - inertial_values;
		right_drive_PID.speed = right_pid_controller.step(right_drive_PID.error); 					//returns speed for right side
		//pros::lcd::set_text(3, std::to_string(leftFront.getPosition()));
		//pros::lcd::set_text(4, std::to_string(rightFront.getPosition()));
		//pros::lcd::set_text(2, std::to_string(left_drive_PID.error));
		//pros::lcd::set_text(3, std::to_string(right_drive_PID.error));

		drive -> getModel() -> tank(-left_drive_PID.speed, right_drive_PID.speed);

		if ((abs(left_drive_PID.error) < 0.5) && (abs(right_drive_PID.error) < 0.5))
		{
			break;
		}

		pros::delay(20);
	}

	drive -> getModel() -> tank(0, 0); 								//brakes drivetrain right after PId movement

}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	// pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous()
{

	mogo_lift.moveVelocity(mogo_lift_PID(-500));//lower mogo lift
	translate_PID(46.5, 46.5);//67.5 inches from barrier
	pros::delay(100);//wait
	mogo_lift.moveVelocity(mogo_lift_PID(-400));//pick up mogo
	rotate_PID(-37);//turn towards alliance mogo :)
	pros::delay(50);//pause
	translate_PID(40, 40);//move forward to alliance mogo
	four_bar_lift.moveVelocity(foud_bar_lift_PID(-500));//activates passivve intake
	translate_PID(-10, -10);//move mogo from the line
	chain_bar_lift.moveVelocity(chain_bar_PID(-1300));//setup for pneumatic release
	four_bar_lift.mvoveVelocity(four_bar_PID(-500));//setup for pneumatic release
	//pneumatic release




}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
	//translate_PID(80, 80);
	//rotate_PID(90);
	//rotate_PID(90);
	//translate_PID(80, 80);

	double chain_bar_setpoint = chain_bar.getPosition();    //Marks position of chain bar
	double four_bar_setpoint = four_bar_lift.getPosition(); //Marks position of four bar
	double mogo_lift_setpoint = mogo_lift.getPosition();		//Marks position of mogo lift

	while (true)
	{
		//Main drivetrain code
		drive -> getModel() -> tank(controller.getAnalog(ControllerAnalog::leftY),
																controller.getAnalog(ControllerAnalog::rightY));



		if (controller.getDigital(ControllerDigital::A) == 1)
		{
			if (A_buttonPress > 1) 														 //resets count
			{
				lift_macro_level = 0;
				A_buttonPress = 1;
			}
			else
			{
				A_buttonPress = A_buttonPress + 1;
			}
			pros::lcd::set_text(2, std::to_string(A_buttonPress));
			pros::delay(500); 																//delay so won't keep adding to count
		}


		//Macros for lift ------------------------------------------------------------------------------
		if (A_buttonPress == 1)
		{

			if (controller.getDigital(ControllerDigital::R1) == 1) //sets 4 different macros
			{

				if (lift_macro_level > 40) //max 50
				{
					lift_macro_level = 10;
				}
				else
				{
					lift_macro_level += 10; //incrememnts by 10, possible values (10, 20, 30, 40, 50)
				}
				//pros::lcd::set_text(5, std::to_string(R1_buttonPress));
				pros::delay(500);
			}

			if (controller.getDigital(ControllerDigital::R2) == 1)
			{
				if (lift_macro_level < 20) //min 10
				{
					lift_macro_level = 50;
				}
				else
				{
					lift_macro_level -= 10; //decrements by 10
				}
				pros::delay(500);
			}
			//pros::lcd::set_text(3, std::to_string(lift_macro_level));

			if (controller.getDigital(ControllerDigital::up) == 1)		//toggle between up and down position
			{
				if (mogo_macro_level < 1)
				{
					mogo_macro_level  = 1;
				}
				else
				{
					mogo_macro_level -= 1;
				}
				pros::delay(500);
			}

			if (lift_macro_level == 10) 												// first macro (neutral mogo, bottom branch)
			{
				chain_bar_PID_speed = chain_bar_PID(-130);
				four_bar_PID_speed = four_bar_PID(-500);

				chain_bar.moveVelocity(chain_bar_PID_speed);
				four_bar_lift.moveVelocity(four_bar_PID_speed);
			}
			else if (lift_macro_level == 20)										//second macro (neutral mogo, top branch)
			{
				chain_bar_PID_speed = chain_bar_PID(-430);
				four_bar_PID_speed = four_bar_PID(-500);

				chain_bar.moveVelocity(chain_bar_PID_speed);
				four_bar_lift.moveVelocity(four_bar_PID_speed);
			}
			else if (lift_macro_level == 30)										//thrid macro (alliance mogo)
			{
				chain_bar_PID_speed = chain_bar_PID(-1300);
				four_bar_PID_speed = four_bar_PID(-500);

				chain_bar.moveVelocity(chain_bar_PID_speed);
				four_bar_lift.moveVelocity(four_bar_PID_speed);
			}
			else if (lift_macro_level == 40)										//fourth macro (ring intake)
			{
				chain_bar_PID_speed = chain_bar_PID(-1200);
				four_bar_PID_speed = four_bar_PID(0);

				chain_bar.moveVelocity(chain_bar_PID_speed);
				four_bar_lift.moveVelocity(four_bar_PID_speed);
			}
			else if (lift_macro_level == 50 || lift_macro_level == 0)										//fifth macro (starting position)
			{
				chain_bar_PID_speed = chain_bar_PID(0);
				chain_bar.moveVelocity(chain_bar_PID_speed);

				four_bar_PID_speed = four_bar_PID(0);
				four_bar_lift.moveVelocity(four_bar_PID_speed);
			}

			//Macros for Mogo Lift ------------------------------------------------------------------------------
			if (mogo_macro_level == 0)
			{
				mogo_lift_speed = mogo_lift_PID(-500);
				mogo_lift.moveVelocity(mogo_lift_speed);
			}
			else if (mogo_macro_level == 1)
			{
				mogo_lift_speed = mogo_lift_PID(-420);
				mogo_lift.moveVelocity(mogo_lift_speed);
			}
		}

		else if (A_buttonPress == 2)												//user control lift
		{

			//Mogo Lift Buttons ------------------------------------------------------------------------------
			if (controller.getDigital(ControllerDigital::up) == 1)
			{
				if (mogo_lift.getPosition() > -10)
				{
					mogo_lift.moveVelocity(0);
					mogo_lift_setpoint = mogo_lift.getPosition();
				}
				else
				{
					mogo_lift.moveVelocity(100);
					mogo_lift_setpoint = mogo_lift.getPosition();
				}
			}
			else if (controller.getDigital(ControllerDigital::down) == 1)
			{
				if (mogo_lift.getPosition() < -490)
				{
					mogo_lift.moveVelocity(0);
					mogo_lift_setpoint = mogo_lift.getPosition();
				}
				else
				{
					mogo_lift.moveVelocity(-100);
					mogo_lift_setpoint = mogo_lift.getPosition();
				}
			}
			else if ((controller.getDigital(ControllerDigital::Y) == 0) and controller.getDigital(ControllerDigital::X) == 0)
			{
				mogo_lift_speed = mogo_lift_PID(mogo_lift_setpoint);
				mogo_lift.moveVelocity(mogo_lift_speed);
			}

			//Four Bar Buttons ------------------------------------------------------------------------------
			if (controller.getDigital(ControllerDigital::L2) == 1)
			{
				if (four_bar_lift.getPosition() > -5)
				{
					four_bar_lift.moveVelocity(0);
					four_bar_setpoint = four_bar_lift.getPosition();
				}
				else
				{
					four_bar_lift.moveVelocity(100);								//Moves four bar up when L1 is pressed
					four_bar_setpoint = four_bar_lift.getPosition();
				}
			}
			else if (controller.getDigital(ControllerDigital::L1) == 1)
			{
				if (four_bar_lift.getPosition() < -500)
				{
					four_bar_lift.moveVelocity(0);
					four_bar_setpoint = four_bar_lift.getPosition();
				}
				else
				{
					four_bar_lift.moveVelocity(-100);								//Moves four bar down when L2 is pressed
					four_bar_setpoint = four_bar_lift.getPosition();
				}
			}
			else if((controller.getDigital(ControllerDigital::L1) == 0) and controller.getDigital(ControllerDigital::L2) == 0)
			{
				four_bar_speed = four_bar_PID(four_bar_setpoint);
				four_bar_lift.moveVelocity(four_bar_speed);
			}


			//Chain Bar Buttons ------------------------------------------------------------------------------
			if (controller.getDigital(ControllerDigital::R2) == 1)
			{
				if (chain_bar.getPosition() > -10)								//Stops chain bar from going under starting position
				{
					chain_bar.moveVelocity(0);
					chain_bar_setpoint = chain_bar.getPosition();
				}
				else
				{
					chain_bar.moveVelocity(75);										//Moves chain bar down when R2 is pressed
					chain_bar_setpoint = chain_bar.getPosition();
				}
			}

			else if (controller.getDigital(ControllerDigital::R1) == 1)
			{
				if (chain_bar.getPosition() < -1175)						//Stops chain bar from going over maximum position
				{
					chain_bar.moveVelocity(0);
					chain_bar_setpoint = chain_bar.getPosition();
				}
				else
				{
					chain_bar.moveVelocity(-75);									//Moves chain bar up when R1 is pressed
					chain_bar_setpoint = chain_bar.getPosition();
				}
			}
			else if (controller.getDigital(ControllerDigital::R2) == 0 && (controller.getDigital(ControllerDigital::R1) == 0))
			{
				chain_bar_speed = chain_bar_PID(chain_bar_setpoint);
				chain_bar.moveVelocity(chain_bar_speed);				//Keeps chain bar in the air when nothing is pressed
			}
			//pros::lcd::set_text(4, std::to_string(chain_bar.getPosition()));
		}

		if (controller.getDigital(ControllerDigital::B) == 1)				//automated platform balancing
		{
			inertial_sensor.reset();
			drive -> getModel() -> resetSensors();

			while (true)
			{
				inertial_values = inertial_sensor.get();
			}
		}

		pros::delay(20);

	}
}
//https://github.com/ananthgoyal/TippingPoint/tree/master/src
