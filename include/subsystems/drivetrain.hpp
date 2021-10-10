#include "main.h"

using namespace okapi;

extern Motor leftBack;
extern Motor leftFront;
extern Motor rightBack;
extern Motor rightFront;
extern ADIEncoder encoder;

void updateDrive();

void odometry(double x_pos, double y_pos);
