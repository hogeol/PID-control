#ifndef __TRAPETRAJEC__
#define __TRAPETRAJEC__

#include <cmath>
#define	SQR(x) ((x)*(x))

struct motion
{
	double position;
	double velocity;
	double acceleration;
};

void trapezoid_coeff(double start_position, double final_position, double max_velocity, double max_accelerate, double& accel_time, double& constant_time, double& final_time);
motion trapezoidar(double start_position, double final_position, double max_velocity, double max_accelerate, double accel_time, double final_time, double constant_time, double current_time);

#endif __TRAPETRAJEC__