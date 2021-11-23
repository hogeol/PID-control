#include "trapezoid_trajectory.hpp"

void trapezoid_coeff(double start_position, double final_position, double max_velocity, double max_accelerate, double& accel_time, double& constant_time, double& final_time)
{
	double position = final_position - start_position;
	if (position <= SQR(max_velocity) / max_accelerate)
	{
		accel_time = sqrt((position) / max_accelerate);
		constant_time = 0.0;
	}
	else
	{
		accel_time = max_velocity / max_accelerate;
		constant_time = (position - SQR(max_velocity) / max_accelerate) / max_velocity;
	}
	final_time = constant_time + 2 * accel_time;
}

motion trapezoidar(double start_position, double final_position, double max_velocity, double max_accelerate, double accel_time, double final_time, double constant_time, double current_time)
{
	motion pva;
	if (current_time <= accel_time)
	{
		pva.acceleration = max_accelerate;
		pva.velocity = max_accelerate * current_time;
		pva.position = start_position + max_accelerate * SQR(current_time) / 2.0;
	}
	else if (current_time <= constant_time + accel_time)
	{
		pva.acceleration = 0.0;
		pva.velocity = max_velocity;
		pva.position = start_position - max_accelerate * SQR(accel_time) / 2.0 + max_velocity * current_time;
	}
	else if (current_time < final_time)
	{
		double tmp = final_time - current_time;
		pva.acceleration = -max_accelerate;
		pva.velocity = max_accelerate * (final_time - current_time);
		pva.position = final_position - (max_accelerate * SQR(tmp) / 2.0);
	}
	else
	{
		pva.acceleration = 0.0;
		pva.velocity = 0.0;
		pva.position = final_position;
	}

	return pva;
}
