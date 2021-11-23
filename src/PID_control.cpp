#include "PID_control.hpp"

//Set the Kp, Ki, Kd parameters by user
void PID_control::Set_K_parameters(double Kp, double Ki, double Kd)
{
	mKp = Kp;
	mKi = Ki;
	mKd = Kd;
}

//Calculate motion error
double PID_control::Calculate_error(double current_pose, double target_pose, double current_deviation, double target_deviation)
{
	mprev_error = merror;
	merror = target_pose - current_pose;
	double deviation_error = target_deviation - current_deviation;

	return deviation_error;
}

//Set PID value
void PID_control::Set_force(double err_integral, double err_deviation)
{
	mtau = mKp * merror + mKi * err_integral + mKd * err_deviation;
}

//Get PID value
double PID_control::Get_force()
{
	return mtau;
}

//Set integral and deviation values
void PID_control::Set_calculus(double current_pose, double target_pose, double target_deviation, double sequence)
{
	double current_deviation = (current_pose - mprev_pose) / sequence;
	double err_deviation = Calculate_error(current_pose, target_pose, current_deviation, target_deviation);
	maccumulated_error += ((merror + mprev_error) * sequence) / 2;
	double err_integral = maccumulated_error;
	Set_force(err_integral, err_deviation);
	mprev_pose = current_pose;
}

