#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <cstdio>
#include "PID_control.hpp"
#include "trapezoid_trajectory.hpp"

#define PI 3.141592
#define TIMESEQEUNCE 0.002
#define KP 0.006
#define KI 0.6
#define KD 0.002

using namespace std;

int main(int argc, char** argv)
{
	FILE* fp = fopen("PID.txt", "w");
	double initial_pose = PI / 4, final_pose = 3 * PI / 4;
	double max_velocity = PI / 6, max_acceleration = PI / 6;
	double accel_time = 0.0, constant_time = 0.0, final_time = 0.0;
	double current_time = 0.0;
	double current_pose = PI / 4, target_pose = 0.0;
	double target_velocity = 0.0;
	//Trajectory planning by trapezoid method
	trapezoid_coeff(initial_pose, final_pose, max_velocity, max_acceleration, accel_time, constant_time, final_time);

	motion trajectory[3000];
	PID_control pcon(initial_pose, final_pose);
	pcon.Set_K_parameters(KP, KI, KD);
	int tmp = 0;
	cout << "\tTarget Pose\tCurrent Pose\n\n";
	while (current_time <= final_time)
	{
		trajectory[tmp] = trapezoidar(initial_pose, final_pose, max_velocity, max_acceleration, accel_time, final_time, constant_time, current_time); //Goal trajectory
		target_pose = trajectory[tmp].position;
		current_pose += pcon.Get_force();
		target_velocity = trajectory[tmp].velocity;
		pcon.Set_calculus(current_pose, target_pose, target_velocity, TIMESEQEUNCE);
		tmp++;
		fprintf(stdout, "\t%.4f\t\t%.4f\n", target_pose, current_pose);
		fprintf(fp, "\t%.4f\t\t%.4f\n", target_pose, current_pose);
		current_time += TIMESEQEUNCE;
	}
	fclose(fp);
	return 0;
}