#ifndef __PID_CONTROL__
#define __PID_CONTROL__

class PID_control
{
private:
	double minitial_pose;
	double mfinal_pose;
	double mKp = 0.0, mKi = 0.0, mKd = 0.0;
	double merror = 0.0;
	double mprev_pose;
	double mprev_error = 0.0;
	double maccumulated_error = 0.0;
	double mtau = 0.0;
public:
	PID_control(double initial_pose, double final_pose) :minitial_pose(initial_pose), mprev_pose(initial_pose), mfinal_pose(final_pose) {}
	void Set_K_parameters(double Kp, double Ki, double Kd);
	double Calculate_error(double current_pose, double target_pose,double current_deviation, double target_deviation);
	void Set_force(double err_integral, double err_deviation);
	double Get_force();
	void Set_calculus(double current_pose, double target_pose, double target_deviation, double sequence);
};

#endif __PID_CONTROL__