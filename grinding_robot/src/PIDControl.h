#pragma once
#include <iostream>
#include <cmath>
class PIDControl
{
public:
	PIDControl(double dt, double max, double min, double Kp, double Kd, double Ki);
	double pidCalculate(double setpoint, double pValue);
	~PIDControl();


private:
	double _dt=1.0;
	double _max;
	double _min;
	double _Kp;
	double _Kd;
	double _Ki;
	double _pre_error;
	double _integral;
};


