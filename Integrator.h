#pragma once
#include "utils.h"
#include "INS_worker.h"
#include "SensorFilter.h"
#define  Stopdetectvar         0.0008
#define  maxNumberofDevGyr     50.0 
class Integrator {
public:
	static double  GravityEpsilon /*= 0.8*/, AngVelEpsilon /*= 0.05*/;
	static const double alpha_exp_coeff;// = 1.0 - 1.0 / 64.0;
	static const int N = 50;

	bool gyroStopDetector = true;

	bool is_start = false;
	int counter = 0, tiltCounter = 0;
	vector3 pos = vector3{ 0, 0, 0 }, vel = vector3{ 0, 0, 0 }, grav = vector3{ 0, 0, -9.81 }, avg_g_last = vector3{ 0, 0, -9.81 };
	vector3 gyr_exp = vector3{ 0, 0, 0 }, acc_exp = vector3{ 0, 0, 0 }, avg_g = vector3{ 0, 0, 0 }, avg_acc = vector3{ 0, 0, 0 };
	double gyr_exp_disp = 0, acc_exp_disp = 0;
	quat q = quat{ 1, 0, 0, 0 };


	SensorFilter GyroscopesStop = SensorFilter(110);

	void GyroStopDetector(const INS_Data& data);

	void Step(const INS_Data& data);
	void HardCorrection(vector3 estimate_grav) {
		quat dq = quat::RotationFromTo(q.Rotate(estimate_grav), grav);
		q = dq * q;
	}
	void SoftCorrection(vector3 estimate_grav) {
		quat dq = quat::RotationFromToOculus(q.Rotate(estimate_grav), grav);
		q = dq * q;

	}
};

