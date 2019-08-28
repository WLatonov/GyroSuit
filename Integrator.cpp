#include "Integrator.h"
#include "Settings.h"
//#define DEBUG_PRINT
extern Settings settings;
double  Integrator::GravityEpsilon = 0.8, Integrator::AngVelEpsilon = 0.05;
const double Integrator::alpha_exp_coeff = 1.0 - 1.0 / 64.0;

void Integrator::GyroStopDetector(const INS_Data& data) {
	vector3 DeltaGyr = GyroscopesStop.MeanVariance50(Stopdetectvar);

	//if (DeltaGyr.x >= maxNumberofDevGyr) { data.gyr.x = 0.0; }
	//if (DeltaGyr.y >= maxNumberofDevGyr) { data.gyr.y = 0.0; }
	//if (DeltaGyr.z >= maxNumberofDevGyr) { data.gyr.z = 0.0; }
}

void Integrator::Step(const INS_Data& data) {
	gyr_exp = gyr_exp*alpha_exp_coeff + data.gyr*(1.0 - alpha_exp_coeff);
	acc_exp = acc_exp*alpha_exp_coeff + data.acc*(1.0 - alpha_exp_coeff);
	gyr_exp_disp = gyr_exp_disp*alpha_exp_coeff + (data.gyr - gyr_exp).DotProduct(data.gyr - gyr_exp)*(1.0 - alpha_exp_coeff);
	acc_exp_disp = acc_exp_disp*alpha_exp_coeff + (data.acc - acc_exp).DotProduct(data.acc - acc_exp)*(1.0 - alpha_exp_coeff);

	GyroscopesStop.AddElement(data.gyr);
	vector3 gyr = data.gyr * settings.INS_gyr_coeff;
	if (gyroStopDetector) { GyroStopDetector(data); }


	avg_g = avg_g + data.acc; 
	vector3 acc = q.Rotate(data.acc) - grav;
	avg_acc = avg_acc + data.acc;

	pos.Integrate(vel, acc, 1e-3); vel.Integrate(acc, 1e-3); q.Integrate(gyr, 1e-3); 

	if (counter++ % N == 0) {
		avg_g = avg_g / N; avg_acc = avg_acc / N;
		avg_g_last = avg_g;
		if (!is_start) {
			HardCorrection(avg_g);
			is_start = true;
		}
		if ((sqrt(acc_exp_disp) < GravityEpsilon) && (sqrt(gyr_exp_disp) < AngVelEpsilon) && (gyr_exp.Norm() < AngVelEpsilon)) {
#ifdef DEBUG_PRINT
			cout << "#";
#endif
			tiltCounter++;
		} else {
			tiltCounter = 0;
		}
		if (tiltCounter > 20) {
#ifdef DEBUG_PRINT
			cout << "(*)";
#endif
			SoftCorrection(avg_g);
		}
#ifdef DEBUG_PRINT
		cout << std::fixed << std::setw(5) << std::setprecision(2) << q[0] << "\t" << q[1] << "\t" << q[2] << "\t" << q[3] << std::setprecision(3) << "\t" << sqrt(acc_exp_disp) << "\t" << sqrt(gyr_exp_disp) << "\t" << gyr_exp.Norm() << endl;
#endif
		avg_acc = avg_g = { 0, 0, 0 };
	}

}