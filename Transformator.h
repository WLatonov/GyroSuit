#pragma once
#include "MyDefProject.h"
#include "INS_worker.h"

class Transformator
{
public:
	//Simple calibration coefficients
	double accBiasX = 0.0, accBiasY = 0.0, accBiasZ = 0.0;
	double accScaleFactorX = 0.0, accScaleFactorY = 0.0, accScaleFactorZ = 0.0;
	double gyrBiasX = 0.0, gyrBiasY = 0.0, gyrBiasZ = 0.0;
	double gyrScaleFactorX = 0.0, gyrScaleFactorY = 0.0, gyrScaleFactorZ = 0.0;
	double magBiasX = 0.0, magBiasY = 0.0, magBiasZ = 0.0;
	double magScaleFactorX = 0.0, magScaleFactorY = 0.0, magScaleFactorZ = 0.0;

	Transformator();
	Transformator(INT64  SerialNumber);
	~Transformator();

	void SimpleTransform(INS_Data& a);
};

