#include "Transformator.h"

Transformator::Transformator() {

}

Transformator::Transformator(INT64  SerialNumber){
	std::wifstream StrmS(L"INS_SimpleCalibration_" + std::to_wstring(SerialNumber) + L".txt");
		StrmS >> accBiasX >> accBiasY >> accBiasZ;
		StrmS >> accScaleFactorX >> accScaleFactorY >> accScaleFactorZ;
		StrmS >> gyrBiasX >> gyrBiasY >> gyrBiasZ;
		StrmS >> gyrScaleFactorX >> gyrScaleFactorY >> gyrScaleFactorZ;
		StrmS >> magBiasX >> magBiasY >> magBiasZ;
		StrmS >> magScaleFactorX >> magScaleFactorY >> magScaleFactorZ;

		if (abs(accBiasX) > 0.001) std::wcout << "Got simple calibration file " << std::to_wstring(SerialNumber) << endl;
}

Transformator::~Transformator(){

}

void Transformator::SimpleTransform(INS_Data& a)
{
	a.acc.x = (a.acc.x + accBiasX) * accScaleFactorX;
	a.acc.y = (a.acc.y + accBiasY) * accScaleFactorY;
	a.acc.z = (a.acc.z + accBiasZ) * accScaleFactorZ;

	a.gyr.x = (a.gyr.x - gyrBiasX) * gyrScaleFactorX;
	a.gyr.y = (a.gyr.y - gyrBiasY) * gyrScaleFactorY;
	a.gyr.z = (a.gyr.z - gyrBiasZ) * gyrScaleFactorZ;

	a.mag.x = (a.mag.x + magBiasX) * magScaleFactorX;
	a.mag.y = (a.mag.y + magBiasY) * magScaleFactorY;
	a.mag.z = (a.mag.z + magBiasZ) * magScaleFactorZ;

	return;
}