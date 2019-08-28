#include "Transformator_DoubledBino.h"

Transformator_DoubledBino::Transformator_DoubledBino(){

}

Transformator_DoubledBino::Transformator_DoubledBino(INT64 SerialNumber) {
	
	std::wifstream StrmS(L"INS_SimpleCalibration_" + std::to_wstring(SerialNumber) + L".txt");
	StrmS >> accBiasX >> accBiasY >> accBiasZ;
	StrmS >> accScaleFactorX >> accScaleFactorY >> accScaleFactorZ;
	StrmS >> gyrBiasX >> gyrBiasY >> gyrBiasZ;
	StrmS >> gyrScaleFactorX >> gyrScaleFactorY >> gyrScaleFactorZ;
	StrmS >> magBiasX >> magBiasY >> magBiasZ;
	StrmS >> magScaleFactorX >> magScaleFactorY >> magScaleFactorZ;
	StrmS >> rotationAxis.x >> rotationAxis.y >> rotationAxis.z;

	if (abs(accBiasX) > 0.001) cout << "Got simple calibration file " << SerialNumber << endl;
}

Transformator_DoubledBino::~Transformator_DoubledBino(){

}
