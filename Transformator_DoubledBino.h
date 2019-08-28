#pragma once
#include "Transformator.h"
#include "utils.h"

class Transformator_DoubledBino :
	public Transformator
{
public:
	Transformator_DoubledBino();
	Transformator_DoubledBino(INT64 SerialNumber);
	~Transformator_DoubledBino();

	vector3 rotationAxis = {0.0, 0.0, 1.0};
};

