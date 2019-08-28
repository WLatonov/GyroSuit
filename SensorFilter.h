#ifndef SensorFilter_h
#define SensorFilter_h
#define M_PI 3.14159265358979323846

#include <math.h>
#include "INS_worker.h"
#include <unordered_map>

class SensorFilter
{
    enum
    {
        MaxFilterSize     = 110,
        DefaultFilterSize = 20
    };

private:
    int         LastIdx;                    
    int         Size;                       
    vector3    Elements[110];
	quat Addition50;
	quat Prediction;

public:
    // Create a new filter with default size
    SensorFilter() 
    {
        LastIdx = -1;
        Size = DefaultFilterSize;
		for (int i = 0; i < 110; ++i)
		{
			Elements[i] = { 0.0, 0.0, 0.0 };
		}
		Addition50 = { 1.0, 0.0, 0.0, 0.0 };
		Prediction = { 1.0, 0.0, 0.0, 0.0 };
    };

    // Create a new filter with size i
    SensorFilter(int i) 
    {
        LastIdx = -1;
		if (i > MaxFilterSize) { i = MaxFilterSize; }
        Size = i;
		for (int i = 0; i < 110; ++i)
		{
			Elements[i] = { 0.0, 0.0, 0.0 };
		}
		Addition50 = { 1.0, 0.0, 0.0, 0.0 };
		Prediction = { 1.0, 0.0, 0.0, 0.0 };
    };


    // Create a new element to the filter
    void AddElement (const vector3& e) 
    {
		if (LastIdx == Size - 1) { LastIdx = 0; }
		else { LastIdx++; }

        Elements[LastIdx] = e;
    };

    // Get element i.  0 is the most recent, 1 is one step ago, 2 is two steps ago, ...
    vector3 GetPrev(int i) const
    {

        int idx = (LastIdx - i);
        if (idx < 0) // Fix the wraparound case
            idx += Size;
        return Elements[idx];
    };

	quat GetAdd()
	{
		return Addition50;
	}

	quat GetPrediction()
	{
		return Prediction;
	}

    // Simple statistics
    vector3 Total();
    vector3 Mean();
	vector3 MeanN(int s);
    vector3 Median();
    vector3 Variance(); // The diagonal of covariance matrix
	vector3 MeanVariance(double delta);
	vector3 MeanVariance50(double delta);
	vector3 MeanVarianceUniversal(int N, double delta);
	vector3 MeanBandVariance(double delta1, double delta2);
	vector3 LowPass_Left_Lancosh(int N, double high_freq);
	vector3 LowPass_Left_Lancosh_Latency(int N, double high_freq, int latency);

    // A popular family of smoothing filters and smoothed derivatives
    vector3 SavitzkyGolaySmooth8();
    vector3 SavitzkyGolayDerivative4();
    vector3 SavitzkyGolayDerivative5();
    vector3 SavitzkyGolayDerivative12(); 
    vector3 SavitzkyGolayDerivativeN(int n);

    ~SensorFilter() {};

	// Коллекция прогнозов
	void IntegrateAddition();
	void IntegrateLinearPrediction(int P, int N);
	void IntegrateLinearPrediction_ConstAngVel(int Prev, int N);
	void IntegrateLinearPrediction_Holt_1(int N);
	void IntegrateLinearPrediction_Holt_2(int N);
	void IntegrateLinearPrediction_Holt_5(int N);

	void SetPredictionToZero();
};

#endif 
