#include "SensorFilter.h"
#include "math.h"

vector3 SensorFilter::Total()
{
	vector3 total = { 0.0, 0.0, 0.0 };
    for (int i = 0; i < Size; i++)
        total += Elements[i];
    return total;
}

vector3 SensorFilter::Mean()
{
	vector3 total = { 0.0, 0.0, 0.0 };
    for (int i = 0; i < Size; i++)
        total += Elements[i];

    return total / (double) Size;
}

vector3 SensorFilter::MeanN(int s)
{
	if (s > Size) { s = Size; }
	vector3 total = { 0.0, 0.0, 0.0 };
	for (int i = 0; i < s; i++)
		total += Elements[i];

	return total / (double)s;
}

vector3 SensorFilter::Median()
{
    int half_window = (int) Size / 2;
    double sortx[MaxFilterSize];
    double resultx = 0.0f;

	double sorty[MaxFilterSize];
	double resulty = 0.0f;

	double sortz[MaxFilterSize];
	double resultz = 0.0f;

    for (int i = 0; i < Size; i++) 
    {
        sortx[i] = Elements[i].x;
        sorty[i] = Elements[i].y;
        sortz[i] = Elements[i].z;
    }
    for (int j = 0; j <= half_window; j++) 
    {
        int minx = j;
        int miny = j;
        int minz = j;
        for (int k = j + 1; k < Size; k++) 
        {
            if (sortx[k] < sortx[minx]) minx = k;
            if (sorty[k] < sorty[miny]) miny = k;
            if (sortz[k] < sortz[minz]) minz = k;
        }
        double tempx = sortx[j];
        double tempy = sorty[j];
        double tempz = sortz[j];
        sortx[j] = sortx[minx];
        sortx[minx] = tempx;

        sorty[j] = sorty[miny];
        sorty[miny] = tempy;

        sortz[j] = sortz[minz];
        sortz[minz] = tempz;
    }
    resultx = sortx[half_window];
    resulty = sorty[half_window];
    resultz = sortz[half_window];

	return{ resultx, resulty, resultz };
}

//  Only the diagonal of the covariance matrix.
vector3 SensorFilter::Variance()
{
    vector3 mean = Mean();
	vector3 total = { 0.0, 0.0, 0.0 };
	for (int i = 0; i < Size; i++)
    {
        total.x += (Elements[i].x - mean.x) * (Elements[i].x - mean.x);
        total.y += (Elements[i].y - mean.y) * (Elements[i].y - mean.y);
        total.z += (Elements[i].z - mean.z) * (Elements[i].z - mean.z);
    }
    return total / (double) Size;
}


vector3 SensorFilter::MeanVariance(double delta)
{
	vector3 total = { 0.0, 0.0, 0.0 };
	for (int i = 0; i < Size; i++)
	{
		total += Elements[i];
	}
	vector3 Mean = total / (double)Size;
	int CountCorrectionX = 0;
	int CountCorrectionY = 0;
	int CountCorrectionZ = 0;
	for (int i = 0; i < Size; i++)
	{
		if (fabs(Mean.x - Elements[i].x) < delta)
		{
			++CountCorrectionX;
		}

		if (fabs(Mean.y - Elements[i].y) < delta)
		{
			++CountCorrectionY;
		}

		if (fabs(Mean.z - Elements[i].z) < delta)
		{
			++CountCorrectionZ;
		}
	}
	return{ (double)CountCorrectionX, (double)CountCorrectionY, (double)CountCorrectionZ };
}

vector3 SensorFilter::MeanVariance50(double delta)
{
	vector3 total = { 0.0, 0.0, 0.0 };
	for (int i = 0; i < 50; i++)
	{
		total += Elements[i];
	}
	vector3 Mean = total / 50.0;
	int CountCorrectionX = 0;
	int CountCorrectionY = 0;
	int CountCorrectionZ = 0;
	for (int i = 0; i < 50; i++)
	{
		if (fabs(Mean.x - Elements[i].x) < delta)
		{
			++CountCorrectionX;
		}

		if (fabs(Mean.y - Elements[i].y) < delta)
		{
			++CountCorrectionY;
		}

		if (fabs(Mean.z - Elements[i].z) < delta)
		{
			++CountCorrectionZ;
		}
	}
	return{ (double)CountCorrectionX, (double)CountCorrectionY, (double)CountCorrectionZ };
}

vector3 SensorFilter::MeanVarianceUniversal(int N, double delta)
{
	vector3 total = { 0.0, 0.0, 0.0 };
	for (int i = 0; i < N; i++)
	{
		total += Elements[i];
	}
	vector3 Mean = total / (double)N;
	int CountCorrectionX = 0;
	int CountCorrectionY = 0;
	int CountCorrectionZ = 0;
	for (int i = 0; i < N; i++)
	{
		if (fabs(Mean.x - Elements[i].x) < delta)
		{
			++CountCorrectionX;
		}

		if (fabs(Mean.y - Elements[i].y) < delta)
		{
			++CountCorrectionY;
		}

		if (fabs(Mean.z - Elements[i].z) < delta)
		{
			++CountCorrectionZ;
		}
	}
	return{ (double)CountCorrectionX, (double)CountCorrectionY, (double)CountCorrectionZ };
}

vector3 SensorFilter::MeanBandVariance(double delta1, double delta2)
{
	vector3 total = { 0.0, 0.0, 0.0 };
	for (int i = 0; i < Size; i++)
	{
		total += Elements[i];
	}
	vector3 Mean = total / (double)Size;
	int CountCorrectionX = 0;
	int CountCorrectionY = 0;
	int CountCorrectionZ = 0;
	for (int i = 0; i < Size; i++)
	{
		if (fabs(Mean.x - Elements[i].x) > delta1 && fabs(Mean.x - Elements[i].x) < delta2)
		{
			++CountCorrectionX;
		}

		if (fabs(Mean.y - Elements[i].y) > delta1 && fabs(Mean.y - Elements[i].y) < delta2)
		{
			++CountCorrectionY;
		}

		if (fabs(Mean.z - Elements[i].z) > 1.5*delta1 && fabs(Mean.z - Elements[i].z) < 1.5*delta2)
		{
			++CountCorrectionZ;
		}
	}
	return{ (double)CountCorrectionX, (double)CountCorrectionY, (double)CountCorrectionZ };
}

vector3 SensorFilter::LowPass_Left_Lancosh(int N, double high_freq)
{
	if (N > Size) { N = Size; }
	vector3 total = { 0.0, 0.0, 0.0 };
	double k = 0.0;
	for (int i = 0; i < N; i++)
	{
		if (i == 0)
		{
			k = (high_freq / M_PI);
		}
		else if(i > 0)
		{
			k = (sin(high_freq * (double)i) / (M_PI * (double)i)) * (sin(M_PI * (double)i / (double)N) / (M_PI * (double)i / (double)N));
		}

		total += GetPrev(i) * k;
	}
	return total;
}

vector3 SensorFilter::LowPass_Left_Lancosh_Latency(int N, double high_freq, int latency)
{
	if (N + N > Size) { N = (int)(Size / 2); }
	if (N > latency) { N = latency; }
	vector3 total = { 0.0, 0.0, 0.0 };

	for (int i = - N; i < N + 1; i++)
	{
		double k = 0.0;
		if (i == 0)
		{
			k = (high_freq / M_PI);
		}
		else if (i != 0)
		{
			k = (sin(high_freq * ((double)i)) / (M_PI * ((double)i))) * (sin(M_PI * (double)i /((double)N)) / (M_PI * (double)i / ( (double)N)));
		}

		total += GetPrev(i + latency) * k;
	};
	return total;
}

vector3 SensorFilter::SavitzkyGolaySmooth8()
{

    return GetPrev(0)*0.41667f +
            GetPrev(1)*0.33333f +
            GetPrev(2)*0.25f +
            GetPrev(3)*0.16667f +
            GetPrev(4)*0.08333f -
            GetPrev(6)*0.08333f -
            GetPrev(7)*0.16667f;
}


vector3 SensorFilter::SavitzkyGolayDerivative4()
{

    return GetPrev(0)*0.3f +
            GetPrev(1)*0.1f -
            GetPrev(2)*0.1f -
            GetPrev(3)*0.3f;
}

vector3 SensorFilter::SavitzkyGolayDerivative5()
{

    return GetPrev(0)*0.2f +
            GetPrev(1)*0.1f -
            GetPrev(3)*0.1f -
            GetPrev(4)*0.2f;
}

vector3 SensorFilter::SavitzkyGolayDerivative12()
{

    return GetPrev(0)*0.03846f +
            GetPrev(1)*0.03147f +
            GetPrev(2)*0.02448f +
            GetPrev(3)*0.01748f +
            GetPrev(4)*0.01049f +
            GetPrev(5)*0.0035f -
            GetPrev(6)*0.0035f -
            GetPrev(7)*0.01049f -
            GetPrev(8)*0.01748f -
            GetPrev(9)*0.02448f -
            GetPrev(10)*0.03147f -
            GetPrev(11)*0.03846f;
}

vector3 SensorFilter::SavitzkyGolayDerivativeN(int n)
{    

    int m = (n-1)/2;
    vector3 result = vector3();
    for (int k = 1; k <= m; k++) 
    {
        int ind1 = m - k;
        int ind2 = n - m + k - 1;
        result += (GetPrev(ind1) - GetPrev(ind2)) * (double) k;
    }
    double coef = 3.0f/(m*(m+1.0f)*(2.0f*m+1.0f));
    result = result*coef;
    return result;
}

void SensorFilter::IntegrateAddition()
{
	Addition50.Integrate(GetPrev(50)*(-1.0), 1e-3);
	Addition50.Integrate(GetPrev(0), 1e-3);
}

void SensorFilter::IntegrateLinearPrediction(int P, int N)
{
	Prediction = { 1.0, 0.0, 0.0, 0.0 };
	if (P % 2 != 0) { ++P; }
	if (P > Size) { P = Size; }
	vector3 First = { 0.0, 0.0, 0.0 };
	vector3 Second = { 0.0, 0.0, 0.0 };
	for (int k = 0; k < P / 2; ++k)
	{
		First += GetPrev(k);
		Second += GetPrev(k + P / 2);
	}
	First = First * 2.0 / (double)P;
	Second = Second * 2.0 / (double)P;

	vector3 tan = (Second - First) * 2.0 / (double)P;
	for (int k = 0; k < N; ++k)
	{
		Prediction.Integrate(GetPrev(0) + tan*(double)k*(0.001), 1e-3);
	}


}

void SensorFilter::IntegrateLinearPrediction_ConstAngVel(int Prev, int N)
{
	Prediction = { 1.0, 0.0, 0.0, 0.0 };

	for (int k = 0; k < N; ++k)
	{
		Prediction.Integrate(GetPrev(Prev), 1e-3);
	}
}

void SensorFilter::IntegrateLinearPrediction_Holt_1(int N)
{
	Prediction = { 1.0, 0.0, 0.0, 0.0 };
	vector3 X = GetPrev(2), XN = GetPrev(1);

	double alpha = 0.5;

	for (int k = 0; k < N; ++k)
	{
		vector3 Est = XN * alpha + X * (1 - alpha);
		Prediction.Integrate(Est, 1e-3);
		X = XN; XN = Est;
	}
}

void SensorFilter::IntegrateLinearPrediction_Holt_2(int N)
{
	Prediction = { 1.0, 0.0, 0.0, 0.0 };
	vector3 XP = GetPrev(3), X = GetPrev(2), XN = GetPrev(1);

	double alpha = 0.5;

	for (int k = 0; k < N; ++k)
	{
		vector3 Est = XN * alpha + X * (1 - alpha) * alpha + XP * (1-alpha) * (1-alpha);
		Prediction.Integrate(Est, 1e-3);
		XP = X; X = XN; XN = Est;
	}
}

void SensorFilter::IntegrateLinearPrediction_Holt_5(int N)
{
	Prediction = { 1.0, 0.0, 0.0, 0.0 };
	vector3 XPPPP = GetPrev(6), XPPP = GetPrev(5), XPP = GetPrev(4), XP = GetPrev(3), X = GetPrev(2), XN = GetPrev(1);

	double alpha = 0.5;

	for (int k = 0; k < N; ++k)
	{
		vector3 Est = (XN + X * (1 - alpha) + XP * pow(1 - alpha, 2) + XPP * pow(1 - alpha, 3) + XPPP * pow(1 - alpha, 4)) * alpha + XPPPP * pow(1 - alpha, 5);
		Prediction.Integrate(Est, 1e-3);
		XP = X; X = XN; XN = Est;
	}
}

void SensorFilter::SetPredictionToZero()
{
	Prediction = { 1.0, 0.0, 0.0, 0.0 };
}
