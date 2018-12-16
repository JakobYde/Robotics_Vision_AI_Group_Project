#pragma once

#include <vector>

class RandFloat
{
public:
	enum DistributionType {
		Uniform, IrwinHall
	};

	RandFloat(double min, double max, DistributionType eType, int N = 3) { 
		val = (RandFloat(eType, N) * (max - min)) + min; 
	}

	RandFloat(DistributionType eType = Uniform, int N = 3) {
		switch (eType) {
		case IrwinHall:
			for (int i = 0; i < N * 5; i++) val += RandFloat(Uniform, N);
			val /= N * 5;
			break;

		default:
			val = fmod((double)rand() / pow(10, N), 1.0);
		} 
	}
	~RandFloat() {};

	operator double() {
		return val;
	}

	double val = 0;
};

