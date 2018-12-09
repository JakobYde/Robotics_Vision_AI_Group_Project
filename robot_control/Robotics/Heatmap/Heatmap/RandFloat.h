#pragma once

#include <vector>

class RandFloat
{
public:
	RandFloat() { *this = RandFloat(3); }
	RandFloat(int N) { val = fmod((double)rand() / pow(10, N), 1.0); }
	~RandFloat() {};

	operator double() {
		return val;
	}

	double val;
};

