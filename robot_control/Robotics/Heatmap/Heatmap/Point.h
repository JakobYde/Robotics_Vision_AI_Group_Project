#pragma once
#include <vector>

template <typename T>
class Point;

class PolarPoint 
{
private:
	double d, angle;

public:
	PolarPoint();
	PolarPoint(double d, double angle);
	Point<double> asPoint();
	Point<unsigned int> asUIntPoint();

};

template <typename T>
class Point
{
private:
	T X, Y;

public:
	Point() {}
	Point(T x, T y) : X(x), Y(y) {}
	~Point() {}

	T& x() { 
		return X; 
	}

	T& y() {
		return Y;
	}

	Point<T> operator-(Point<T> p) {
		return Point<T>(X - p.x(), Y - p.y());
	}

	Point<T> operator+(Point<T> p) {
		return Point<T>(X + p.x(), Y + p.y());
	}

	bool operator!=(Point<T> p) {
		return (X != p.x() || Y != p.y());
	}

	bool operator==(Point<T> p) {
		return (X == p.x() && Y == p.y());
	}

	PolarPoint asPolar() {
		return PolarPoint(sqrt(X*X+Y*Y), atan((double)Y / (double)X));
	}
};


