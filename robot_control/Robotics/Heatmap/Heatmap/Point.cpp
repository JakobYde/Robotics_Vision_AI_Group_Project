#include "Point.h"
PolarPoint::PolarPoint()
{

}

PolarPoint::PolarPoint(double d, double angle) : d(d), angle(angle)
{

}

Point<double> PolarPoint::asPoint() {
	return Point<double>(d * cos(angle), d * sin(angle));
}

Point<unsigned int> PolarPoint::asUIntPoint() {
	return Point<unsigned int>(round(d * cos(angle)), round(d * sin(angle)));
}