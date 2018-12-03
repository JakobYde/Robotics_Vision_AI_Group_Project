#include "Point.h"
#include "Edge.h"

PolarPoint::PolarPoint()
{

}

PolarPoint::PolarPoint(double d, double angle) : d(d), angle(angle)
{

}

Point PolarPoint::asPoint() {
	return Point(d * cos(angle), d * sin(angle));
}

double Point::getDistance(Edge e)
{
	std::vector<Point> pts = e.getPoints();
	double dist = INT_MAX;
	for (Point p : pts) dist = MIN(dist, getDistance(p));
	return dist;
}

double Point::getDistance(Line l)
{
	double x, y, tA = tan(l.a);

	x = (X + Y * tA) / (1 + tA * tA);
	y = x * tA;
	Point parallelPoint(x, y);
	return abs(l.d - parallelPoint.getDistance(ORIGIN));
}
