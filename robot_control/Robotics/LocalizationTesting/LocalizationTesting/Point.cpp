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

PolarPoint::operator Point()
{
	return Point(d * cos(angle), d * sin(angle));
}

std::vector<Point> PolarPoint::asPoint(std::vector<PolarPoint> pts) {
	std::vector<Point> result;
	for (PolarPoint pt : pts) result.push_back(pt);
	return result;
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
    double x, y;

    x = (X + Y * l.A) / (1 + l.A * l.A);
    y = x * l.A;
	Point parallelPoint(x, y);
    double dist = abs(getDistance(parallelPoint) - abs(l.d));
    return dist;
}
