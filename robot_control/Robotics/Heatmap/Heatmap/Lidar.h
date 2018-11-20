#pragma once

#include "Point.h"
#include "Edge.h"

class Lidar
{
public:
	Lidar();
	~Lidar();

	void newMeasurement();
	void addMeasurement(PolarPoint p);
	void addMeasurements(std::vector<PolarPoint> points);

	Line leastSquareFit(std::vector<PolarPoint> points);
	std::vector<Line> getLines();
	std::vector<Point<double>> getCorners();

private:
	std::vector<PolarPoint> points;
};

