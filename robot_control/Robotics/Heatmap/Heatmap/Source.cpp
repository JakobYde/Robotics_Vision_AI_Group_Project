
#include "Map.h"
#include "Lidar.h"
#include <iostream>

int main() {
	/*
	Map m(90,45);
	cv::Mat img = cv::imread("Map.png");
	m.loadImage(img, 1);
	//m.drawMap(eBasic);
	m.drawMap(eGeometry);
	m.drawMap(eBrushfire);

	drawArguments dA;
	dA.A = Point<double>(9, 7);
	dA.B = Point<double>(54,71);
	m.drawMap(ePath, true, dA);
	*/

	Lidar lidar;
	std::vector<PolarPoint> measurements = { Point<double>(-1,2).asPolar(), Point<double>(-1,3).asPolar(),Point<double>(-4,3).asPolar() };
	Line line = lidar.leastSquareFit(measurements);
	PolarPoint linePtP(line.d, line.a);
	Point<double> linePtC = linePtP.asPoint();

	double dist = Point<double>(-1, 2).getDistance(line);

	int c;
	std::cin >> c;
	return 0;
}