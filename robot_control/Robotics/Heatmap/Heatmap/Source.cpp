
#include "Map.h"
#include "Lidar.h"
#include "RandFloat.h"
#include "GraphDrawer.h"
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
	dA.A = Point(9, 7);
	dA.B = Point(54,71);
	m.drawMap(ePath, true, dA);
	*/

	srand(time(NULL));
	Lidar lidar;
	int N = 100;
	double a = 0.3, b = 1.5, randomness = 0.3;
	std::vector<PolarPoint> measurements(N);

	for (int i = 0; i < N; i++) {
		double x = RandFloat() * 2 - 1;
		double y = a * x + b + (RandFloat() - 0.5) * randomness;
		measurements[i] = Point(x, y).asPolar();
	}

	Line line = lidar.leastSquareFit(measurements);
	PolarPoint linePtP(line.d, line.a);
	Point linePtC = linePtP.asPoint();

	double dist = Point(-1, 2).getDistance(line);

	std::vector<Point> pMeasurements;
	for (PolarPoint p : measurements) pMeasurements.push_back(p.asPoint());

	GraphDrawer gD(cv::Size(800, 600));
	gD.addPoints(pMeasurements);
	gD.addLine(line);

	cv::imshow("Graph", gD.getImage());
	cv::waitKey();
	int c;
	std::cin >> c;
	return 0;
}