
#include "Map.h"
#include "Lidar.h"
#include "RandFloat.h"
#include "GraphDrawer.h"
#include <iostream>

int main() {
	
	Map m(60,60);
	cv::Mat img = cv::imread("Maps/Map.png");
	m.loadImage(img, 1);

	std::vector<PolarPoint> points;

	srand(time(NULL));

	double maxRange = 10;
	int N = 100;
	for (int i = 0; i < N; i++) {
		points.push_back(Point(-(double)i / N * maxRange, 2 + 0.01 * i).asPolar());
	}
	for (int i = 0; i < N; i++) {
		points.push_back(Point(0, (double)i / N * maxRange).asPolar());
	}

	Lidar l;
	l.addMeasurements(points);
	l.drawProcess();

	int key = 0;
	while (key != 'q') key = cv::waitKey();
	return 0;
}