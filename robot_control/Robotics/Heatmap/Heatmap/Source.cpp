
#include "Map.h"
#include <iostream>

int main() {
	Point<double> pt(2, 1);
	PolarPoint ppt = pt.asPolar();
	Point<double> pt2 = ppt.asPoint();
	Point<unsigned int> pt3 = ppt.asUIntPoint();

	Map m(90,45);
	cv::Mat img = cv::imread("Map.png");
	resize(img, img, cv::Size(), 2, 2, cv::INTER_NEAREST);
	m.loadImage(img);
	drawArguments dA{ Point<unsigned int>(6,6), Point<unsigned int>(120,140), 1 };
	m.drawMap(eBrushfire, dA);
	m.drawMap(ePath, dA);

	cv::waitKey(0);
	int c;
	std::cin >> c;
	return 0;
}