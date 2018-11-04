
#include "Map.h"
#include <iostream>

int main() {

	Map m(90,45);
	cv::Mat img = cv::imread("Map.png");
	resize(img, img, cv::Size(), 4, 4, cv::INTER_NEAREST);
	m.loadImage(img);
	m.drawMap(eBasic);
	m.drawMap(eGeometry);
	m.drawMap(eBrushfire);

	int c;
	std::cin >> c;
	return 0;
}