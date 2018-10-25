
#include "Map.h"
#include <iostream>

int main() {

	Map m(90,150);
	cv::Mat img = cv::imread("Map.png");
	m.loadImage(img);
	m.drawMap(eHeatmap);

	int c;
	std::cin >> c;
	return 0;
}