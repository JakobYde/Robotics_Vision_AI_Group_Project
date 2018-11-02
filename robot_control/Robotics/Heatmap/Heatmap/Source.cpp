
#include "Map.h"
#include <iostream>

int main() {

	Map m(90,45);
	cv::Mat img = cv::imread("Map.png");
	resize(img, img, cv::Size(), 1, 1, cv::INTER_NEAREST);
	m.loadImage(img);


	for (int i = 0; i < 10; i++) {
		m.drawMap(eLargestBox);
		cv::waitKey();
	}

	int c;
	std::cin >> c;
	return 0;
}