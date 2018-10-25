#include "Map.h"

Map::Map()
{
	*this = Map(90, 100);
}

Map::Map(double fov, double viewDistance) : fov(fov), viewDistance(viewDistance)
{
}


Map::~Map()
{
}

void Map::loadImage(cv::Mat img)
{
	int i = 0;
	for (cv::MatIterator_<cv::Vec3b> p = img.begin<cv::Vec3b>(); p < img.end<cv::Vec3b>(); p++, i++) {
		unsigned int x, y;
		x = i % img.cols;
		y = i / img.cols;
		MapNode* n = &(map.at(x, y));
		if (*p == vFree) n->type = eFree;
		else n->type = eObstacle;
	}
}

void Map::drawMap(drawType type)
{
	std::string windowName;
	unsigned int w = map.cols(), h = map.rows();
	cv::Mat img(h, w, CV_8UC3);

	placePoint(Point<unsigned int>(300, 220));

	switch (type) {
	case eBasic:
		windowName = "Basic Map";
		for (int i = 0; i < w * h; i++) {
			unsigned int x = i % w, y = i / w;
			cv::Vec3b* v = &img.at<cv::Vec3b>(cv::Point(x, y));
			if (map.at(x, y).type == eFree) *v = vFree;
			else *v = vObstacle;
		}
		break;

	case eHeatmap:
		windowName = "Heatmap";
		for (int i = 0; i < w * h; i++) {
			unsigned int x = i % w, y = i / w;
			cv::Vec3b* v = &img.at<cv::Vec3b>(cv::Point(x, y));

			MapNode n = map.at(x, y);
			if (n.type == eObstacle) *v = vObstacle;
			else {
				if (n.hmDistance == -1 || n.hmDistance > viewDistance) *v = vUndiscovered;
				else *v = (1 - n.hmDistance / viewDistance) * vDiscovered + (n.hmDistance / viewDistance) * vUndiscovered;
			}

		}
		break;

	}

	cv::imshow(windowName, img);
	cv::waitKey();
}

std::vector<Point<unsigned int>> Map::getPoints()
{
	return std::vector<Point<unsigned int>>();
}

std::vector<Point<unsigned int>> Map::getLine(Point<unsigned int> a, Point<unsigned int> b)
{
	std::vector<Point<unsigned int>> line;
	if (a != b) {
		int dX = (int)b.x() - (int)a.x(), dY = (int)b.y() - (int)a.y(), l, g;
		bool findingX = false;
		if (abs(dX) > abs(dY)) l = dY, g = dX;
		else l = dX, g = dY, findingX = true;

		double slope = (double)l / (double)g;
		int inc = g / abs(g);

		for (int i = 0; abs(i) < abs(g) + 1; i += inc) {
			Point<unsigned int> p;
			if (findingX) p = Point<unsigned int>(i * slope + (int)a.x(), i + (int)a.y());
			else p = Point<unsigned int>(i + (int)a.x(), i * slope + (int)a.y());

			line.push_back(p);
		}
	}
	return line;
}

void Map::placePoint(Point<unsigned int> p)
{
	unsigned int w = map.cols(), h = map.rows();
	unsigned int xStart = MAX(0, p.x() - viewDistance), xEnd = MIN(w, p.x() + viewDistance);
	unsigned int yStart = MAX(0, p.y() - viewDistance), yEnd = MIN(w, p.y() + viewDistance);
	unsigned int dX = xEnd - xStart, dY = yEnd - yStart;
	for (int i = 0; i < dX * dY; i++) {
		int x = i % dX + xStart, y = i / dX + yStart;
		double dx = x - (int)p.x(), dy = y - (int)p.y();
		double dist = sqrt(pow(dx, 2) + pow(dy, 2));
		bool LOS = hasLineOfSight(p, Point<unsigned int>(x, y));
		if (LOS) 
			map.at(x, y).hmDistance = dist;
	}
}

bool Map::hasLineOfSight(Point<unsigned int> a, Point<unsigned int> b)
{
	std::vector<Point<unsigned int>> line = getLine(a, b);
	for (Point<unsigned int> p : line) {
		if (map.at(p.x(), p.y()).type == eObstacle) return false;
	}
	return true;
}
