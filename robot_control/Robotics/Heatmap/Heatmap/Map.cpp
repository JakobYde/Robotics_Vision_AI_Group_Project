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
		n->position = Point<unsigned int>(x, y);
	}
	std::vector<Point<unsigned int>> edgePoints;
	for (int i = 0; i < img.cols; i++) {
		edgePoints.push_back(Point<unsigned int>(i, 0));
		edgePoints.push_back(Point<unsigned int>(i, img.rows - 1));
	}
	for (int i = 1; i < img.rows - 1; i++) {
		edgePoints.push_back(Point<unsigned int>(0, i));
		edgePoints.push_back(Point<unsigned int>(img.cols - 1, i));
	}
	for (Point<unsigned int> p : edgePoints) if (map.at(p).type == eFree) recursivelyFill(p);
}

void Map::drawMap(drawType type, drawArguments args)
{
	std::string windowName;
	unsigned int w = map.cols(), h = map.rows();
	cv::Mat img(h, w, CV_8UC3);

	placePoint(Point<unsigned int>(100, 40));

	switch (type) {
	case eBasic:
		windowName = "Basic Map";
		for (int i = 0; i < w * h; i++) {
			unsigned int x = i % w, y = i / w;
			cv::Vec3b* v = &img.at<cv::Vec3b>(cv::Point(x, y));
			nodeType type = map.at(x, y).type;
			if (type == eFree) *v = vFree;
			else if (type == eOutside) *v = vOutside;
			else if (type == eObstacle) *v = vObstacle;
		}
		break;

	case eHeatmap:
		windowName = "Heatmap";
		for (int i = 0; i < w * h; i++) {
			unsigned int x = i % w, y = i / w;
			cv::Vec3b* v = &img.at<cv::Vec3b>(cv::Point(x, y));

			MapNode n = map.at(x, y);
				 if (n.type == eObstacle) *v = vObstacle;
			else if (n.type == eOutside) *v = vOutside;
			else if (n.type == eFree) {
				if (n.hmDistance == -1 || n.hmDistance > viewDistance) *v = vUndiscovered;
				else if (n.hmDistance == 0) *v = vPoint;
				else *v = (1 - n.hmDistance / viewDistance) * vDiscovered + (n.hmDistance / viewDistance) * vUndiscovered;
			}

		}
		break;

	case eBrushfire:
		windowName = "Brushfire Map";
		seperateIntoRooms();
		for (int i = 0; i < w * h; i++) {
			unsigned int x = i % w, y = i / w;
			cv::Vec3b* v = &img.at<cv::Vec3b>(cv::Point(x, y));

			int d = map.at(x, y).distanceFromDiscovered;
			if (d < 1) *v = vObstacle;
			else {
				double a = (double)d / (double)maxDist;
				*v = vUndiscovered * (1 - a) + cv::Vec3b(0, 255, 255) * a;
			}
		}
		break;

	case ePath:
		std::vector<Point<unsigned int>> path = getPath(args.A, args.B, args.padding);
		windowName = "Path";
		for (int i = 0; i < w * h; i++) {
			unsigned int x = i % w, y = i / w;
			cv::Vec3b* v = &img.at<cv::Vec3b>(cv::Point(x, y));
			nodeType type = map.at(x, y).type;
			if (type == eFree) {
				if (map.at(x, y).asVisited) *v = cv::Vec3b(240,60,60);
				else if (map.at(x, y).asSeen) *v = cv::Vec3b(240,160,160);
				else *v = vFree;
			}
			else if (type == eOutside) *v = vOutside;
			else if (type == eObstacle) *v = vObstacle;
		}
		for (Point<unsigned int> p : path) {
			img.at<cv::Vec3b>(cv::Point(p.x(), p.y())) = vUndiscovered;
		}
		break;
	}

	int scale = MIN((1080 / img.rows), (1920 / img.cols));
	cv::resize(img, img, cv::Size(), scale, scale, cv::INTER_NEAREST);
	cv::imshow(windowName, img);
}

std::vector<Point<unsigned int>> Map::getPoints()
{
	return points;
}

std::vector<Point<unsigned int>> Map::getLine(Point<unsigned int> a, Point<unsigned int> b)
{
	std::vector<Point<unsigned int>> line;
	if (a != b) {
		int dX = (int)b.x() - (int)a.x(), dY = (int)b.y() - (int)a.y(), l, g;
		bool findingX = false;
		if (abs(dX) > abs(dY)) l = dY, g = dX;
		else l = dX, g = dY, findingX = true;
		line = std::vector<Point<unsigned int>>(abs(g) + 1);

		double slope = (double)l / (double)g;
		int inc = g / abs(g);

		for (int i = 0; abs(i) < abs(g) + 1; i += inc) {
			Point<unsigned int> p;
			if (findingX) p = Point<unsigned int>(i * slope + (int)a.x(), i + (int)a.y());
			else p = Point<unsigned int>(i + (int)a.x(), i * slope + (int)a.y());

			line[abs(i)] = p;
		}
	}
	return line;
}

void Map::placePoint(Point<unsigned int> p)
{
	unsigned int w = map.cols(), h = map.rows();
	unsigned int xStart = MAX(0, p.x() - viewDistance), xEnd = MIN(w - 1, p.x() + viewDistance);
	unsigned int yStart = MAX(0, p.y() - viewDistance), yEnd = MIN(h - 1, p.y() + viewDistance);
	int dX = xEnd - xStart, dY = yEnd - yStart;
	for (int i = 0; i < dX * dY; i++) {
		int x = i % dX + xStart, y = i / dX + yStart;
		if (map.at(x, y, false).type == eFree) {
			double dx = x - (int)p.x(), dy = y - (int)p.y();
			double dist = sqrt(dx * dx + dy * dy);
			bool LOS = hasLineOfSight(p, Point<unsigned int>(x, y));
			if (LOS) map.at(x, y, false).hmDistance = dist;
		}
	}
}

bool Map::hasLineOfSight(Point<unsigned int> a, Point<unsigned int> b)
{
	std::vector<Point<unsigned int>> line = getLine(a, b);
	for (Point<unsigned int> p : line) {
		if (map.at(p.x(), p.y(), false).type == eObstacle) return false;
	}
	return true;
}

void Map::recursivelyFill(Point<unsigned int> p)
{
	map.at(p).type = eOutside;
	Point<unsigned int> dirs[] = { Point<unsigned int>(-1,0) ,Point<unsigned int>(1,0) ,Point<unsigned int>(0,-1) ,Point<unsigned int>(0,1) };
	for (int i = 0; i < 4; i++) {
		Point<unsigned int> nextPoint = p + dirs[i];
		if (map.inBounds(nextPoint)) if (map.at(nextPoint).type == eFree) recursivelyFill(nextPoint);
	}
}

bool Map::isDiscoverable(Point<unsigned int> p)
{
	nodeType t = map.at(p).type;
	//return (t != eObstacle && t != eOutside && map.at(p).hmDistance == -1);
	return (t != eObstacle && t != eOutside);
}

void Map::seperateIntoRooms(int layers)
{
	bool tilesChanged;
	do {
		maxDist = 0;
		layers--;
		tilesChanged = false;
		for (int i = 0; i < map.cols() * map.rows(); i++) {
			Point<unsigned int> p(i % map.cols(),i / map.cols());
			if (isDiscoverable(p)) {
				int* d = &map.at(p, false).distanceFromDiscovered;
				int minNeighbor = getMinNeighbor(p) + 1;
				if (*d > minNeighbor) {
					*d = minNeighbor;
					tilesChanged = true;
				}
			}
			else (map.at(p).distanceFromDiscovered = 0);
			maxDist = MAX(map.at(p, false).distanceFromDiscovered, maxDist);
		}
	} while (tilesChanged && layers != 0);
	calculatedLayers = layers;
}

int Map::getMinNeighbor(Point<unsigned int> p)
{
	int minVal = INT_MAX;
	for (int i = 0; i < 8; i++) {
		if (!map.inBounds(p + dirs[i])) return 0;
		if (!isDiscoverable(p + dirs[i])) return 0;
		minVal = MIN(map.at(p + dirs[i]).distanceFromDiscovered, minVal);
	}
	return minVal;
}

std::vector<Point<unsigned int>> Map::getPath(Point<unsigned int> A, Point<unsigned int> B, unsigned int padding)
{
	if (!isDiscoverable(A) || !isDiscoverable(B)) throw "ASTAR - Points inside wall.";
	if (!map.inBounds(A) || !map.inBounds(B)) throw "ASTAR - Points out of bounds.";

	//A and B are swapped to create the path vector easily.
	if (calculatedLayers < padding) seperateIntoRooms(padding);
	std::vector<Point<unsigned int>> path;


	for (int i = 0; i < map.cols() * map.rows(); i++) {
		Point<unsigned int> p(i % map.cols(), i / map.cols());
		MapNode* n = &map.at(p);
		n->asH = GET_DISTANCE(p, A);
		n->asF = n->asG = INT_MAX;
		n->asParent = NULL;
		n->asSeen = n->asVisited = false;
		
	}
	std::priority_queue<MapNode*, std::vector<MapNode*>, GreaterH> queue;
	bool pathFound = false;

	map.at(B).asSeen = map.at(B).asVisited = true;
	map.at(B).asG = 0;
	map.at(B).asF = map.at(B).asH;
	queue.push(&map.at(B));

	while (!pathFound && queue.size() > 0) {
		MapNode* n = queue.top();
		n->asVisited = true;
		queue.pop();

		for (int i = 0; i < 8; i++) {
			Point<unsigned int> p = dirs[i] + n->position;
			if (map.inBounds(p)) {
				MapNode* pN = &map.at(p);

				if (pN->distanceFromDiscovered > padding) {
					double g = GET_DISTANCE(p, n->position) + n->asG, f = pN->asH + g;

					if (p == A) {
						pathFound = true;
						pN->asParent = n;
						break;
					}

					if (f < pN->asF) {
						pN->asF = f;
						pN->asG = g;
						pN->asParent = n;
					}

					if (!pN->asSeen) {
						pN->asSeen = true;
						queue.push(pN);
					}
				}
			}
		}
	}

	MapNode* nextNode = &map.at(A);
	while (nextNode != NULL) {
		path.push_back(nextNode->position);
		nextNode = nextNode->asParent;
	}

	return path;
}
