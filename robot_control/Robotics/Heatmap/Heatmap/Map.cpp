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

void Map::loadImage(cv::Mat img, int upscaling)
{

	//PRE-SCALING PREPROCESSING

	//UPSCALING (1ms)
	if (upscaling > 1) {
		resize(img, img, img.size() * upscaling, 0, 0, cv::INTER_NEAREST);
		scale = 1 / (double)upscaling;
	}

	//CREATION OF MAP OBJECT (14ms)
	int i = 0;
	for (cv::MatIterator_<cv::Vec3b> p = img.begin<cv::Vec3b>(); p < img.end<cv::Vec3b>(); p++, i++) {
		unsigned int x, y;
		x = i % img.cols;
		y = i / img.cols;
		MapNode* n = &(map.at(x, y));
		if (*p == vFree) n->type = eFree;
		else n->type = eObstacle;
		n->position = Point(x, y);
	}

	//WALL/OUTSIDE CATEGORISATION (17ms)
	std::vector<Point> edgePoints;
	for (int i = 0; i < img.cols; i++) {
		edgePoints.push_back(Point(i, 0));
		edgePoints.push_back(Point(i, img.rows - 1));
	}
	for (int i = 1; i < img.rows - 1; i++) {
		edgePoints.push_back(Point(0, i));
		edgePoints.push_back(Point(img.cols - 1, i));
	}
	for (Point p : edgePoints) if (map.at(p).type == eFree) recursivelyFill(p);
	std::vector<Point> surroundedPoints;
	for (int i = 0; i < map.cols() * map.rows(); i++) {
		Point p = getPointFromIndex(i);
		if (map.at(p, false).type == eObstacle) {
			bool surroundedByWalls = true;
			for (int i = 0; i < 8 && surroundedByWalls; i++) {
				Point _p = p + dirs[i];
				if (map.inBounds(_p)) surroundedByWalls &= map.at(_p, false).type == eObstacle || map.at(_p, false).type == eOutside;
			}
			if (surroundedByWalls) map.at(p).type = eOutside;
		}
	}

	//POST-SCALING PREPROCESSING 
	vertices = getVertices(); //(28ms)
	edges = getEdges(); //(223ms)
	calculateBrushfire(); //(437ms)
}

cv::Mat Map::drawMap(drawType type, bool draw, drawArguments args)
{
	std::string windowName;
	unsigned int w = map.cols(), h = map.rows();
	cv::Mat img(h, w, CV_8UC3);

	placePoint(Point(100, 40));

	std::vector<Point> path;

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
		for (int i = 0; i < w * h; i++) {
			unsigned int x = i % w, y = i / w;
			cv::Vec3b* v = &img.at<cv::Vec3b>(cv::Point(x, y));
			int d = map.at(x, y).wallDistance;
			if (d <= 0) *v = vObstacle;
			else {
				double a = (double)d / (double)maxDist;
				*v = vUndiscovered * (1 - a) + cv::Vec3b(0, 255, 255) * a;
			}
		}
		break;

	case eGeometry:
  		windowName = "Geometry";
		img = drawMap(eBasic, false);
		for (Edge e : edges) {
			std::vector<Point> line = e.getPoints();
			for (Point p : line) img.at<cv::Vec3b>(p.getCVPoint()) = vUndiscovered;
		}
		for (Point p : vertices) img.at<cv::Vec3b>(p.getCVPoint()) = vPoint;
		break;

	case ePath:
		path = getPath(args.A / scale, args.B / scale);
		windowName = "Path";
		img = drawMap(eBasic, false);
		for (Point p : path) img.at<cv::Vec3b>(p.getCVPoint()) = vUndiscovered;
		path = simplifyPath(path);
		for (Point p : path) img.at<cv::Vec3b>(p.getCVPoint()) = vPoint;
		break;
	}

	if (draw) {
		int scale = MIN((980 / img.rows), (1720 / img.cols));
		cv::resize(img, img, cv::Size(), scale, scale, cv::INTER_NEAREST);
		cv::namedWindow(windowName, CV_WINDOW_AUTOSIZE);
		cv::imshow(windowName, img);
		cv::waitKey();
	}
	return img;
}

std::vector<Point> Map::getPoints()
{
	return points;
}

void Map::placePoint(Point p)
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
			bool LOS = hasLineOfSight(p, Point(x, y));
			if (LOS) map.at(x, y, false).hmDistance = dist;
		}
	}
}

bool Map::hasLineOfSight(Point a, Point b, double wallDistanceThreshold)
{
	std::vector<Point> line = Edge(a,b).getPoints();
	for (Point p : line) if (map.at(p.x(), p.y(), false).wallDistance <= wallDistanceThreshold) return false;
	return true;
}

void Map::recursivelyFill(Point p)
{
	map.at(p).type = eOutside;
	Point dirs[] = { Point(-1,0), Point(1,0), Point(0,-1), Point(0,1) };
	for (int i = 0; i < 4; i++) {
		Point nextPoint = p + dirs[i];
		if (map.inBounds(nextPoint)) if (map.at(nextPoint).type == eFree) recursivelyFill(nextPoint);
	}
}

bool Map::isDiscoverable(Point p)
{
	nodeType t = map.at(p, false).type;
	return (t != eObstacle && t != eOutside);
}

void Map::calculateBrushfire()
{
	std::unordered_set<MapNode*> nodesToCheck;
	for (int i = 0; i < map.cols() * map.rows(); i++) {
		Point p = getPointFromIndex(i);
		MapNode* n = &map.at(p, false);
		if (n->type == eObstacle) {
			n->wallDistance = 0;
			for (int i = 0; i < 8; i++) {
				Point _p = p + dirs[i];
				if (map.inBounds(_p)) {
					MapNode* _n = &map.at(_p, false);
					if (_n->type == eFree && (nodesToCheck.find(_n) == nodesToCheck.end())) nodesToCheck.insert(_n);
				}
			}
		}
	}
	while (!nodesToCheck.empty()) {
		std::unordered_set<MapNode*> tempSet;
		for (auto i = nodesToCheck.begin(); i != nodesToCheck.end();) {
			(*i)->wallDistance = getMinNeighbor((*i)->position) + scale;
			if (maxDist < (*i)->wallDistance) maxDist = (*i)->wallDistance;
			for (int j = 0; j < 8; j++) {
				Point p = (*i)->position + dirs[j];
				MapNode* n = &map.at(p, false);
				if (n->type == eFree && 
					(n->wallDistance > (*i)->wallDistance + scale || n->wallDistance == -1) &&
					(nodesToCheck.find(n) == nodesToCheck.end())) 
					tempSet.insert(n);
			}
			auto oldIterator = i++;
			nodesToCheck.erase(oldIterator);
		}
		nodesToCheck = tempSet;
	}
}

double Map::getMinNeighbor(Point p)
{
	double minVal = INT_MAX;
	for (int i = 0; i < 8; i++) {
		if (!map.inBounds(p + dirs[i])) return scale / 2.0;
		if (!isDiscoverable(p + dirs[i])) return scale / 2.0;
		if (map.at(p + dirs[i], false).wallDistance > 0) minVal = MIN(map.at(p + dirs[i]).wallDistance, minVal);
	}
	return minVal;
}

Point Map::getPointFromIndex(unsigned int i, unsigned int w)
{
	if (w == 0) w = map.cols();
	return Point(i % w, i / w);
}

std::vector<Point> Map::getVertices()
{
	if (vertices.size() != 0) return vertices;
	std::vector<Point> _vertices;
	nodeType vertexTest[2][3] = {
		{ eObstacle, eFree, eObstacle },
		{ eFree, eFree, eFree } };
	for (int i = 0; i < map.cols() * map.rows(); i++) {
		Point p = getPointFromIndex(i);
		if (map.at(p, false).type == eObstacle) {
			for (int j = 0; j < 2; j++) {
				bool vertex = false;
				for (int i = 0; i < 4 && !vertex; i++) {
					bool _vertex = true;
					for (int k = 0; k < 3 && _vertex; k++) {
						Point _p = p + dirs[(i * 2 + k) % 8];
						if (map.inBounds(_p)) _vertex &= (map.at(_p).type == vertexTest[j][k]);
						else _vertex = false;
					}
					vertex |= _vertex;
				}
				if (vertex) {
					_vertices.push_back(p);
					break;
				}
			}
		}
	}
	vertices = _vertices;
	return _vertices;
}

std::vector<Edge> Map::getEdges()
{
	if (edges.size() != 0) return edges;
	std::vector<Edge> _edges;
	if (vertices.size() == 0) getVertices();
	nodeType edgeTest[] = { eObstacle, eFree, eFree, eFree, eObstacle };
	for (int i = 0; i < vertices.size(); i++) {
		for (int j = i + 1; j < vertices.size(); j++) {
			Edge edge(vertices[i], vertices[j]);
			std::vector<Point> line = edge.getPoints();
			line.erase(line.begin());
			line.erase(line.end() - 1);
			bool isEdge = true;
			for (Point p : line) {
				isEdge &= map.at(p, false).type == eObstacle;
			}
			if (isEdge) _edges.push_back(edge);
		}
	}
	edges = _edges;
	return _edges;
}

std::vector<Point> Map::getPath(Point A, Point B, double padding)
{
	if (!isDiscoverable(A) || !isDiscoverable(B)) throw "ASTAR - Points inside wall.";
	if (!map.inBounds(A) || !map.inBounds(B)) throw "ASTAR - Points out of bounds.";

	//A and B are swapped to create the path vector easily.
	std::vector<Point> path;

	// Preprocessing
	for (int i = 0; i < map.cols() * map.rows(); i++) {
		Point p = getPointFromIndex(i);
		MapNode* n = &map.at(p);
		n->asH = GET_DISTANCE(p, A);
		n->asF = n->asG = INT_MAX;
		n->asParent = NULL;
		n->asSeen = n->asVisited = false;
	}

	// Setup for algorithm
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
			Point p = dirs[i] + n->position;
			if (map.inBounds(p)) {
				MapNode* pN = &map.at(p);

				if (pN->wallDistance > padding) {
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

	// Extracting path
	MapNode* nextNode = &map.at(A);
	while (nextNode != NULL) {
		path.push_back(nextNode->position);
		nextNode = nextNode->asParent;
	}

	return path;
}

std::vector<Point> Map::simplifyPath(std::vector<Point> path)
{
	bool changedPath;
	do {
		changedPath = false;
		int i = 0;
		while (i < path.size() - 2 && !changedPath) {
			if (path[i + 1] + (path[i + 1] - path[i]).normalized() == path[i + 2]) {
				changedPath = true;
				path.erase(path.begin() + i + 1);
			}
			i++;
		}
	} while (changedPath);
	return path;
}
