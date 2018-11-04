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
		n->position = MapPoint(x, y);
	}
	std::vector<MapPoint> edgePoints;
	for (int i = 0; i < img.cols; i++) {
		edgePoints.push_back(MapPoint(i, 0));
		edgePoints.push_back(MapPoint(i, img.rows - 1));
	}
	for (int i = 1; i < img.rows - 1; i++) {
		edgePoints.push_back(MapPoint(0, i));
		edgePoints.push_back(MapPoint(img.cols - 1, i));
	}
	for (MapPoint p : edgePoints) if (map.at(p).type == eFree) recursivelyFill(p);
	std::vector<MapPoint> surroundedPoints;
	for (int i = 0; i < map.cols() * map.rows(); i++) {
		MapPoint p = getPointFromIndex(i);
		if (map.at(p, false).type == eObstacle) {
			bool surroundedByWalls = true;
			for (int i = 0; i < 8 && surroundedByWalls; i++) {
				MapPoint _p = p + dirs[i];
				if (map.inBounds(_p)) surroundedByWalls &= map.at(_p, false).type == eObstacle || map.at(_p, false).type == eOutside;
			}
			if (surroundedByWalls) map.at(p).type = eOutside;
		}
	}
}

cv::Mat Map::drawMap(drawType type, bool draw, drawArguments args)
{
	std::string windowName;
	unsigned int w = map.cols(), h = map.rows();
	cv::Mat img(h, w, CV_8UC3);

	placePoint(MapPoint(100, 40));

	std::vector<MapPoint> vertices;
	std::vector<MapPoint> path;

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
		calculateBrushfire();
		for (int i = 0; i < w * h; i++) {
			unsigned int x = i % w, y = i / w;
			cv::Vec3b* v = &img.at<cv::Vec3b>(cv::Point(x, y));

			int d = map.at(x, y).wallDistance;
			if (d < 1) *v = vObstacle;
			else {
				double a = (double)d / (double)maxDist;
				*v = vUndiscovered * (1 - a) + cv::Vec3b(0, 255, 255) * a;
			}
		}
		break;

	case eGeometry:
		windowName = "Geometry";
		img = drawMap(eBasic, false);
		vertices = getVertices();
		edges = getEdges();
		for (Edge e : edges) {
			std::vector<MapPoint> line = getLine(e.A, e.B);
			for (MapPoint p : line) img.at<cv::Vec3b>(p.getCVPoint()) = vUndiscovered;
		}
		for (MapPoint p : vertices) img.at<cv::Vec3b>(p.getCVPoint()) = vPoint;
		break;

	case ePath:
		path = getPath(args.A, args.B, args.padding);
		windowName = "Path";
		img = drawMap(eBasic, false);
		for (MapPoint p : path) img.at<cv::Vec3b>(p.getCVPoint()) = vUndiscovered;
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

std::vector<MapPoint> Map::getPoints()
{
	return points;
}

std::vector<MapPoint> Map::getLine(MapPoint a, MapPoint b)
{
	std::vector<MapPoint> line;
	if (a != b) {
		int dX = (int)b.x() - (int)a.x(), dY = (int)b.y() - (int)a.y(), l, g;
		bool findingX = false;
		if (abs(dX) > abs(dY)) l = dY, g = dX;
		else l = dX, g = dY, findingX = true;
		line = std::vector<MapPoint>(abs(g) + 1);

		double slope = (double)l / (double)g;
		int inc = g / abs(g);

		for (int i = 0; abs(i) < abs(g) + 1; i += inc) {
			MapPoint p;
			if (findingX) p = MapPoint(i * slope + (int)a.x(), i + (int)a.y());
			else p = MapPoint(i + (int)a.x(), i * slope + (int)a.y());

			line[abs(i)] = p;
		}
	}
	return line;
}

void Map::placePoint(MapPoint p)
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
			bool LOS = hasLineOfSight(p, MapPoint(x, y));
			if (LOS) map.at(x, y, false).hmDistance = dist;
		}
	}
}

bool Map::hasLineOfSight(MapPoint a, MapPoint b)
{
	std::vector<MapPoint> line = getLine(a, b);
	for (MapPoint p : line) {
		if (map.at(p.x(), p.y(), false).type == eObstacle) return false;
	}
	return true;
}

void Map::recursivelyFill(MapPoint p)
{
	map.at(p).type = eOutside;
	MapPoint dirs[] = { MapPoint(-1,0) ,MapPoint(1,0) ,MapPoint(0,-1) ,MapPoint(0,1) };
	for (int i = 0; i < 4; i++) {
		MapPoint nextPoint = p + dirs[i];
		if (map.inBounds(nextPoint)) if (map.at(nextPoint).type == eFree) recursivelyFill(nextPoint);
	}
}

bool Map::isDiscoverable(MapPoint p)
{
	nodeType t = map.at(p, false).type;
	return (t != eObstacle && t != eOutside);
}

void Map::calculateBrushfire(int layers)
{
	std::unordered_set<MapNode*> nodesToCheck;
	for (int i = 0; i < map.cols() * map.rows(); i++) {
		MapPoint p = getPointFromIndex(i);
		MapNode* n = &map.at(p, false);
		if (n->type == eObstacle) {
			n->wallDistance = 1;
			for (int i = 0; i < 8; i++) {
				MapPoint _p = p + dirs[i];
				if (map.inBounds(_p)) {
					MapNode* _n = &map.at(_p, false);
					if (_n->type == eFree && (nodesToCheck.find(_n) == nodesToCheck.end())) nodesToCheck.insert(_n);
				}
			}
		}
	}
	while (!nodesToCheck.empty() && layers-- != 0) {
		std::unordered_set<MapNode*> tempSet;
		for (auto i = nodesToCheck.begin(); i != nodesToCheck.end();) {
			(*i)->wallDistance = getMinNeighbor((*i)->position) + 1;
			if (maxDist < (*i)->wallDistance) maxDist = (*i)->wallDistance;
			for (int j = 0; j < 8; j++) {
				MapPoint p = (*i)->position + dirs[j];
				MapNode* n = &map.at(p, false);
				if (n->type == eFree && 
					(n->wallDistance > (*i)->wallDistance + 1 || n->wallDistance == 0) &&
					(nodesToCheck.find(n) == nodesToCheck.end())) 
					tempSet.insert(n);
			}
			auto oldIterator = i++;
			nodesToCheck.erase(oldIterator);
		}
		nodesToCheck = tempSet;
	}
}

int Map::getMinNeighbor(MapPoint p)
{
	int minVal = INT_MAX;
	for (int i = 0; i < 8; i++) {
		if (!map.inBounds(p + dirs[i])) return 1;
		if (!isDiscoverable(p + dirs[i])) return 1;
		if (map.at(p + dirs[i], false).wallDistance > 0) minVal = MIN(map.at(p + dirs[i]).wallDistance, minVal);
	}
	return minVal;
}

MapPoint Map::getPointFromIndex(unsigned int i, unsigned int w)
{
	if (w == 0) w = map.cols();
	return MapPoint(i % w, i / w);
}

std::vector<MapPoint> Map::getVertices()
{
	if (vertices.size() != 0) return vertices;
	std::vector<MapPoint> _vertices;
	nodeType vertexTest[2][3] = {
		{ eObstacle, eFree, eObstacle },
		{ eFree, eFree, eFree } };
	for (int i = 0; i < map.cols() * map.rows(); i++) {
		MapPoint p = getPointFromIndex(i);
		if (map.at(p, false).type == eObstacle) {
			for (int j = 0; j < 2; j++) {
				bool vertex = false;
				for (int i = 0; i < 4 && !vertex; i++) {
					bool _vertex = true;
					for (int k = 0; k < 3 && _vertex; k++) {
						MapPoint _p = p + dirs[(i * 2 + k) % 8];
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

std::vector<Map::Edge> Map::getEdges()
{
	if (edges.size() != 0) return edges;
	std::vector<Edge> _edges;
	if (vertices.size() == 0) getVertices();
	nodeType edgeTest[] = { eObstacle, eFree, eFree, eFree, eObstacle };
	for (int i = 0; i < vertices.size(); i++) {
		for (int j = i + 1; j < vertices.size(); j++) {
			std::vector<MapPoint> line = getLine(vertices[i], vertices[j]);
			line.erase(line.begin());
			line.erase(line.end() - 1);
			bool isEdge = true;
			for (MapPoint p : line) {
				isEdge &= map.at(p, false).type == eObstacle;
			}
			if (isEdge) _edges.push_back(Edge{ vertices[i], vertices[j] });
		}
	}
	edges = _edges;
	return _edges;
}

std::vector<MapPoint> Map::getPath(MapPoint A, MapPoint B, unsigned int padding)
{
	if (!isDiscoverable(A) || !isDiscoverable(B)) throw "ASTAR - Points inside wall.";
	if (!map.inBounds(A) || !map.inBounds(B)) throw "ASTAR - Points out of bounds.";

	//A and B are swapped to create the path vector easily.
	if (calculatedLayers < padding) calculateBrushfire(padding);
	std::vector<MapPoint> path;


	for (int i = 0; i < map.cols() * map.rows(); i++) {
		MapPoint p = getPointFromIndex(i);
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
			MapPoint p = dirs[i] + n->position;
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

	MapNode* nextNode = &map.at(A);
	while (nextNode != NULL) {
		path.push_back(nextNode->position);
		nextNode = nextNode->asParent;
	}

	return path;
}
