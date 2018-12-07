#include "Map.h"

Map::Map()
{
	*this = Map(90, 40);
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
		viewDistance /= scale;
	}

	Room r(Point(10, 5), 5, 3);
	r.getPoints(M_SQRT2);

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
	wallVertices = getVertices(eObstacle); //(28ms)
	floorVertices = getVertices(eFree); //(28ms)
	//edges = getEdges(); //(223ms)
	calculateBrushfire(); //(437ms)
}

cv::Mat Map::drawMap(drawType type, bool draw, drawArguments args)
{
	std::string windowName;
	unsigned int w = map.cols(), h = map.rows();
	cv::Mat img(h, w, CV_8UC3);

	int postScale = MIN((980 / img.rows), (1720 / img.cols));
	bool dontResize = false;

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

	case eHeatmap: {
		windowName = "Heatmap";
		std::priority_queue<Room> _rooms = rooms;
		while (!_rooms.empty()) {
			Room r = _rooms.top();
			if (!isScouted(r)) {
				std::vector<Point> points = r.getPoints(viewDistance);
				for (Point p : points) {
					placePoint(p);
				}
			}
			_rooms.pop();
		}
		for (int i = 0; i < w * h; i++) {
			unsigned int x = i % w, y = i / w;
			cv::Vec3b* v = &img.at<cv::Vec3b>(cv::Point(x, y));

			MapNode n = map.at(x, y);
			if (n.type == eObstacle) *v = vObstacle;
			else if (n.type == eOutside) *v = vOutside;
			else if (n.type == eFree) {
				if (n.hmDistance == -1 || n.hmDistance > viewDistance) *v = vUndiscovered;
				//else if (n.hmDistance == 0) *v = vPoint;
				else *v = (1 - n.hmDistance / viewDistance) * vDiscovered + (n.hmDistance / viewDistance) * vUndiscovered;
			}
		}
		cv::resize(img, img, cv::Size(), postScale, postScale, cv::INTER_NEAREST);
		for (int i = 0; i < map.cols() * map.rows(); i++) {
			Point p = getPointFromIndex(i);
			MapNode n = map.at(p);
			if (n.roomNumber == ROOM_PARTITION) {
				for (int i = 0; i < postScale; i++) img.at<cv::Vec3b>((p * postScale + Point(i, 0.5 * postScale)).getCVPoint()) = vPoint;
			}
		}
		for (Point p : points) cv::circle(img, (p * postScale).getCVPoint(), 4, vPoint, -1);
		for (int i = 0; i < points.size(); i++) cv::putText(img, std::to_string(i), (points[i] * postScale).getCVPoint(), cv::FONT_HERSHEY_PLAIN, 4, cv::Vec3b(0,0,0));

		dontResize = true;
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
		for (Point p : wallVertices) img.at<cv::Vec3b>(p.getCVPoint()) = vPoint;
		for (Point p : floorVertices) img.at<cv::Vec3b>(p.getCVPoint()) = vDiscovered;
		break;

	case ePath: {
		Path path = getPath(args.A / scale, args.B / scale, args.padding / scale);
		windowName = "Path";
		img = drawMap(eBasic, false);
		for (Point p : path) img.at<cv::Vec3b>(p.getCVPoint()) = vUndiscovered;
		path = simplifyPath(path);
		for (Point p : path) img.at<cv::Vec3b>(p.getCVPoint()) = vPoint;
	}
		break;

	case eCells:
		windowName = "Cells";
		img = drawMap(eBasic, false);
		seperateIntoRooms();

		for (int i = 0; i < w * h; i++) {
			unsigned int x = i % w, y = i / w;
			if (map.at(x, y).type == eFree) {
				switch (map.at(x, y).roomNumber) {
				case ROOM_DEFAULT:
					img.at<cv::Vec3b>(cv::Point(x, y)) = vFree;
					break;

				case ROOM_PARTITION:
					img.at<cv::Vec3b>(cv::Point(x, y)) = map.at(x, y).roomColor;
					break;

				default:
					img.at<cv::Vec3b>(cv::Point(x, y)) = map.at(x, y).roomColor;
					break;
				}
			}
		}
		for (Point p : floorVertices) img.at<cv::Vec3b>(p.getCVPoint()) = cv::Vec3b(0, 255, 0);
		break;
	}

	if (draw) {
		if (!dontResize) cv::resize(img, img, cv::Size(), postScale, postScale, cv::INTER_NEAREST);
		cv::namedWindow(windowName, CV_WINDOW_AUTOSIZE);
		cv::imshow(windowName, img);
		cv::waitKey();
	}
	return img;
}

Map::Plan Map::getPlan()
{
	Plan plan;
	std::priority_queue<Plan, std::vector<Plan>, std::greater<Plan>> openSet;

	size_t N = points.size();
	//Path* paths = new Path[N * N];
	Path paths[21 * 21];

	int startIndex = 2;

	for (int i = 0; i < N; i++) {
		if (i != startIndex) {
			paths[i + N * startIndex] = getPath(points[startIndex], points[i]);
			paths[N * i + startIndex] = paths[i];
			Plan plan({ paths[i] });
			plan.pointsVisited = { startIndex, i };
			openSet.push(plan);
		}
	}

	bool stop = false;
	while (!stop) {
		Plan plan = openSet.top();
		openSet.pop();
		if (plan.pointsVisited.size() == N) break;
		int lastPointIndex = plan.pointsVisited[plan.pointsVisited.size() - 1];
		std::vector<int> pointsLeft;
		for (int i = 0; i < N; i++) if (std::find(plan.pointsVisited.begin(), plan.pointsVisited.end(), i) == plan.pointsVisited.end()) pointsLeft.push_back(i);
		for (int i : pointsLeft) {
			Path path = paths[lastPointIndex + i * N];
			if (path == Path()) {
				path = getPath(points[i], points[lastPointIndex]);
				paths[lastPointIndex + i * N] = path;
			}
			Plan p = plan + path;
			p.pointsVisited.push_back(i);
			openSet.push(p);
		}
	}
	return plan;
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
	points.push_back(p);
	for (int i = 0; i < dX * dY; i++) {
		int x = i % dX + xStart, y = i / dX + yStart;
		if (map.at(x, y, false).type == eFree) {
			double dx = x - p.x(), dy = y - p.y();
			double dist = sqrt(dx * dx + dy * dy);
			bool LOS = hasLineOfSight(p, Point(x, y));
			if (LOS) {
				double* phmDist = &map.at(x, y, false).hmDistance;
				if (*phmDist == HMDISTANCE_DEFAULT || *phmDist > dist) *phmDist = dist;
			}
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
	std::queue<Point> points;
	points.push(p);
	while (!points.empty()) {
		Point p = points.front();
		points.pop();
		if (map.at(p).type == eFree) {
			map.at(p).type = eOutside;
			Point dirs[] = { Point(-1,0), Point(1,0), Point(0,-1), Point(0,1) };
			for (int i = 0; i < 4; i++) {
				Point nextPoint = p + dirs[i];
				if (map.inBounds(nextPoint)) if (map.at(nextPoint).type == eFree) points.push(nextPoint);
			}
		}
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

std::vector<Point> Map::getVertices(nodeType type)
{
	if (type == eObstacle && wallVertices.size() != 0) return wallVertices;
	if (type == eFree && floorVertices.size() != 0) return floorVertices;
	nodeType invType;
	if (type == eObstacle) invType = eFree;
	else invType = eObstacle;
	std::vector<Point> _vertices;
	nodeType vertexTest[2][3] = {
		{ type, invType, type },
		{ invType, invType, invType } };
	for (int i = 0; i < map.cols() * map.rows(); i++) {
		Point p = getPointFromIndex(i);
		if (map.at(p, false).type == type) {
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
	if (type == eObstacle) wallVertices = _vertices;
	else floorVertices = _vertices;
	return _vertices;
}

std::vector<Edge> Map::getEdges()
{
	if (edges.size() != 0) return edges;
	std::vector<Edge> _edges;
	if (wallVertices.size() == 0) getVertices(eObstacle);
	if (floorVertices.size() == 0) getVertices(eFree);
	nodeType edgeTest[] = { eObstacle, eFree, eFree, eFree, eObstacle };
	for (int i = 0; i < wallVertices.size(); i++) {
		for (int j = i + 1; j < wallVertices.size(); j++) {
			Edge edge(wallVertices[i], wallVertices[j]);
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

void Map::seperateIntoRooms()
{
	getVertices(eFree);
	for (Point p1 : floorVertices) {
		for (int i = 0; i < 2; i++) {
			Point dir = dirs[(i * 4) % 8];
			Point p = p1 + dir;
			std::vector<Point> points;
			bool inLineWithOtherPoint = false;
			while (map.at(p).type != eObstacle) {
				points.push_back(p);
				inLineWithOtherPoint |= std::find(floorVertices.begin(), floorVertices.end(), p) != floorVertices.end();
				p += dir;
			}
			for (Point p : points)
			{
				map.at(p).roomNumber = ROOM_PARTITION;
				map.at(p).roomColor = vUndiscovered;
			}
		}
	}

	for (int i = 0; i < map.cols() * map.rows(); i++) {
		Point origin = getPointFromIndex(i);

		int nextRoomNum = rooms.size();
		if (map.at(origin).type == eFree && map.at(origin).roomNumber == ROOM_DEFAULT) {
			int width = 0, height = 0;
			cv::Vec3b roomColor(rand() % 256, rand() % 256, rand() % 256);
			std::queue<Point> points;
			points.push(origin);

			while (!points.empty()) {
				Point p = points.front();
				points.pop();
				if (map.at(p).roomNumber == ROOM_DEFAULT) {
					map.at(p).roomNumber = nextRoomNum;
					map.at(p).roomColor = roomColor;
					width = MAX(width, p.x() - origin.x());
					height = MAX(height, p.y() - origin.y());
					for (int d = 0; d < 4; d++) {
						Point _p = p + dirs[(d * 2) % 8];
						if (map.at(_p).type == eFree && map.at(_p).roomNumber == ROOM_DEFAULT) points.push(_p);
					}
				}
			}
			rooms.push(Room(origin, width, height));
			nextRoomNum++;
		}
	}

}

bool Map::isScouted(Room room)
{
	for (int i = 0; i < room.height * room.width; i++) {
		Point p(i % room.width, i / room.width);
		p += room.origin;
		if (map.at(p).hmDistance == -1 || map.at(p).hmDistance > viewDistance) return false;
	}
	return true;
}

Map::Path Map::getPath(Point A, Point B, double padding)
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

	return Path(path);
}

Map::Path Map::simplifyPath(Path path)
{
	bool changedPath;
	do {
		changedPath = false;
		int i = 0;
		while (i < path.points.size() - 2 && !changedPath) {
			if (path.points[i + 1] + (path.points[i + 1] - path.points[i]).normalized() == path.points[i + 2]) {
				changedPath = true;
				path.points.erase(path.points.begin() + i + 1);
			}
			i++;
		}
	} while (changedPath);
	return path;
}

Map::Room::Room(Point A, Point B)
{
	origin = Point(MIN(A.x(), B.x()), MIN(A.y(), B.y()));
	width = abs(A.x() - B.x());
	height = abs(A.y() - B.y());
}

std::vector<Point> Map::Room::getPoints(double viewDistance)
{
	std::vector<Point> result;
	double viewLength = 2 * viewDistance / M_SQRT2;
	int nX = ceil(width / viewLength), nY = ceil(height / viewLength);
	double dW = (double)width / (double)nX, dH = (double)height / (double)nY;
	for (int x = 0; x < nX; x++)  for (int y = 0; y < nY; y++)  result.push_back(origin + Point((x + 0.5) * dW, (y + 0.5) * dH));
	return result;
}

int Map::Room::getArea()
{
	return width * height;
}

