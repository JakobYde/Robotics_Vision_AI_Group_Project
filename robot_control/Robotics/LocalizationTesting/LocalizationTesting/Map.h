#pragma once

#include "Grid.h"
#include "Point.h"
#include "Edge.h"
#include "GraphDrawer.h"

#include <opencv2/opencv.hpp>

#include <queue>
#include <unordered_set>

#define BLEND_COLOR(a,b,alpha) (a * (1 - alpha) + b * alpha)
#define GET_DISTANCE(a,b) (sqrt((a.x() - b.x())*(a.x() - b.x()) + sqrt((a.y() - b.y())*(a.y() - b.y()))))

#define ROBOT_WIDTH 1.01
#define BALL_WIDTH 1

#define ROOM_DEFAULT -1
#define ROOM_PARTITION -2

#define HMDISTANCE_DEFAULT -1


enum nodeType {
	eObstacle,
	eFree,
	eOutside
};

enum drawType {
	eBasic,
	eHeatmap,
	eBrushfire,
	eCells,
	ePath,
	eGeometry
};

struct drawArguments {
	Point A;
	Point B;
	unsigned int padding;
};

struct Box {
	unsigned int x, y, w, h;
};

class Map
{
private: class MapNode;
public:
	class Path
	{
	public:
		Path() {}
		Path(std::vector<Point> pts) : points(pts), length(pts.size()) { }
		~Path() {};

		bool operator>(Path p) const {
			return length > p.length;
		}

		bool operator==(Path p) {
			if (length != p.length) return false;
			for (int i = 0; i < points.size(); i++) if (points[i] != p.points[i]) return false;
			//if (points != p.points) return false;
			return true;
		}

		Path reverse() {
			Path p = *this;
			std::reverse(p.points.begin(), p.points.end());
			return p;
		}

		std::vector<Point>::iterator begin() { return points.begin(); }
		std::vector<Point>::iterator end() { return points.end(); }
		void erase(int i) { points.erase(points.begin() + 1); }

		std::vector<Point> points = std::vector<Point>();
		double length = 0;
	};

	class Plan {
	public:
		Plan() {}
		Plan(std::vector<Path> paths) : paths(paths) { 
			for (Path p : paths) length += p.points.size(); 
		}
		Plan(std::vector<Path> paths, double length) : paths(paths), length(length) { }
		~Plan() {};

		double heuristic() const {
			return length / (pointsVisited.size() * pointsVisited.size());
			//return length / pointsVisited.size();
			return length;
		}

		bool operator>(const Plan p) const {
			return heuristic() > p.heuristic();
		}

		std::vector<Path>::iterator begin() { return paths.begin(); }
		std::vector<Path>::iterator end() { return paths.end(); }

		Plan operator+(Path pth) const {
			std::vector<Path> newPaths = paths;
			newPaths.push_back(pth);
			Plan newPlan(newPaths, length + pth.length);
			newPlan.pointsVisited = pointsVisited;
			return newPlan;
		}

		bool isEquivalent(Plan p) {
			if (pointsVisited.size() != p.pointsVisited.size()) return false;
			if (pointsVisited[pointsVisited.size() - 1] != p.pointsVisited[p.pointsVisited.size() - 1]) return false;
			std::vector<int> copyA = pointsVisited, copyB = p.pointsVisited;
			std::sort(copyA.begin() + 1, copyA.end() - 1);
			std::sort(copyB.begin() + 1, copyB.end() - 1);
			for (int i = 0; i < pointsVisited.size() - 1; i++) if (copyA[i] != copyB[i]) return false;
			return true;
		}


		std::vector<int> pointsVisited;
		std::vector<Path> paths;
		double length = 0;


	};

	Map();
	Map(double fov, double viewDistance);
	~Map();

	void loadImage(cv::Mat img, int upscaling = 1);
	cv::Mat drawMap(drawType type, bool draw = true, drawArguments args = drawArguments());
	Plan getPlan();
	std::vector<Point> getPoints();

	struct GreaterH {
		constexpr bool operator()(MapNode* A, MapNode* B){
			return (A->asF > B->asF);
		}
	};

private:
	class Room {
	public:
		Room() {}
		Room(Point origin, int width, int height) : origin(origin), width(width), height(height), area(width * height) {};
		Room(Point A, Point B);
		~Room() {};

		std::vector<Point> getPoints(double viewDistance);
		int getArea();

		bool operator<(const Room r) const {
			return area < r.area;
		}

		std::vector<Point>::iterator begin() {
			if (points.size() < area) for (int i = 0; i < width * height; i++) points.push_back(Point(i % width, i / width) + origin);
			return points.begin();
		}

		std::vector<Point>::iterator end() {
			return points.end();
		}

		int width = 0, height = 0, area = 0;
		Point origin = Point(0,0);
		std::vector<Point> points;
	};

	double fov, viewDistance, scale = 1;
	std::vector<Point> points;
	std::priority_queue<Room> rooms;

	Point dirs[8] = { 
		Point(-1,0), 
		Point(-1,-1),
		Point(0,-1), 
		Point(1,-1), 
		Point(1,0),
		Point(1,1), 
		Point(0,1), 
		Point(-1,1) };

	//  -- UTILITY --
	bool hasLineOfSight(Point a, Point b, double wallDistanceThreshold = 0);
	void recursivelyFill(Point p);
	bool isDiscoverable(Point p);
	Point getPointFromIndex(unsigned int i, unsigned int w = 0);

	//  -- HEATMAP --
	void placePoint(Point p);

	//  -- GEOMETRY --
	std::vector<Point> wallVertices;
	std::vector<Point> floorVertices;
	std::vector<Edge> edges;

	std::vector<Point> getVertices(nodeType type);
	std::vector<Edge> getEdges();

	//  -- CELL DECOMPOSITION --
	//  -- CONSERVATIVE REGIONS -- 
	void seperateIntoRooms();
	bool isScouted(Room room);

	//  -- BRUSHFIRE --
	double maxDist = 0;

	void calculateBrushfire();
	double getMinNeighbor(Point p);

	// -- PATH PLANNING --
	Path getPath(Point A, Point B, double padding = ROBOT_WIDTH);
	Path simplifyPath(Path path);


	cv::Vec3b vObstacle = cv::Vec3b(0, 0, 0);
	cv::Vec3b vFree = cv::Vec3b(255, 255, 255);
	cv::Vec3b vRed = cv::Vec3b(0, 0, 255);
	cv::Vec3b vDiscovered = cv::Vec3b(255, 255, 255);
	cv::Vec3b vOutside = cv::Vec3b(80, 80, 80);
	cv::Vec3b vBlue = cv::Vec3b(255, 0, 0);
	cv::Vec3b vGreen = cv::Vec3b(0, 255, 0);

	class MapNode
	{
	public:
		//Base Values
		nodeType type;
		Point position;

		//Room Values
		double wallDistance = -1;
		int roomNumber = ROOM_DEFAULT;
		cv::Vec3b roomColor = cv::Vec3b(0, 0, 0);

		//Heatmap Values
		double hmDistance = HMDISTANCE_DEFAULT;

		//A* Values
		double asH, asG, asF;
		MapNode* asParent;
		bool asSeen, asVisited;
	};


	Grid<MapNode> map;
};

