#pragma once

#include "Grid.h"
#include "Point.h"
#include "Edge.h"

#include <opencv2/opencv.hpp>

#include <queue>
#include <unordered_set>

#define BLEND_COLOR(a,b,alpha) (a * (1 - alpha) + b * alpha)
#define GET_DISTANCE(a,b) (sqrt((a.x() - b.x())*(a.x() - b.x()) + sqrt((a.y() - b.y())*(a.y() - b.y()))))

#define ROBOT_WIDTH 0.5
#define BALL_WIDTH 1

#define ROOM_DEFAULT -1
#define ROOM_PARTITION -2


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
	Map();
	Map(double fov, double viewDistance);
	~Map();

	void loadImage(cv::Mat img, int upscaling = 1);
	cv::Mat drawMap(drawType type, bool draw = true, drawArguments args = drawArguments());
	std::vector<Point> getPoints();

	struct GreaterH {
		constexpr bool operator()(MapNode* A, MapNode* B){
			return (A->asF > B->asF);
		}
	};

private:
	double fov, viewDistance, scale = 1;
	std::vector<Point> points;

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

	//  -- BRUSHFIRE --
	double maxDist = 0;

	void calculateBrushfire();
	double getMinNeighbor(Point p);

	// -- PATH PLANNING --
	std::vector<Point> getPath(Point A, Point B, double padding = ROBOT_WIDTH);
	std::vector<Point> simplifyPath(std::vector<Point> path);


	cv::Vec3b vObstacle = cv::Vec3b(0, 0, 0);
	cv::Vec3b vFree = cv::Vec3b(255, 255, 255);
	cv::Vec3b vUndiscovered = cv::Vec3b(0, 0, 255);
	cv::Vec3b vDiscovered = cv::Vec3b(255, 255, 255);
	cv::Vec3b vOutside = cv::Vec3b(80, 80, 80);
	cv::Vec3b vPoint = cv::Vec3b(255, 0, 0);

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
		double hmDistance = -1;

		//A* Values
		double asH, asG, asF;
		MapNode* asParent;
		bool asSeen, asVisited;


	};

	Grid<MapNode> map;
};

