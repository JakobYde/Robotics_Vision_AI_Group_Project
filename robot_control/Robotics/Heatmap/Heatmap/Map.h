#pragma once

#include "Grid.h"
#include "Point.h"

#include <opencv2/opencv.hpp>

#include <queue>

#define BLEND_COLOR(a,b,alpha) (a * (1 - alpha) + b * alpha)
#define GET_DISTANCE(a,b) (sqrt((a.x() - b.x())*(a.x() - b.x()) + sqrt((a.y() - b.y())*(a.y() - b.y()))))

enum nodeType {
	eObstacle,
	eFree,
	eOutside
};

enum drawType {
	eBasic,
	eHeatmap,
	eBrushfire,
	eRooms,
	ePath,
	eLargestBox
};

struct drawArguments {
	Point<unsigned int> A;
	Point<unsigned int> B;
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

	void loadImage(cv::Mat img);
	cv::Mat drawMap(drawType type, bool draw = true, drawArguments args = drawArguments());
	std::vector<Point<unsigned int>> getPoints();
	std::vector<Point<unsigned int>> getPath(Point<unsigned int> A, Point<unsigned int> B, unsigned int padding);

	struct GreaterH {
		constexpr bool operator()(MapNode* A, MapNode* B){
			return (A->asF > B->asF);
		}
	};

private:
	double fov, viewDistance;
	int maxDist = 0;
	int calculatedLayers = 0;
	int lastArea = -1;
	std::vector<Point<unsigned int>> points;

	Point<unsigned int> dirs[8] = { Point<unsigned int>(-1,0), Point<unsigned int>(-1,-1), Point<unsigned int>(0,-1), Point<unsigned int>(1,-1), Point<unsigned int>(1,0), Point<unsigned int>(1,1), Point<unsigned int>(0,1), Point<unsigned int>(-1,1) };

	std::vector<Point<unsigned int>> getLine(Point<unsigned int> a, Point<unsigned int> b);
	void placePoint(Point<unsigned int> p);
	bool hasLineOfSight(Point<unsigned int> a, Point<unsigned int> b);
	void recursivelyFill(Point<unsigned int> p);
	bool isDiscoverable(Point<unsigned int> p);
	void seperateIntoRooms(int layers = -1);
	int getMinNeighbor(Point<unsigned int> p);
	Box getLargestBox();

	cv::Vec3b vObstacle = cv::Vec3b(0, 0, 0);
	cv::Vec3b vFree = cv::Vec3b(255, 255, 255);
	cv::Vec3b vUndiscovered = cv::Vec3b(0, 0, 255);
	cv::Vec3b vDiscovered = cv::Vec3b(255, 255, 255);
	cv::Vec3b vOutside = cv::Vec3b(200, 35, 225);
	cv::Vec3b vPoint = cv::Vec3b(255, 0, 0);

	class MapNode
	{
	public:
		//A* Values
		double asH, asG, asF;
		MapNode* asParent;
		bool asSeen, asVisited;

		//Base Values
		nodeType type;
		Point<unsigned int> position;

		//Heatmap Values
		double hmDistance = -1;

		//Room Values
		int roomNumber = -1;
		int distanceFromDiscovered = INT_MAX;

		//Rectangle
		bool isRectangle = false;

	};

	Grid<MapNode> map;
};

