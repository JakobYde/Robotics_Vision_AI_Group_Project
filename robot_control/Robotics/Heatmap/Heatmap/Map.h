#pragma once

#include "Grid.h"
#include "Point.h"

#include <opencv2/opencv.hpp>

#define BLEND_COLOR(a,b,alpha) (a * (1 - alpha) + b * alpha)

enum nodeType {
	eObstacle,
	eFree,
	eOutside
};

enum drawType {
	eBasic,
	eHeatmap,
	eBrushfire,
	eRooms
};

class Map
{
public:
	Map();
	Map(double fov, double viewDistance);
	~Map();

	void loadImage(cv::Mat img);
	void drawMap(drawType type);
	std::vector<Point<unsigned int>> getPoints();

private:
	double fov, viewDistance;
	int maxDist = 0;
	std::vector<Point<unsigned int>> points;

	std::vector<Point<unsigned int>> getLine(Point<unsigned int> a, Point<unsigned int> b);
	void placePoint(Point<unsigned int> p);
	bool hasLineOfSight(Point<unsigned int> a, Point<unsigned int> b);
	void recursivelyFill(Point<unsigned int> p);
	bool isDiscoverable(Point<unsigned int> p);
	void seperateIntoRooms();
	int getMinNeighbor(Point<unsigned int> p);
	std::vector<Point<unsigned int>> getPath(Point<unsigned int> A, Point<unsigned int> B);

	cv::Vec3b vObstacle = cv::Vec3b(0, 0, 0);
	cv::Vec3b vFree = cv::Vec3b(255, 255, 255);
	cv::Vec3b vUndiscovered = cv::Vec3b(0, 0, 255);
	cv::Vec3b vDiscovered = cv::Vec3b(255, 255, 255);
	cv::Vec3b vOutside = cv::Vec3b(200, 35, 225);
	cv::Vec3b vPoint = cv::Vec3b(255, 0, 0);

	class MapNode
	{
	public:
		nodeType type;

		//Heatmap Values
		double hmDistance = -1;

		//Room Values
		int roomNumber = -1;
		int distanceFromDiscovered = INT_MAX;

		//A* Values
		double asH; 
		double asG;
	};

	Grid<MapNode> map;
};

