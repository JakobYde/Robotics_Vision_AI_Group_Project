#pragma once

#include "Grid.h"
#include "Point.h"

#include <opencv2/opencv.hpp>

enum nodeType {
	eObstacle,
	eFree,
	eOutside
};

enum drawType {
	eBasic,
	eHeatmap
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
	std::vector<Point<unsigned int>> points;

	std::vector<Point<unsigned int>> getLine(Point<unsigned int> a, Point<unsigned int> b);
	void placePoint(Point<unsigned int> p);
	bool hasLineOfSight(Point<unsigned int> a, Point<unsigned int> b);
	void recursivelyFill(Point<unsigned int> p);

	cv::Vec3b vObstacle = cv::Vec3b(0, 0, 0);
	cv::Vec3b vFree = cv::Vec3b(255, 255, 255);
	cv::Vec3d vUndiscovered = cv::Vec3b(0, 0, 255);
	cv::Vec3d vDiscovered = cv::Vec3b(255, 255, 255);
	cv::Vec3d vOutside = cv::Vec3b(200, 35, 225);
	cv::Vec3d vPoint = cv::Vec3b(255, 0, 0);

	class MapNode
	{
	public:
		nodeType type;

		//Heatmap Values
		double hmDistance = -1;

		//A* Values
		double asH; 
		double asG;
	};

	Grid<MapNode> map;
};

