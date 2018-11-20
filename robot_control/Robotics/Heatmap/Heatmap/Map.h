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

typedef Point<GridCoordinateType> MapPoint;

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
	MapPoint A;
	MapPoint B;
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
	std::vector<MapPoint> getPoints();

	struct GreaterH {
		constexpr bool operator()(MapNode* A, MapNode* B){
			return (A->asF > B->asF);
		}
	};

private:
	double fov, viewDistance, scale = 1;
	std::vector<MapPoint> points;

	MapPoint dirs[8] = { 
		MapPoint(-1,0), 
		MapPoint(-1,-1),
		MapPoint(0,-1), 
		MapPoint(1,-1), 
		MapPoint(1,0),
		MapPoint(1,1), 
		MapPoint(0,1), 
		MapPoint(-1,1) };

	//  -- UTILITY --
	bool hasLineOfSight(MapPoint a, MapPoint b, double wallDistanceThreshold = 0);
	void recursivelyFill(MapPoint p);
	bool isDiscoverable(MapPoint p);
	MapPoint getPointFromIndex(unsigned int i, unsigned int w = 0);

	//  -- HEATMAP --
	void placePoint(MapPoint p);

	//  -- GEOMETRY --
	std::vector<MapPoint> vertices;
	std::vector<Edge> edges;

	std::vector<MapPoint> getVertices();
	std::vector<Edge> getEdges();

	//  -- BRUSHFIRE --
	double maxDist = 0;

	void calculateBrushfire();
	double getMinNeighbor(MapPoint p);

	// -- PATH PLANNING --
	std::vector<MapPoint> getPath(MapPoint A, MapPoint B, double padding = ROBOT_WIDTH);
	std::vector<MapPoint> simplifyPath(std::vector<MapPoint> path);


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
		MapPoint position;

		//Room Values
		double wallDistance = -1;

		//Heatmap Values
		double hmDistance = -1;

		//A* Values
		double asH, asG, asF;
		MapNode* asParent;
		bool asSeen, asVisited;


	};

	Grid<MapNode> map;
};

