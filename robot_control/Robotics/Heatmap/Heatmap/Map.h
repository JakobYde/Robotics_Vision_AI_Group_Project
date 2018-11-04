#pragma once

#include "Grid.h"
#include "Point.h"

#include <opencv2/opencv.hpp>

#include <queue>
#include <unordered_set>

#define BLEND_COLOR(a,b,alpha) (a * (1 - alpha) + b * alpha)
#define GET_DISTANCE(a,b) (sqrt((a.x() - b.x())*(a.x() - b.x()) + sqrt((a.y() - b.y())*(a.y() - b.y()))))

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

	void loadImage(cv::Mat img);
	cv::Mat drawMap(drawType type, bool draw = true, drawArguments args = drawArguments());
	std::vector<MapPoint> getPoints();
	std::vector<MapPoint> getPath(MapPoint A, MapPoint B, unsigned int padding);

	struct GreaterH {
		constexpr bool operator()(MapNode* A, MapNode* B){
			return (A->asF > B->asF);
		}
	};

	struct Edge {
		MapPoint A, B;
	};

private:
	double fov, viewDistance;
	int maxDist = 0;
	int calculatedLayers = 0;
	int lastArea = -1;
	std::vector<MapPoint> points;
	std::vector<MapPoint> vertices;
	std::vector<Edge> edges;

	MapPoint dirs[8] = { 
		MapPoint(-1,0), 
		MapPoint(-1,-1),
		MapPoint(0,-1), 
		MapPoint(1,-1), 
		MapPoint(1,0),
		MapPoint(1,1), 
		MapPoint(0,1), 
		MapPoint(-1,1) };

	std::vector<MapPoint> getLine(MapPoint a, MapPoint b);
	void placePoint(MapPoint p);
	bool hasLineOfSight(MapPoint a, MapPoint b);
	void recursivelyFill(MapPoint p);
	bool isDiscoverable(MapPoint p);
	void calculateBrushfire(int layers = -1);
	int getMinNeighbor(MapPoint p);
	Box getLargestBox();
	MapPoint getPointFromIndex(unsigned int i, unsigned int w = 0);
	std::vector<MapPoint> getVertices();
	std::vector<Edge> getEdges();

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
		int wallDistance = 0;

		//Heatmap Values
		double hmDistance = -1;

		//A* Values
		double asH, asG, asF;
		MapNode* asParent;
		bool asSeen, asVisited;


	};

	Grid<MapNode> map;
};

