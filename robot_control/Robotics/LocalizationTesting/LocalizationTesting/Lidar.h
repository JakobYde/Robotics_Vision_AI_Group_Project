#pragma once

#include "Point.h"
#include "Edge.h"
#include "Map.h"

class Lidar
{
public:
	struct transformation {
		Point p;
		double angle;
	};

	struct sNearestPoint {
		Point p;
		Point nearestPoint;
	};

	struct IterationData {
		std::vector<sNearestPoint> nPs;
		Point centroids[2];
	};

    struct BoundedLine {
        Line line;
        Point ends[2];
    };

	Lidar();
	~Lidar();

	void newMeasurement();
	void addMeasurement(PolarPoint p);
	void addMeasurements(std::vector<PolarPoint> points);

	void setMaxRadius(double val);
    void setAngleStep(double val);
    transformation drawProcess(Map* map, transformation position);

	Line leastSquareFit(std::vector<PolarPoint> points);
    std::vector<BoundedLine> getLines();
	std::vector<Point> getCorners();
	transformation ICP(std::vector<Point> cloudA, std::vector<Point> cloudB, transformation position = transformation{ Point(0,0),0 });

private:
    double maxRadius = 0;
    double angleStep = -1;
	Map* map;
	std::vector<PolarPoint> points;
};

