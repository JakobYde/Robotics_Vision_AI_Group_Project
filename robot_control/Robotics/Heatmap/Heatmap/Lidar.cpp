#include "Lidar.h"



Lidar::Lidar()
{
}


Lidar::~Lidar()
{
}

void Lidar::newMeasurement()
{
	points.clear();
}

void Lidar::addMeasurement(PolarPoint p)
{
	points.push_back(p);
}

void Lidar::addMeasurements(std::vector<PolarPoint> _points)
{
	points.insert(points.end(), _points.begin(), _points.end());
}

Line Lidar::leastSquareFit(std::vector<PolarPoint> points)
{
	struct Pt {
		double d, angle;
	};

	// Polar coordinates of the points (-2, 1) and (5, 1) respectively.
	std::vector<Pt> pts = { {2.2360679774997898, 2.6779450445889870 } , { 5.0990195135927845, 0.19739555984988075 } };

	double a = 0, r = 0;

	double n = points.size();
	double sumOfWeights = n;

	double num1 = 0, num2 = 0, den1 = 0, den2 = 0;
	for (int i = 0; i < n; i++) {
		double iP = points[i].d, iTheta = points[i].angle;

		num1 += iP * iP * sin(2 * iTheta);
		den1 += iP * iP * cos(2 * iTheta);

		for (int j = 0; j < n; j++) {
			double jP = points[j].d, jTheta = points[j].angle;

			num2 += iP * jP * cos(iTheta) * sin(jTheta);
			den2 += iP * jP * cos(iTheta + jTheta);

		}
	}

	a = 0.5 * atan2((num1 - (2.0 / sumOfWeights) * num2), (den1 - (1.0 / sumOfWeights) * den2)) + M_PI/2;
	
	for (int i = 0; i < n; i++) r += points[i].d * cos(points[i].angle - a);
	r /= sumOfWeights;

	return Line{ a, r };
}

std::vector<Line> Lidar::getLines()
{
	struct PointSet {
		std::vector<PolarPoint> points;
		Line line;
		bool closeEnough = false;
	};

	std::vector<PointSet> pointSets;
	pointSets.push_back(PointSet{ points, leastSquareFit(points) });

	double distThreshold = 0.4;

	bool closeEnough = false;

	while (!closeEnough) {
		closeEnough = true;
		for (int i = 0; i < pointSets.size(); i++) {
			PointSet set = pointSets[i];
			if (!set.closeEnough) {
				double iMaxDistance = 0, maxDist = set.points[0].asPoint().getDistance(set.line);
				for (int p = 1; p < set.points.size(); p++) {
					double dist = set.points[i].asPoint().getDistance(set.line);
					if (dist > maxDist) iMaxDistance = p, maxDist = dist;
				}
				if (maxDist > distThreshold) {
					pointSets.erase(pointSets.begin() + i);
					std::vector<PolarPoint> l1(set.points.begin(), set.points.begin() + iMaxDistance), l2(set.points.begin() + iMaxDistance, set.points.end());
					pointSets.push_back(PointSet{ l1, leastSquareFit(l1) });
					pointSets.push_back(PointSet{ l2, leastSquareFit(l2) });
					closeEnough = false;
				}
				else set.closeEnough = true;
			}
		}
	}

	std::vector<Line> lines(pointSets.size());
	for (PointSet set : pointSets) lines.push_back(set.line);

	return lines;
}

std::vector<Point> Lidar::getCorners()
{
	std::vector<Point> corners;
	std::vector<Line> lines = getLines();

	for (std::vector<Line>::iterator lA = lines.begin(); lA != lines.end(); lA++) {
		for (std::vector<Line>::iterator lB = lA + 1; lB != lines.end(); lB++) {
			if (abs(lA->a - lB->a) > M_PI_4 / 2) {
				double x, y;
				x = (lA->B - lB->B) / (lA->A - lB->A);
				y = lA->A * x + lA->B;
				corners.push_back(Point(x, y));
			}
		}
	}

	return corners;
	return std::vector<Point>();
}
