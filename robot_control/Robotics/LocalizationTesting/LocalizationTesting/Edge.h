#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

#include "Point.h"

class Point;

class Edge
{
public:
	Edge();
	Edge(Point A, Point B);
	~Edge();

	std::vector<Point> getPoints(double stepSize = 1);
	Point A, B;

	Edge operator+(Point p) {
		return (Edge(A + p, B + p));
	}

	void operator+=(Point p) {
		A += p;
		B += p;
	}

	void operator*=(double n) {
		A *= n;
		B *= n;
	}
};

class Line 
{
public:
	Line();
	Line(double a, double d);
	~Line();

	double x(double y);
	double y(double x);

	Edge asEdge(Point A, Point B, bool extensive = true);
    bool isColinear(Line l, double threshold = 0.05);
	Point getIntersection(Line l);

	// Polar representation:
	// a: Angle to x-axis.
	// d: Distance to origin.
	// 
	// Cartesian representation:
	// A: Slope of line.
	// B: Intersection of y-axis.
	double a, d, A, B;

};
