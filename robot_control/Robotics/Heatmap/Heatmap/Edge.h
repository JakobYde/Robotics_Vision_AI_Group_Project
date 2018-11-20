#pragma once

#include "Point.h"

#define _USE_MATH_DEFINES
#include <math.h>

class Line;

class Edge
{
public:
	Edge();
	Edge(Point<double> A, Point<double> B);
	~Edge();

	std::vector<Point<double>> getPoints(double stepSize = 1);
	Point<double> A = ORIGIN, B = ORIGIN;

	Edge operator+(Point<double> p) {
		return (Edge(A + p, B + p));
	}

	void operator+=(Point<double>p) {
		A += p;
		B += p;
	}

	void operator*=(double n) {
		A *= n;
		B *= n;
	}
};

class Line {
public:
	Line();
	Line(double a, double d);
	~Line();

	Edge asEdge(Point<double> A, Point<double> B);

	// Polar representation:
	// a: Angle to x-axis.
	// d: Distance to origin.
	// 
	// Cartesian representation:
	// A: Slope of line.
	// B: Intersection of y-axis.
	double a, d, A, B;

};
