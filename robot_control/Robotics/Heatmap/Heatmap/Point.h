#pragma once

#include <opencv2\opencv.hpp>

#include <vector>
#include <math.h>

#define ORIGIN (Point(0,0))
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))

class Point;

class Line;
class Edge;

class PolarPoint 
{
public:
	PolarPoint();
	PolarPoint(double d, double angle);
	Point asPoint();
	double d = 0, angle = 0;
};

class Point
{
private:
	double X, Y;

public:
	Point() {}
	Point(double x, double y) : X(x), Y(y) {}
	~Point() {}

	double& x() { 
		return X; 
	}

	double& y() {
		return Y;
	}

	Point operator-(Point p) {
		return Point(X - p.x(), Y - p.y());
	}

	Point operator+(Point p) {
		return Point(X + p.x(), Y + p.y());
	}

	Point operator*(double n) {
		return Point(X * n, Y * n);
	}

	Point operator/(double n) {
		return Point(X / n, Y / n);
	}

	bool operator!=(Point p) {
		return (X != p.x() || Y != p.y());
	}

	bool operator==(Point p) {
		return (X == p.x() && Y == p.y());
	}

	void operator+=(Point p) {
		X += p.x();
		Y += p.y();
	}

	void operator*=(double n) {
		X *= n;
		Y *= n;
	}

	double length() {
		return (sqrt(X*X + Y * Y));
	}

	double getDistance(Point p) {
		return sqrt(pow(X - p.x(), 2) + pow(Y - p.y(), 2));
	}

	double getDistance(Edge e);

	double getDistance(Line l);

	Point normalized() {
		return Point(X / MAX(abs(X),abs(Y)), Y / MAX(abs(X),abs(Y)));
	}

	PolarPoint asPolar() {
		return PolarPoint(sqrt(X*X+Y*Y), atan2((double)Y, (double)X));
	}

	Point rotate(double angle) {
		double x, y;
		x = sin(angle) * Y + cos(angle) * X;
		y = cos(angle) * Y - sin(angle) * X;
		return Point(x, y);
	}

	Point times(Point p) {
		return Point(X * p.x(), Y * p.y());
	}

	cv::Point getCVPoint() {
		return cv::Point(X,Y);
	}

	cv::Point getCVPoint(double x, double y) {
		return cv::Point(X * x, Y * y);
	}
};