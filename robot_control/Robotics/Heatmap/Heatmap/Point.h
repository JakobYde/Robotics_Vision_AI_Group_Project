#pragma once
#include <vector>
#include <opencv2\opencv.hpp>

template <typename T>
class Point;

#define ORIGIN (Point<double>(0,0))

class Edge;
class Line;

class PolarPoint 
{
public:
	PolarPoint();
	PolarPoint(double d, double angle);
	Point<double> asPoint();
	Point<unsigned int> asUIntPoint();
	double d = 0, angle = 0;

};

template <typename T>
class Point
{
private:
	T X = 0, Y = 0;

public:
	Point() {}
	Point(T x, T y) : X(x), Y(y) {}
	~Point() {}

	T& x() { 
		return X; 
	}

	T& y() {
		return Y;
	}

	Point<T> operator-(Point<T> p) {
		return Point<T>(X - p.x(), Y - p.y());
	}

	Point<T> operator+(Point<T> p) {
		return Point<T>(X + p.x(), Y + p.y());
	}

	Point<T> operator*(double n) {
		return Point<T>(X * n, Y * n);
	}

	Point<T> operator/(double n) {
		return Point<T>(X / n, Y / n);
	}

	bool operator!=(Point<T> p) {
		return (X != p.x() || Y != p.y());
	}

	bool operator==(Point<T> p) {
		return (X == p.x() && Y == p.y());
	}

	void operator+=(Point<T> p) {
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

	double getDistance(Point<T> p) {
		return sqrt(pow(X - p.x(), 2) + pow(Y - p.y(), 2));
	}

	double getDistance(Edge e) {
		std::vector<Point<T>> pts = e.getPoints();
		double dist = INT_MAX;
		for (Point<T> p : pts) dist = MIN(dist, getDistance(p));
		return dist;
	}

	double getDistance(Line l) {
		T x, y, tA = tan(l.a);

		x = (X + Y * tA) / (1 + tA*tA);
		y = x * tA;
		Point<T> parallelPoint(x,y);
		return abs(l.d - parallelPoint.getDistance(ORIGIN));
	}

	Point<T> normalized() {
		return Point<T>(X / MAX(abs(X),abs(Y)), Y / MAX(abs(X),abs(Y)));
	}

	PolarPoint asPolar() {
		return PolarPoint(sqrt(X*X+Y*Y), atan2((double)Y, (double)X));
	}

	cv::Point getCVPoint() {
		return cv::Point(X,Y);
	}
};
