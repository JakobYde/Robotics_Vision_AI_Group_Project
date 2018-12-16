#pragma once

#include <opencv2/opencv.hpp>

#include <vector>

#define _USE_MATH_DEFINES
#include <math.h>

#define ORIGIN (Point(0,0))
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))

class Point;

class Line;
class Edge;

class Angle
{
private:
    double val;
  public:
    Angle();
    Angle(double val) : val(val) { }

    operator double() const {
        double _val = val;
        while (_val < 0) _val += M_PI * 2;
        _val = fmod(_val, M_PI * 2);
        return _val;
    }

    Angle operator+(double val) {
        return Angle(this->val + val);
    }

    void operator+=(double val) {
        this->val += val;
    }

    void operator/=(double n) {
        val /= n;
    }

};

class PolarPoint
{
public:
	PolarPoint();
	PolarPoint(double d, double angle);
	Point asPoint();
	operator Point();
	double d = 0, angle = 0;
	
	static std::vector<Point> asPoint(std::vector<PolarPoint> pts);
};

class Point
{
private:
	double X, Y;

public:
	Point() {}
	Point(double val) : X(val), Y(val) {}
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

	void operator-=(Point p) {
		X -= p.x();
		Y -= p.y();
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
		return Point(X / MAX(abs(X), abs(Y)), Y / MAX(abs(X), abs(Y)));
	}

	PolarPoint asPolar() {
		return PolarPoint(sqrt(X*X + Y * Y), atan2((double)Y, (double)X));
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

	Point floor() {
		return Point(std::floor(X), std::floor(Y));
	}

	cv::Point asCV() {
		return cv::Point(X, Y);
	}

	cv::Point asCV(Point offset, cv::Size size) {
		return cv::Point((X + offset.X) * (0.5 * size.width / offset.X), (Y + offset.Y) * (0.5 * size.height / offset.Y));
	}

	cv::Point asCV(double x, double y) {
		return cv::Point(X * x, Y * y);
	}

	static Point getMinPoint(std::vector<Point> pts) {
		Point result;
		if (pts.size() > 0) {
			result = pts[0];
			for (Point pt : pts) result = Point(MIN(pt.x(), result.x()), MIN(pt.y(), result.y()));
		}
		return result;
	}

	static Point getMaxPoint(std::vector<Point> pts) {
		Point result;
		if (pts.size() > 0) {
			result = pts[0];
			for (Point pt : pts) result = Point(MAX(pt.x(), result.x()), MAX(pt.y(), result.y()));
		}
		return result;
	}

    static bool isWithin(Point pt, std::vector<Point> pts) {
        if (pts.size() != 2) return false;
        if (pt.x() < MIN(pts[0].x(), pts[1].x()) || pt.x() > MAX(pts[0].x(), pts[1].x())) return false;
        if (pt.y() < MIN(pts[0].y(), pts[1].y()) || pt.y() > MAX(pts[0].y(), pts[1].y())) return false;
        return true;
    }

};
