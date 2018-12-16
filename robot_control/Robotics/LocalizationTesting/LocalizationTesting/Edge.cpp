#include "Edge.h"


Edge::Edge()
{
}

Edge::Edge(Point A, Point B) : A(A), B(B)
{
}

Edge::~Edge()
{
}

std::vector<Point> Edge::getPoints(double stepSize)
{
	std::vector<Point> line;
	if (Point(floor(A.x()), floor(A.y())) != Point(floor(B.x()), floor(B.y()))) {
		int dX = (int)B.x() - (int)A.x(), dY = (int)B.y() - (int)A.y(), l, g;
		bool findingX = false;
		if (abs(dX) > abs(dY)) l = dY, g = dX;
		else l = dX, g = dY, findingX = true;
		line = std::vector<Point>(abs(g) + 1);

		double slope = (double)l / (double)g;
		int inc = stepSize * g / abs(g);

		for (int i = 0; abs(i) < abs(g) + 1; i += inc) {
			Point p;
			if (findingX) p = Point(i * slope + (int)A.x(), i + (int)A.y());
			else p = Point(i + (int)A.x(), i * slope + (int)A.y());

			line[abs(i)] = p;
		}
	}
	return line;
}

Line::Line()
{
}

Line::Line(double a, double d) : a(a), d(d)
{
	A = tan(a - M_PI/2);
	double x = cos(a) * d, y = sin(a) * d;
	B = y - x * A;
}

Line::~Line()
{
}

double Line::x(double y)
{
	return (y - B) / A;
}

double Line::y(double x)
{
	return A * x + B;
}

Edge Line::asEdge(Point A, Point B, bool extensive)
{
	double minX, maxX, minY, maxY;
	if (extensive) minX = MIN(MIN(A.x(), x(A.y())), MIN(B.x(), x(B.y())));
	else minX = MIN(MAX(A.x(), x(A.y())), MAX(B.x(), x(B.y())));
	if (extensive) maxX = MAX(MAX(A.x(), x(A.y())), MAX(B.x(), x(B.y())));
	else maxX = MAX(MIN(A.x(), x(A.y())), MIN(B.x(), x(B.y())));
	Point edgeA(minX, y(minX));
	Point edgeB(maxX, y(maxX));
	return Edge(edgeA, edgeB);
}

bool Line::isColinear(Line l, double threshold)
{
	double dAngle = abs(a - l.a), dDistance = abs(d - l.d);
	return (dAngle <= threshold && dDistance <= threshold);
}

Point Line::getIntersection(Line l)
{
	double x, y;
	x = (l.B - B) / (A - l.A);
	y = this->y(x);
	return Point(x, y);
}
