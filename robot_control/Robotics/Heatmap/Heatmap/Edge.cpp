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
	if (A != B) {
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

Edge Line::asEdge(Point A, Point B)
{
	double minX, maxX, minY, maxY;
	minX = MIN(MIN(A.x(), (A.y() - Line::B) / Line::A), MIN(B.x(), (B.y() - Line::B) / Line::A));
	maxX = MAX(MAX(A.x(), (A.y() - Line::B) / Line::A), MAX(B.x(), (B.y() - Line::B) / Line::A));
	Point edgeA(minX, Line::A * minX + Line::B);
	Point edgeB(maxX, Line::A * maxX + Line::B);
	return Edge(edgeA, edgeB);
}
