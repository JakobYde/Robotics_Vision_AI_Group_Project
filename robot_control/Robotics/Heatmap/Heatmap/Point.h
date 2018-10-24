#pragma once
template <typename T>
class Point
{
public:
	Point() {};
	Point(T x, T y) : X(x), Y(y) {};
	~Point() {};

	T& x() { 
		return X; 
	}

	T& y() {
		return Y;
	}

	Point<T> operator-(Point<T> p) {
		return Point<T>(X - p.x(), Y - p.y());
	}

private:
	T X, Y;
};

