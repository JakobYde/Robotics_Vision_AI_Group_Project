#pragma once

#define MAX(a,b) ((a > b) ? (a) : (b))
#define MIN(a,b) ((a < b) ? (a) : (b))


#include <vector>
#include "Point.h"

typedef double GridCoordinateType;

template <class T>
class Grid
{
public:
	Grid() {}
	~Grid() {}

	T& at(GridCoordinateType x, GridCoordinateType y, bool secure = true) {
		if (secure && !inBounds(x, y)) {
			grid.resize(y + 1);
			if (grid.size() > 1) grid[grid.size() - 1].resize(MAX(x + 1, grid[grid.size() - 2].size()));
			else grid[grid.size() - 1].resize(x + 1);
		}	
		return grid[y][x]; 
	}

	T& at(Point<GridCoordinateType> p, bool secure = true) { return at(p.x(), p.y(), secure); }

	unsigned int rows() { return grid.size(); }
	unsigned int cols() { 
		if (rows() > 0) return grid[0].size();
		return 0;
	}

	bool inBounds(Point<GridCoordinateType> p) { return inBounds(p.x(), p.y()); }

	bool inBounds(GridCoordinateType x, GridCoordinateType y) {
		if (y < grid.size() && y >= 0) if (x < grid[y].size() && x >= 0) return true;
		return false;
	}

private:
	std::vector<std::vector<T>> grid;
};

