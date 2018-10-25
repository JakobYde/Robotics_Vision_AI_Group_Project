#pragma once

#define MAX(a,b) (a > b) ? a : b
#define MIN(a,b) (a < b) ? a : b


#include <vector>
#include "Point.h"

template <class T>
class Grid
{
public:
	Grid() {}
	~Grid() {}

	T& at(unsigned int x, unsigned int y) { 
		if (!inBounds(x, y)) {
			grid.resize(y + 1);
			if (grid.size() > 1) grid[grid.size() - 1].resize(MAX(x + 1, grid[grid.size() - 2].size()));
			else grid[grid.size() - 1].resize(x + 1);
		}	
		return grid[y][x]; 
	}

	T& at(Point<unsigned int> p) { return at(p.x(), p.y()); }

	unsigned int rows() { return grid.size(); }
	unsigned int cols() { 
		if (rows() > 0) return grid[0].size();
		return 0;
	}

private:
	std::vector<std::vector<T>> grid;

	bool inBounds(unsigned int x, unsigned int y) { 
		if (y < grid.size()) if (x < grid[y].size()) return true;
		return false;
	}
};

