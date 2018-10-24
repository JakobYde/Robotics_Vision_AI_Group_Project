#pragma once

#include <vector>

template <class T>
class Grid
{
public:
	Grid() {}
	~Grid() {}

	T& at(unsigned int x, unsigned int y) { 
		if (!inBounds(x, y)) {
			grid.resize(y + 1);
			for (int i = 0; i < grid.size(); i++)  grid[i].resize(x + 1);
		}	
		return grid[y][x]; 
	}

	unsigned int rows() { return grid.size(); }
	unsigned int cols() { 
		if (rows() < 0) return grid[0].size();
		return 0;
	}

private:
	std::vector<std::vector<T>> grid;

	bool inBounds(unsigned int x, unsigned int y) { 
		if (y < grid.size()) if (x < grid.size()) return true;
		return false;
	}
};

