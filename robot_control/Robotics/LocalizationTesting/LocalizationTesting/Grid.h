#pragma once

#define MAX(a,b) ((a > b) ? (a) : (b))
#define MIN(a,b) ((a < b) ? (a) : (b))


#include <vector>
#include "Point.h"

typedef double GridCoordinateType;

template <class T>
class Grid
{
private: 
	class GridIterator;
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

	T& at(Point p, bool secure = true) { return at(p.x(), p.y(), secure); }

	unsigned int rows() { return grid.size(); }
	unsigned int cols() { 
		if (rows() > 0) return grid[0].size();
		return 0;
	}

    /*GridIterator begin() {
		return GridIterator(this, Point(0, 0));
	}

	GridIterator end() {
		return GridIterator();
    }*/

	bool inBounds(Point p) { return inBounds(p.x(), p.y()); }

	bool inBounds(GridCoordinateType x, GridCoordinateType y) {
		if (y < grid.size() && y >= 0) if (x < grid[y].size() && x >= 0) return true;
		return false;
	}

    /*class GridIterator {
	public:
		GridIterator() {};
		GridIterator(Grid<T>* grid, Point point) : grid(grid), point(point) {}
		~GridIterator() {}

		bool operator==(GridIterator gI) {
			return (point == gI.point && grid == gI.grid);
		}

		bool operator!=(GridIterator gI) {
			return (point != gI.point || grid != gI.grid);
		}

		T& operator*() {
			if (grid == NULL) return *(new T);
			return grid->at(point);
		}

		void operator++() {
			point += Point(1, 0);
			if (!grid->inBounds(point)) point = Point(0, point.y() + 1);
			if (!grid->inBounds(point)) {
				point = Point();
				grid = nullptr;
			}
		}

	private:
		Grid<T>* grid = nullptr;
		Point point;
    };*/

private:
	std::vector<std::vector<T>> grid;
};

