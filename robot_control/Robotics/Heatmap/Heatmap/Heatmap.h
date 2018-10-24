#pragma once

#include "Grid.h"
#include "Point.h"

class Heatmap
{
public:
	Heatmap();
	~Heatmap();


private:
	std::vector<Point<unsigned int>> points;

	class MapElement
	{
		double distance;
	};

	Grid<MapElement> map;
};

