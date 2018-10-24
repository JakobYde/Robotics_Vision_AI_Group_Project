
#include "Heatmap.h"
#include <iostream>

typedef Grid<int> iGrid;

int main() {
	iGrid grid;

	Point<int> a(1, 1);
	Point<int> b(-1, 2);

	Point<int> ab = a - b;

	int c;
	std::cin >> c;
	return 0;
}