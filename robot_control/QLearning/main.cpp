#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "qlearning.h"
#include "jsonplot.h"

int main()
{
    QLearning q("stats.txt", 0);
    q.print_stats();
    return 0;
}
