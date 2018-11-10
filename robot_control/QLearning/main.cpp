#include <iostream>
#include <fstream>
#include <string>
#include <unordered_map>
#include "qlearning.h"

int main()
{
    QLearning q("stats.txt");
    q.print_stats();
    return 0;
}
