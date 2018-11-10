#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "qlearning.h"
#include "jsonplot.h"

int main()
{
    //std::string filename, float learningRate, float stepSize, float greedy, float qInitValue
    QLearning q("stats.txt","S0",0,0.4,0.7,0.0);
    std::cout << "Start: " << std::endl;
    q.print_stats();

    for(int i=0;i<5;i++){
        std::cout << "Run: " << i << std::endl;
        q.simulateActionReward();
        q.print_stats();
    }
    std::cout << "Run 5 to 98: " << std::endl;
    for(int i=5;i<100;i++){
        q.simulateActionReward();
    }
    std::cout << "Run: 99" << std::endl;
    q.print_stats();
    q.wirteJSON("statsjson.json");
    return 0;
}
