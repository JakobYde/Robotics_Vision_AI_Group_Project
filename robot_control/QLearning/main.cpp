#include <iostream>
#include <fstream>
#include <string>
#include "qlearning.h"

int main()
{
    std::string line;
    std::ifstream myfile ("stats.txt");
    if (myfile.is_open())
    {
        while ( std::getline (myfile,line) )
        {
          std::cout << line << '\n';
        }
        myfile.close();
    }
    else std::cout << "Unable to open file\n";

    return 0;
}
