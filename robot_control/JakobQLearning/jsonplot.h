#ifndef JSONPLOT_H
#define JSONPLOT_H
#include "json.h"
#include <string>
#include <fstream>
#include <iostream>

void makeHistogramJSON(std::string titel, std::vector<float> data, int bins = -1);

class JSONPlot
{
protected:
    Json j;
    int count;
    std::string titel;
public:
    JSONPlot(std::string titel, std::string xlabel, std::string ylabel, std::string plotType = "plot");

    void addData(std::string legend, std::vector<float> xdata, std::vector<float> ydata);
    void write(std::string filename = " ");
    ~JSONPlot();
};

#endif // JSONPLOT_H
