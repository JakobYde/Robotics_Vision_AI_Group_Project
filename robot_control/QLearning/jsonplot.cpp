#include "jsonplot.h"

JSONPlot::JSONPlot(std::string titel, std::string xlabel, std::string ylabel, std::string plotType)
{
    JSONPlot::titel = titel;
    count = 0;
    j.add("titel", titel);
    j.add("xlabel", xlabel);
    j.add("ylabel", ylabel);
    j.add("plotType", plotType);
}

void JSONPlot::write(std::string filename)
{

    if (filename == " ") filename = titel+"_"+std::to_string(std::time(nullptr))+".json";
    j.write(filename);
    std::cout << "File writen: " << filename << std::endl;
}

void JSONPlot::addData(std::string legend, std::vector<float> xdata, std::vector<float> ydata)
{
    j.add("legend_" + std::to_string(count), legend);
    j.add("xdata_" + std::to_string(count), xdata);
    j.add("ydata_" + std::to_string(count), ydata);
    count++;
}

void JSONPlot::addData(std::string legend, std::vector<int> xdata, std::vector<float> ydata)
{
    j.add("legend_" + std::to_string(count), legend);
    j.add("xdata_" + std::to_string(count), xdata);
    j.add("ydata_" + std::to_string(count), ydata);
    count++;
}

void JSONPlot::addData(std::string legend, std::vector<int> xdata, std::vector<std::vector<float>> ydata)
{
    j.add("legend_" + std::to_string(count), legend);
    j.add("xdata_" + std::to_string(count), xdata);
    j.add("ydata_" + std::to_string(count), ydata);
    count++;
}

JSONPlot::~JSONPlot()
{
}
void makeHistogramJSON(std::string titel, std::vector<float> data, int bins){
    Json j;
    j.add("plotType", "hist");
    j.add("titel", titel);
    if (bins != -1) j.add("bins", bins);
    j.add("data", data);
    j.write(titel+".json");
}
