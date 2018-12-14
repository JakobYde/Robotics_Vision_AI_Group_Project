#include "json.h"

Json::Json()
{
}


Json::~Json()
{
}

void Json::add(std::string name, std::string value)
{
    jsonValue val(name, '"'+value+'"');
    addJsonValue(val);
}
template<typename T>
void Json::add(std::string name, T value)
{
    jsonValue val(name, std::to_string(value));
    addJsonValue(val);
}
template<typename T>
void Json::add(std::string name, std::vector<T> value)
{
    std::string valueString = "[ ";

    for (int i = 0; i < value.size()-1; i++) {
        valueString += std::to_string(value.at(i)) + ", ";
    }
    valueString += std::to_string(value.back()) + " ]";

    jsonValue val(name, valueString);
    addJsonValue(val);
}
void Json::add(std::string name, std::vector<std::string> value)
{
    std::string valueString = "[ ";

    for (int i = 0; i < value.size() - 1; i++) {
        valueString += '"' + value.at(i) + '"' + ", ";
    }
    valueString += '"' + value.back() + '"' + " ]";

    jsonValue val(name, valueString);
    addJsonValue(val);
}

void Json::write(std::string filename)
{
    std::ofstream jsonfile;
    jsonfile.open(filename);
    jsonfile << to_string();
    jsonfile.close();
}

std::string Json::to_string()
{
    std::string jsonString = "{";
    for(int i = 0; i < valuse.size()-1; i++)
    {
        jsonString += '"' + valuse.at(i).name + '"' + ": " + valuse.at(i).value + ", \n";
    }
    jsonString += '"' + valuse.back().name + '"' + ": " + valuse.back().value + "\n";

    jsonString += "}";
    return jsonString;
}

void Json::addJsonValue(jsonValue val)
{
    for (int i = 0; i < valuse.size(); i++){
        if (val.name == valuse.at(i).name) {
            valuse.at(i) = val;
            return;
        }
    }
    valuse.push_back(val);
}

void makeJOSNPlotData(std::string titel, std::string xlabel, std::string ylabel, std::vector<float> xdata, std::vector<float> ydata, std::string plotType)
{
    Json j;

    j.add("plotType", plotType);
    j.add("titel", titel);
    j.add("xlabel", xlabel);
    j.add("ylabel", ylabel);
    j.add("xdata", xdata);
    j.add("ydata", ydata);

    j.write(titel + ".json");
}

void Json::csvCreate(std::vector<float> data, std::string filename)
{
    std::stringstream ss;
    for(int i = 0; i < data.size(); i++)
    {
        ss <<  data[i] << ",";
    }
    std::ofstream newfile;
    newfile.open(filename);
    newfile << "[";
     newfile << ss.str();
    newfile << "]";
    newfile.close();
    return;
}
