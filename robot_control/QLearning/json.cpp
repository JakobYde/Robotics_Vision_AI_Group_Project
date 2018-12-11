#include "json.h"

Json::Json()
{
}


Json::~Json()
{
}
void Json::add(std::string name, float value)
{
    jsonValue val(name, std::to_string(value));
    addJsonValue(val);
}
void Json::add(std::string name, int value)
{
    jsonValue val(name, std::to_string(value));
    addJsonValue(val);
}
void Json::add(std::string name, std::string value)
{
    jsonValue val(name, '"'+value+'"');
    addJsonValue(val);
}
void Json::add(std::string name, std::vector<float> value)
{
    std::string valueString = "[ ";

    for (unsigned int i = 0; i < value.size()-1; i++) {
        valueString += std::to_string(value.at(i)) + ", ";
    }
    valueString += std::to_string(value.back()) + " ]";

    jsonValue val(name, valueString);
    addJsonValue(val);
}

void Json::add(std::string name, std::vector<std::vector<float>> value)
{
    std::string valueString = "[ ";

    for (unsigned int i = 0; i < value.size(); i++) {

        std::string valueString2 = "[ ";

        for (unsigned int j = 0; j < value.at(i).size()-1; j++) {
            valueString2 += std::to_string(value.at(i).at(j)) + ", ";
        }
        valueString2 += std::to_string(value.at(i).back()) + " ]";

        if(i < value.size()-1) valueString += valueString2 + ", ";
        else valueString += valueString2 + " ]";
    }


    jsonValue val(name, valueString);
    addJsonValue(val);
}

void Json::add(std::string name, std::vector<int> value)
{
    std::string valueString = "[ ";

    for (unsigned int i = 0; i < value.size()-1; i++) {
        valueString += std::to_string(value.at(i)) + ", ";
    }
    valueString += std::to_string(value.back()) + " ]";

    jsonValue val(name, valueString);
    addJsonValue(val);
}
void Json::add(std::string name, std::vector<std::string> value)
{
    std::string valueString = "[ ";

    for (unsigned int i = 0; i < value.size() - 1; i++) {
        valueString += '"' + value.at(i) + '"' + ", ";
    }
    valueString += '"' + value.back() + '"' + " ]";

    jsonValue val(name, valueString);
    addJsonValue(val);
}

void Json::add(std::string name, Json value){
    std::string jsonValueString = value.to_string();
    jsonValue val(name, jsonValueString);
    addJsonValue(val);
}

void Json::add(std::string name,  std::vector<Json> value){
    std::string valueString = "[ ";

    for (unsigned int i = 0; i < value.size()-1; i++) {
        valueString += value.at(i).to_string() + ", ";
    }
    valueString += value.back().to_string() + " ]";

    jsonValue val(name, valueString);
    addJsonValue(val);
}

void Json::write(std::string filename)
{
    std::ofstream jsonfile;
    jsonfile.open(filename);

    jsonfile << "{";
    for(unsigned int i = 0; i < valuse.size()-1; i++)
    {
        jsonfile << '"' + valuse.at(i).name + '"' + ": " + valuse.at(i).value + ", \n";
    }
    jsonfile << '"' + valuse.back().name + '"' + ": " + valuse.back().value;

    jsonfile << "}";

    jsonfile.close();
}

void Json::addJsonValue(jsonValue val)
{
    for (unsigned int i = 0; i < valuse.size(); i++){
        if (val.name == valuse.at(i).name) {
            valuse.at(i) = val;
            return;
        }
    }
    valuse.push_back(val);
}

std::string Json::to_string()
{
    std::string jsonString = "{";
    for(unsigned int i = 0; i < valuse.size()-1; i++)
    {
        jsonString += '"' + valuse.at(i).name + '"' + ": " + valuse.at(i).value + ", \n";
    }
    jsonString += '"' + valuse.back().name + '"' + ": " + valuse.back().value;

    jsonString += "}";
    return jsonString;
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
