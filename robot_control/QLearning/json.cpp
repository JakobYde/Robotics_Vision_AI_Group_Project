#include "json.h"

Json::Json()
{
}


Json::~Json()
{
}
void Json::add(string name, float value)
{
    jsonValue val(name, std::to_string(value));
    addJsonValue(val);
}
void Json::add(string name, int value)
{
    jsonValue val(name, std::to_string(value));
    addJsonValue(val);
}
void Json::add(string name, string value)
{
    jsonValue val(name, '"'+value+'"');
    addJsonValue(val);
}
void Json::add(string name, vector<float> value)
{
    string valueString = "[ ";

    for (int i = 0; i < value.size()-1; i++) {
        valueString += std::to_string(value.at(i)) + ", ";
    }
    valueString += std::to_string(value.back()) + " ]";

    jsonValue val(name, valueString);
    addJsonValue(val);
}
void Json::add(string name, vector<string> value)
{
    string valueString = "[ ";

    for (int i = 0; i < value.size() - 1; i++) {
        valueString += '"' + value.at(i) + '"' + ", ";
    }
    valueString += '"' + value.back() + '"' + " ]";

    jsonValue val(name, valueString);
    addJsonValue(val);
}

void Json::write(string filename)
{
    ofstream jsonfile;
    jsonfile.open(filename);
    jsonfile << to_string();
    jsonfile.close();
}

string Json::to_string()
{
    string jsonString = "{";
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

void makeJOSNPlotData(string titel, string xlabel, string ylabel, vector<float> xdata, vector<float> ydata, string plotType)
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
