#ifndef JSON_H
#define JSON_H
#include <string>
#include <vector>
#include <fstream>
using namespace std;

void makeJOSNPlotData(string titel, string xlabel, string ylabel, vector<float> xdata, vector<float> ydata, string plotType = "plot");

class Json
{
public:
    Json();
    ~Json();

    void add(string name, string value);
    void add(string name, vector<float> value);
    void add(string name, vector<string> value);
    void add(string name, int value);
    void add(string name, float value);
    void add(string name,  vector<Json> value);
    void add(string name, Json value);
    void write(string filename);
    string to_string();

protected:

    struct jsonValue
    {
        string name;
        string value;

        jsonValue(string namein, string valuein) {
            name = namein;
            value = valuein;
        }
    };
    void addJsonValue(jsonValue val);
    vector<jsonValue> valuse;
};

#endif // JSON_H
