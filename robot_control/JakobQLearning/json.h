#ifndef JSON_H
#define JSON_H
#include <string>
#include <vector>
#include <fstream>

void makeJOSNPlotData(std::string titel, std::string xlabel, std::string ylabel, std::vector<float> xdata, std::vector<float> ydata, std::string plotType = "plot");

class Json
{
public:
    Json();
    ~Json();

    void add(std::string name, std::string value);
    void add(std::string name, std::vector<float> value);
    void add(std::string name, std::vector<std::string> value);
    void add(std::string name, int value);
    void add(std::string name, float value);
    void add(std::string name, std::vector<Json> value);
    void add(std::string name, Json value);
    void write(std::string filename);
    std::string to_string();

protected:

    struct jsonValue
    {
        std::string name;
        std::string value;

        jsonValue(std::string namein, std::string valuein) {
            name = namein;
            value = valuein;
        }
    };
    void addJsonValue(jsonValue val);
    std::vector<jsonValue> valuse;
};

#endif // JSON_H
