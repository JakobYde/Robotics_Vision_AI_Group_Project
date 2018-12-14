#ifndef JSON_H
#define JSON_H
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

void makeJOSNPlotData(std::string titel, std::string xlabel, std::string ylabel, std::vector<float> xdata, std::vector<float> ydata, std::string plotType = "plot");

class Json
{
public:
    Json();
    ~Json();

    void add(std::string name, std::string value);
    template<typename T>
    void add(std::string name, T value);
    template<typename T>
    void add(std::string name, std::vector<T> value);
    void add(std::string name, std::vector<std::string> value);

    void write(std::string filename);
    std::string to_string();
    void csvCreate(std::vector<float> data, std::string filename);



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
