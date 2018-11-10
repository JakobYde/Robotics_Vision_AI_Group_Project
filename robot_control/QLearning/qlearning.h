#ifndef QLEARNING_H
#define QLEARNING_H
#include <vector>
#include <random>
#include <string>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <locale>
#include <random>
#define STATE_NAME_INDEX 0
#define STATE_X_INDEX 1
#define STATE_Y_INDEX 2
#define STATE_CONN_INDEX 3
#define STATE_MEAN_INDEX 4
#define STATE_STDDIV_INDEX 5

class QLearning
{
public:
    struct state
    {
        state() {
            name = "No name";
            actionStates = std::vector<state*>();
            qValues = std::vector<float>();
        }
        ~state(){}

        std::string name;
        std::vector<state*> actionStates;
        std::vector<float> qValues;

        float x, y;

        float stddev;
        float mean;
    };

    QLearning(std::string filename, float qInitValue);
    state* getNewState();
    void giveReward(float r);

    void print_stats();
protected:
    std::default_random_engine generator;

    std::vector<std::vector<std::vector<std::string>>> stringFromFile(std::string filename);
    std::vector<state> states;

public://Midlatidig offlig
    float runNormal_distribution(float neam, float stddev);
protected:
    float getMaxQ(state* newState);
    float getReward(state* newState);
    state* policy();//Returnerer den nye state som også er den action man tager

    void stripWhitespace(std::string& s);

    bool actionGiven;//Sikrer at getNewState og giveReward køres skiftevis

    float learningRate;
    float stepSize;
    state* currentState;
};

#endif // QLEARNING_H
