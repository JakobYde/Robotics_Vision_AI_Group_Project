#ifndef QLEARNING_H
#define QLEARNING_H
#include <vector>
#include <random>
#include <string>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <locale>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <bitset>
#include "json.h"

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
            qValues = std::vector<std::vector<float>>();
        }
        ~state(){}

        std::string name;
        std::vector<state*> actionStates;
        std::vector<std::vector<float>> qValues;

        float x, y;

        float stddev;
        float mean;
    };

    struct rewardH
    {
        rewardH() {
        }
        rewardH(float rin, std::string stateName) {
            r = rin;
            fromState = stateName;
        }
        float r;
        std::string fromState;
    };

    QLearning(std::string filename, std::string startState, float discount_rate, float stepSize, float greedy, float qInitValue, bool useMultiQ, unsigned int numberOfQValues = 2, bool debug=false);

    state* getNewState();
    void giveReward(float r);
    void print_stats();
    void setState(std::string state);
    std::vector<std::string> getStats();
    std::vector<bool> getVisits();
    bool allVisits();
    void simulateActionReward();
    void wirteJSON(std::string filename);
    float runNormal_distribution(float neam, float stddev);


    float getAvgReward();
    void clearRewardHistroic();
    void clear();
    void clear(std::string startState);
    void runGreedy(bool on = true);
    state* getCurrentStarte();

protected:
    bool useMultiQ;

    std::vector<rewardH> rewardHistroic;
    state * currentStat;
    state * preStat;

    unsigned long long int setBit(unsigned long long  int j, unsigned int i);
    unsigned long long int setBit(unsigned long long  int j, unsigned int i, unsigned int g);
    std::default_random_engine generator;
    std::unordered_map<std::string, int> stateNameIndex;
    std::vector<std::vector<std::vector<std::string>>> stringFromFile(std::string filename);

    std::vector<std::vector<state>> states;


    //[0,0,0,0,0,0]; std::vector<std::vector<state>> states;
    std::string toBits(unsigned int n);

    float getMaxQ(state* newState, unsigned int nQ);
    float getReward(state* newState);
    int policy();//Returnerer det nye state index som også er den action man tager

    void stripWhitespace(std::string& s);
    bool inVec(std::vector<int> vec, int a);
    std::vector<int> getMaxActionIndexs();
    std::vector<int> getMaxActionIndexs(unsigned int nQ);
    int getRandomactionIndex();
    bool debug;
    float discount_rate;
    float stepSize;
    float greedy;
    float ininQValue;
    bool runGreedyBool;

    unsigned int numberOfQValues;
    std::string startState;
    unsigned int nextStateActionIndex;
    //unsigned int currentStateIndex;
    unsigned long long int calIndex(std::vector<bool> visits);

    std::vector<bool> visits;  //the bits indicate whether or not we visitied the state with the corresponding position. false is not visited. true is visited.
};

#endif // QLEARNING_H
