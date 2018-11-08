#ifndef QLEARNING_H
#define QLEARNING_H
#include <vector>
#include <random>

class QLearning
{
public:
    struct state
    {
        state() {}

        std::vector<state*> actionStates;
        std::vector<float> qValues;

        float x, y;

        float stdDiv;
        float mean;
    };
    QLearning();
    state* getNewState();
    void giveReward(float r);


protected:

    std::vector<state> states;

    float getMaxQ(state* newState);
    float getReward(state* newState);
    state* policy();//Returnerer den nye state som også er den action man tager


    bool actionGiven;//Sikrer at getNewState og giveReward køres skiftevis

    float learningRate;
    float stepSize;
    state* currentState;
};

#endif // QLEARNING_H
