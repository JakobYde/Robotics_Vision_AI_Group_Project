#ifndef QLEARNING_H
#define QLEARNING_H
#include <vector>

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
    void giveReword(float r);


protected:

    std::vector<state> states;

    float getMaxQ(state* newState);
    float getReword(state* newSate);
    state* policy();//Retunere den nye state som også er den action man tager


    bool actionGiven;//Sikere at getNewState og giveReword køres skiftevis

    float learningRate;
    float stepSize;
    state* currentState;
};

#endif // QLEARNING_H
