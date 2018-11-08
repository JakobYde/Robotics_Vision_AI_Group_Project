#include "qlearning.h"

QLearning::QLearning()
{

}



float QLearning::getMaxQ(state* newstate)  //Returns the higest value in the qValues vector for a given state.
{
    float max_q =newstate->qValues[0];
    for(int i= 1; i < newstate->qValues.size ; i++)
    {
        if(newstate->qValues[i]>max_q)
        {
            max_q = newstate->qValues[i];
        }
    }
    return max_q;
}


float QLearning::getReward(state* newstate) //should return reward of given state.
{
    normal_distribution<float> distribution(newstate->mean,newstate->stdDiv);

    return 0
}
