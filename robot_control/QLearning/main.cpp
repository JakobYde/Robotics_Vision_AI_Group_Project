#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "qlearning.h"
#include "jsonplot.h"
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>

#include <fl/Headers.h>

#include <array>
#include <iostream>
#include <math.h>
#include <sstream>      // std::stringstream
#include <fstream>
#include <vector>
#include <thread>
#include <mutex>
#include <queue>
#include <boost/thread/thread.hpp>

#include "FuzzyBugController.h"
#include "LaserScanner.h"
#include "json.h"
#include "jsonplot.h"


std::string getRandomState(QLearning & q){
    std::vector<std::string> stats = q.getStats();
    return stats.at(rand()%stats.size());
}


//Gives average of elements between vectors. Used to estimate the reward of an episode after learning in a given amount of episodes
std::vector<float> getAvg(std::vector <std::vector<float>> vec, int axle = 0) {
    std::vector<float> sum;
    if (axle == 0){
        for (size_t i = 0; i < vec.size(); i++) {
            sum.push_back(0.0);
            for (size_t j = 0; j < vec.at(i).size(); j++)
            {
                sum.at(i) += vec.at(i).at(j);
            }
            sum.at(i) /= vec.at(i).size();
        }
    }
    else{
        for (size_t i = 0; i < vec.size(); i++) {
            for (size_t j = 0; j < vec.at(i).size(); j++)
            {
                if(i == 0) sum.push_back(0.0);
                sum.at(j) += vec.at(i).at(j);
            }
        }
        for(size_t i = 0; i < sum.size(); i++) sum.at(i)/=vec.size();
    }
    return sum;
}

std::vector<float> getMovingAvg(std::vector<float> vec, float alfa = 0.6) {
    std::vector<float> avg;
    float s = vec.at(0);
    avg.push_back(s);

    for(unsigned int i = 1; i < vec.size(); i++){
        s = alfa*vec.at(i) + (1-alfa)*s;
        avg.push_back(s);
    }
    return avg;
}

std::string fts(float f, int dec){
    std::string floatstring = std::to_string(f);
    std::string res = "";
    unsigned int i;
    for(i = 0; i < floatstring.length(); i++) {
        if(floatstring[i] != '.') res += floatstring[i];
        else break;
    }
    for(unsigned int j = i+dec; i <= j and i < floatstring.length(); i++) {
        res += floatstring[i];
    }
    return res;
}

std::string getProcessbar(long long unsigned int n, long long unsigned int maxn, long long unsigned int len = 10){
    const char rn[8] = {'|', '/', '-', '\\', '|', '/', '-', '\\'};
    static long long unsigned int c = 0;

    std::string bar = "[";

    float pro = (n)/float(maxn)*100;
    unsigned int barNumber = std::floor(pro/100.0*len);

    for(unsigned int i = 0; i < barNumber; i++) bar += '#';

    if(barNumber < len) bar += rn[c++%8];

    for(unsigned int i = barNumber+1; i < len; i++) bar += ' ';
    bar += "]";

    bar += fts(std::roundf(pro * 100) / 100,2) + "%";
    bar += ". " + std::to_string(n) + " / " + std::to_string(maxn);
    return bar;
}

//initialization data for QLearning
struct qTestPra
{
    qTestPra() {}
    int epsiodes;
    int maxStepsInEpsiode;
    int avgOver;
    bool useDoubelQ;
    unsigned int numberOfQValues;
    bool randomStartState;

    std::string filename;
    std::string startState;
    float discount_rate;
    float stepSize;
    float greedy;
    float qInitValue;
};

struct data{
    std::vector<int> xdata;
    std::vector<float> ydata;
    qTestPra prameters;
};

//Too shorten learning time multiple threads are used. Critical parts of the code are protected with mutex locks.
struct workerParameter
{
    workerParameter() {}
    std::queue<qTestPra> *qqueue;
    std::mutex *mux_qqueue;
    std::queue<data> *dataqueue;
    std::mutex *mux_dataqueue;
    long long unsigned int *count;
    std::mutex *mux_count;
};

//Central function that runs the learning iterations. Given a configuration it uses QLearning on the model, and saves the result.
data testQ(QLearning &q, workerParameter &wp, int epsiodes = 2000, int maxStepsInEpsiode = 20, int avgOver = 10, bool randomStartState = true, std::string startState = "S0", bool print = false, std::string preSet = ""){
    data dataset;
    std::vector<std::vector<float>> ydata;
    for(int k = 0; k < avgOver; k++){
        q.clear();//"S0"; getRandomState(q)
        ydata.push_back(std::vector<float>());

        for(int i = 0; i < epsiodes; i++){
            if(randomStartState) q.setState(getRandomState(q));//getRandomState(q)
            else q.setState(startState); //"S0"

            if(print) printf("\033c");
            if(print) std::cout << preSet << "Epsiode " << i+1 << "/" << epsiodes << " --- " << getProcessbar(i, epsiodes, 30);

            if(k==0) dataset.xdata.push_back(i);

            int step = 0;

            while(not q.allVisits() and step < maxStepsInEpsiode){
                q.simulateActionReward();
                step++;
            }
            float avgR = q.getAvgReward();
            //std::cout << avgR << std::endl;
            ydata.at(k).push_back(avgR);
            q.clearRewardHistroic();
        }
        wp.mux_count->lock();
        *wp.count = *wp.count+1;
        wp.mux_count->unlock();
    }
    //dataset.ydata = getMovingAvg(getAvg(ydata,1),mvAvgAlfa);
    dataset.ydata = getAvg(ydata,1);

    return dataset;
}


void worker(workerParameter wp){
    qTestPra qp;
    while(true){
        wp.mux_qqueue->lock();
        if(wp.qqueue->empty()){
            wp.mux_qqueue->unlock();
            break;
        }
        qp = wp.qqueue->front();
        wp.qqueue->pop();
        wp.mux_qqueue->unlock();

        QLearning q(qp.filename,qp.startState,qp.discount_rate,qp.stepSize,qp.greedy,qp.qInitValue, qp.useDoubelQ, qp.numberOfQValues);

        data testdata = testQ(q,wp,qp.epsiodes,qp.maxStepsInEpsiode,qp.avgOver, qp.randomStartState, qp.startState, false,"");
        testdata.prameters = qp;

        wp.mux_dataqueue->lock();
        wp.dataqueue->push(testdata);
        wp.mux_dataqueue->unlock();
        boost::this_thread::sleep( boost::posix_time::microseconds(10));
    }
}

int main()
{
    const int thredsN = 9;

    qTestPra ground;
    ground.epsiodes = 200000;
    ground.maxStepsInEpsiode = 5;
    ground.avgOver = 100;

    ground.useDoubelQ = true;
    ground.numberOfQValues = 2;
    ground.filename = "../QLearning/stats.txt";
    ground.startState = "S0";
    ground.randomStartState = true;
    ground.discount_rate = 0.75;
    ground.stepSize = 0.2;
    ground.greedy = 0.05;
    ground.qInitValue = 0;

    JSONPlot j("Q-learning. Discount_rate: "+fts(ground.discount_rate,3) +", stepSize: "+fts(ground.stepSize,3)+", greedy: test"/*+fts(ground.greedy,3)*/+", qInitValue: "+fts(ground.qInitValue,3) , "Episode", "Avg reward over "+std::to_string(ground.avgOver)+" repetitions");

    //std::vector<int> testVar= {1, 2,3,4};
    //0.1 <- 0.001

    const float maxTest = 0.1;
    const float minTest = 0.001;
    const int numberOfTests = 22;//Will be +1
    const float inc = (maxTest-minTest)/numberOfTests;
    std::vector<float> testVar;
    for(float testV = minTest; testV <= maxTest+inc; testV += inc) testVar.push_back(testV);

    //std::vector<float> testVar= {-1.0, 0.0, 0.001, 0.005, 0.01, 0.05, 0.2, 0.8, 1.0};

    std::queue<data> dataqueue;
    std::mutex mux_dataqueue;
    std::queue<qTestPra> qqueue;
    std::mutex mux_qqueue;

    for(unsigned int i = 0; i < testVar.size(); i++){
        qTestPra test = ground;
        test.greedy = testVar.at(i);

        qqueue.push(test);
    }

    long long unsigned int count = 0;
    std::mutex mux_count;

    std::thread threads[thredsN];
    workerParameter wp;
    wp.dataqueue = &dataqueue;
    wp.mux_dataqueue = &mux_dataqueue;
    wp.qqueue = &qqueue;
    wp.mux_qqueue = &mux_qqueue;
    wp.count = &count;
    wp.mux_count = &mux_count;

    for(int i = 0; i < thredsN; i++) threads[i] = std::thread(worker,wp);

    while(dataqueue.size() != testVar.size()){
        printf("\033c");
        std::cout << "Running test: " << getProcessbar(count,ground.avgOver*testVar.size(),20) << std::endl;

        boost::this_thread::sleep( boost::posix_time::seconds(1) );

    }

    for(unsigned int i = 0; i < thredsN; i++) threads[i].join();

    int dataSice = dataqueue.size();
    while(not dataqueue.empty()){
        printf("\033c");
        std::cout << "Wirting JSON: " << getProcessbar(dataSice-dataqueue.size(),dataSice,dataSice) << std::endl;

        data dt = dataqueue.front();
        dataqueue.pop();

        j.addData("Greedy: "+std::to_string(dt.prameters.greedy),dt.xdata,dt.ydata);//fts(dt.prameters.greedy,5),dt.xdata,dt.ydata);
    }
    j.write();

    return 0;
}
