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

#define ESC_KEY 27
#define PI 3.14159265
static boost::mutex mutex;
LaserScanner controllerScan;

struct Position
{
    float x;
    float y;
    Position()
    {
         x = 0.0;
         y = 0.0;
    }
    Position(float xIn,float yIn)
    {
         x = xIn;
         y = yIn;
    }

};

void statCallback(ConstWorldStatisticsPtr &_msg) {
  (void)_msg;
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();
  //  std::cout << std::flush;
}

std::ofstream *myfile;
Position robotPos[2];

std::vector<float> robot_xvalues;
std::vector<float> robot_yvalues;

void poseCallback(ConstPosesStampedPtr &_msg) {
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();

  for (int i = 0; i < _msg->pose_size(); i++) {
    if (_msg->pose(i).name() == "pioneer2dx") {
      //robot_xvalues.push_back(_msg->pose(i).position().x());
      //robot_yvalues.push_back(_msg->pose(i).position().y());

      robotPos[1] = robotPos[0];
      robotPos[0].x = _msg->pose(i).position().x();
      robotPos[0].y = _msg->pose(i).position().y();
      std::stringstream pos_ori_stream;

      pos_ori_stream << std::setprecision(2) << std::fixed << std::setw(6)
                << _msg->pose(i).position().x() << ", " << std::setw(6)
                << _msg->pose(i).position().y() << ", " << std::setw(6)
                << _msg->pose(i).position().z() << ", " << std::setw(6)
                << _msg->pose(i).orientation().w() << ", " << std::setw(6)
                << _msg->pose(i).orientation().x() << ", " << std::setw(6)
                << _msg->pose(i).orientation().y() << ", " << std::setw(6)
                << _msg->pose(i).orientation().z() << std::endl;
      std::string pos_ori_str = pos_ori_stream.str();
      //std::cout << pos_ori_str;
      *myfile << pos_ori_str;
    }
  }
}

void cameraCallback(ConstImageStampedPtr &msg) {

  std::size_t width = msg->image().width();
  std::size_t height = msg->image().height();
  const char *data = msg->image().data().c_str();
  cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

  im = im.clone();
  cv::cvtColor(im, im, CV_BGR2RGB);

  mutex.lock();
  cv::imshow("camera", im);
  mutex.unlock();
}



//Values that define the three different cone sections of the robots lidar scanner. Notice that the robot has roughly 270 degrees of "vision" and the center cone only makes up a tenth of that.
float angle_min = -2.26889;
float angle_max = 2.2689;
float angle_step = 0.0228029648241206;
float total_angle_range = abs(angle_min)+abs(angle_max);
float center_angle_pct = 0.1;

float rightStartAngle = angle_min;
float rightEndAngle = angle_step*round((angle_min+total_angle_range*(1-center_angle_pct)/2)/angle_step);
float centerStartAngle = rightEndAngle;
float centerEndAngle = angle_step*round((centerStartAngle+total_angle_range*center_angle_pct)/angle_step);

float leftStartAngle= centerEndAngle;
float leftEndAngle = angle_max;


void lidarCallbackImg(ConstLaserScanStampedPtr &msg) {
  controllerScan.parseLaserScannerMessage(msg);
  //std::cout << ">> " << msg->DebugString() << std::endl;
  float angle_min = float(msg->scan().angle_min());
  //  double angle_max = msg->scan().angle_max();
  float angle_increment = float(msg->scan().angle_step());

  float range_min = float(msg->scan().range_min());
  float range_max = float(msg->scan().range_max());

  int sec = msg->time().sec();
  int nsec = msg->time().nsec();

  int nranges = msg->scan().ranges_size();
  int nintensities = msg->scan().intensities_size();

  assert(nranges == nintensities);

  int width = 400;
  int height = 400;
  float px_per_m = 200 / range_max;

  cv::Mat im(height, width, CV_8UC3);
  im.setTo(0);
  for (int i = 0; i < nranges; i++) {
    float angle = angle_min + i * angle_increment;
    float range = std::min(float(msg->scan().ranges(i)), range_max);
    //    double intensity = msg->scan().intensities(i);
    cv::Point2f startpt(200.5f + range_min * px_per_m * std::cos(angle),
                        200.5f - range_min * px_per_m * std::sin(angle));
    cv::Point2f endpt(200.5f + range * px_per_m * std::cos(angle),
                      200.5f - range * px_per_m * std::sin(angle));

    cv::Scalar collor = cv::Scalar(255, 255, 255, 255);
    if(((angle >= leftStartAngle - angle_increment )and (angle <= leftStartAngle + angle_increment)) or ((angle >= leftEndAngle - angle_increment) and (angle <= leftEndAngle + angle_increment))) collor = cv::Scalar(255, 0, 0, 255);
    else if(((angle >= centerStartAngle - angle_increment) and (angle <= centerStartAngle + angle_increment)) or ((angle >= centerEndAngle - angle_increment) and (angle <= centerEndAngle + angle_increment))) collor = cv::Scalar(255, 255, 0, 255);
    else if(((angle >= rightStartAngle - angle_increment) and (angle <= rightStartAngle + angle_increment)) or ((angle >= rightEndAngle - angle_increment) and (angle <= rightEndAngle + angle_increment))) collor = cv::Scalar(255, 0, 255, 255);
    cv::line(im, startpt * 16, endpt * 16, collor, 1, cv::LINE_AA, 4);
    //std::cout << angle << " " << leftStartAngle << " " << leftEndAngle << " " << centerStartAngle << " " << centerEndAngle<< " " << rightStartAngle << " " << rightEndAngle << std::endl;

    //    std::cout << angle << " " << range << " " << intensity << std::endl;
  }
  cv::circle(im, cv::Point(200, 200), 2, cv::Scalar(0, 0, 255));
  cv::putText(im, std::to_string(sec) + ":" + std::to_string(nsec),
              cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0,
              cv::Scalar(255, 0, 0));

  mutex.lock();
  cv::imshow("lidar", im);
  mutex.unlock();
}



// A list of functions that determines relationship between two positions on the map. Used in combination with fuzzycontrol to steer toward a specific point in a straight line.
float calDist(Position pos1, Position pos2){
    float xDif = pos2.x - pos1.x;
    float yDif = pos2.y - pos1.y;
    return std::sqrt((std::pow(xDif, 2))+std::pow(yDif, 2));
}

float calDotVec(Position vec1, Position vec2){
    return vec1.x*vec2.x+vec1.y*vec2.y;
}

float calCrossVec(Position vec1, Position vec2){
    return vec1.x*vec2.y-vec1.y*vec2.x;
}

float angleVec(Position vec1, Position vec2){
    return std::atan2(calCrossVec(vec1,vec2),calDotVec(vec1,vec2));
}

float calAngleError(Position *posHist, Position goal){
    Position headingVector;
    headingVector.x = posHist[0].x - posHist[1].x;
    headingVector.y = posHist[0].y - posHist[1].y;

    Position goalVector;
    goalVector.x = goal.x - posHist[1].x;
    goalVector.y = goal.y - posHist[1].y;

    return angleVec(headingVector,goalVector);
}


struct pointManager{
    std::vector<Position> poss;
    unsigned int index;
};


//Function that updates the current goal position of the robot based on a vector of positions. When the robot is close enough to a goal that the distance is smaller than mindist it will update the goal position with the
//next position in the list. When it reaches either end of the list it repeats the list in reverse order.
bool getPointI = true;
Position getpoint(pointManager &pm, Position pos, float mindist){
    Position goal = pm.poss.at(pm.index);

    float dist = calDist(goal,pos);
    if(dist<=mindist)
    {
        if(getPointI) pm.index++;
        else pm.index--;

        if(pm.index==pm.poss.size()-1) getPointI=false;
        else if (pm.index==0) getPointI=true;
    }
    Position newGoal = pm.poss.at(pm.index);
    return newGoal;
}

bool atState(Position goal, Position pos, float mindist){
    float dist = calDist(goal,pos);
    if(dist<=mindist) return true;
    return false;
}

/*
int main(int _argc, char **_argv) {
    //Zero start pos
    robotPos[0].x = 0.0;
    robotPos[0].y = 0.0;
    robotPos[1].x = 0.0;
    robotPos[1].y = 0.0;

    //Lav pos log
    myfile = new std::ofstream("pos.log");

    //Lav controller

    FuzzyBugController controller( & controllerScan);
    controller.buildController();

    // Load gazebo
    gazebo::client::setup(_argc, _argv);

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Listen to Gazebo topics
    gazebo::transport::SubscriberPtr statSubscriber =
      node->Subscribe("~/world_stats", statCallback);

    gazebo::transport::SubscriberPtr poseSubscriber =
      node->Subscribe("~/pose/info", poseCallback);

    gazebo::transport::SubscriberPtr cameraSubscriber =
      node->Subscribe("~/pioneer2dx/camera/link/camera/image", cameraCallback);

    gazebo::transport::SubscriberPtr lidarSubscriber =
      node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", lidarCallbackImg);

    // Publish to the robot velkey_esc_cmd topic
    gazebo::transport::PublisherPtr movementPublisher =
      node->Advertise<gazebo::msgs::Pose>("~/pioneer2dx/vel_cmd");

    // Publish a reset of the world
    gazebo::transport::PublisherPtr worldPublisher =
      node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
    gazebo::msgs::WorldControl controlMessage;
    controlMessage.mutable_reset()->set_all(true);
    worldPublisher->WaitForConnection();
    worldPublisher->Publish(controlMessage);


    enum statsStateMachine {onTheWay, atState_};

    statsStateMachine statemc = onTheWay;

    //filename, startState, discount_rate, stepSize, greedy, qInitValue
    QLearning q("../QLearning/stats.txt","S0",0.7,0.4,0.0,0.0, true);
    QLearning::state* currentstate = q.getNewState();
    q.print_stats();
    Position goal = Position(currentstate->x,currentstate->y);

    ControlOutput controllerOut;

    // Loop
    int run = 0;
    while (true) {
        gazebo::common::Time::MSleep(10);



        switch (statemc) {
            case onTheWay:
                if(atState(goal,robotPos[0],0.5)) statemc = atState_;

                controllerOut = controller.getControlOutput(calAngleError(robotPos,goal),calDist(robotPos[0],goal), center_angle_pct);

                break;

            case atState_:
                controllerOut.direction = 0;
                controllerOut.speed = 0;

                std::cout << "Run : " << run++ << " at state: " << currentstate->name << std::endl;
                q.giveReward(q.runNormal_distribution(currentstate->mean,currentstate->stddev));

                currentstate = q.getNewState();

                goal = Position(currentstate->x,currentstate->y);
                statemc = onTheWay;
                q.print_stats();

                break;
        }

        //Få control signal





        mutex.lock();
        int key = cv::waitKey(1);
        mutex.unlock();

        if (key == ESC_KEY)
          break;

        // Generate a pose
        ignition::math::Pose3d pose(controllerOut.speed, 0, 0, 0, 0, controllerOut.direction);

        // Convert to a pose message
        gazebo::msgs::Pose msg;
        gazebo::msgs::Set(&msg, pose);
        movementPublisher->Publish(msg);

    }
    q.wirteJSON("stats.json");
    myfile->close();
    delete myfile;
    //makeJOSNPlotData("Robot parth","x","y",robot_xvalues,robot_yvalues);
    // Make sure to shut everything down.
    gazebo::client::shutdown();
}
*/

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

std::string getProcessbar(int n, int maxn, int len = 10){
    char rn[8] = {'|', '/', '-', '\\', '|', '/', '-', '\\'};
    static int c = 0;

    std::string bar = "[";

    float pro = (n)/float(maxn)*100;
    int barNumber = std::floor(pro/100*len);

    for(int i = 0; i < barNumber; i++) bar += '#';

    if(barNumber < len) bar += rn[c++%8];

    for(int i = barNumber+1; i < len; i++) bar += ' ';
    bar += "]";

    bar += fts(std::roundf(pro * 100) / 100,2) + "%";

    return bar;
}

//initialization data for QLearning
struct qTestPra
{
    qTestPra() {}
    int epsiodes;
    int maxStepsInEpsiode;
    int avgOver;
    float mvAvgAlfa;
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

//Central function that runs the learning iterations. Given a configuration it uses QLearning on the model, and saves the result.
data testQ(QLearning &q, int epsiodes = 2000, int maxStepsInEpsiode = 20, int avgOver = 10, float mvAvgAlfa = 0.01, bool randomStartState = true, std::string startState = "S0", bool print = false, std::string preSet = ""){
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
    }
    dataset.ydata = getMovingAvg(getAvg(ydata,1),mvAvgAlfa);

    return dataset;
}


//Too shorten learning time multiple threads are used. Critical parts of the code are protected with mutex locks.
struct workerParameter
{
    workerParameter() {}
    std::queue<qTestPra> *qqueue;
    std::mutex *mux_qqueue;
    std::queue<data> *dataqueue;
    std::mutex *mux_dataqueue;
};

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

        data testdata = testQ(q,qp.epsiodes,qp.maxStepsInEpsiode,qp.avgOver,qp.mvAvgAlfa, qp.randomStartState, qp.startState, false,"");
        testdata.prameters = qp;

        wp.mux_dataqueue->lock();
        wp.dataqueue->push(testdata);
        wp.mux_dataqueue->unlock();
        boost::this_thread::sleep( boost::posix_time::microseconds(10));
    }
}

int main()
{
    const int thredsN = 5;

    qTestPra ground;
    ground.epsiodes = 100000;
    ground.maxStepsInEpsiode = 5;
    ground.avgOver = 100;
    ground.mvAvgAlfa = 0.005;

    ground.useDoubelQ = true;
    ground.numberOfQValues = 2;
    ground.filename = "../QLearning/stats.txt";
    ground.startState = "S0";
    ground.randomStartState = true;
    ground.discount_rate = 0.75;
    ground.stepSize = 0.2;
    ground.greedy = 0.05;
    ground.qInitValue = 10;

    JSONPlot j("Q-learning. Discount_rate: "+fts(ground.discount_rate,3) +", stepSize: "+fts(ground.stepSize,3)+", greedy: "+fts(ground.greedy,3)+", qInitValue: "+fts(ground.qInitValue,3) , "Steps", "movingAvg reward (alfa = "+fts(ground.mvAvgAlfa,3)+")");

    std::vector<int> testVar= {1, 2,3,4};
    //std::vector<float> testVar= {0.0, 0.001, 0.005, 0.01, 0.05, 0.2, 0.8};
    //for(float var = 0.0; var <= 1.0; var+=0.05) testVar.push_back(var);


    std::queue<data> dataqueue;
    std::mutex mux_dataqueue;
    std::queue<qTestPra> qqueue;
    std::mutex mux_qqueue;

    for(unsigned int i = 0; i < testVar.size(); i++){
        qTestPra test = ground;
        test.numberOfQValues = testVar.at(i);

        qqueue.push(test);
    }

    std::thread threads[thredsN];
    workerParameter wp;
    wp.dataqueue = &dataqueue;
    wp.mux_dataqueue = &mux_dataqueue;
    wp.qqueue = &qqueue;
    wp.mux_qqueue = &mux_qqueue;

    for(int i = 0; i < thredsN; i++) threads[i] = std::thread(worker,wp);

    while(dataqueue.size() != testVar.size()){
        printf("\033c");
        std::cout << "Running test: " << getProcessbar(dataqueue.size(),testVar.size(),testVar.size()) << std::endl;

        boost::this_thread::sleep( boost::posix_time::seconds(1) );

    }

    for(unsigned int i = 0; i < thredsN; i++) threads[i].join();

    int dataSice = dataqueue.size();
    while(not dataqueue.empty()){
        printf("\033c");
        std::cout << "Wirting JSON: " << getProcessbar(dataSice-dataqueue.size(),dataSice,dataSice) << std::endl;

        data dt = dataqueue.front();
        dataqueue.pop();

        j.addData("NumberOfQValues: "+std::to_string(dt.prameters.numberOfQValues),dt.xdata,dt.ydata);//fts(dt.prameters.greedy,5),dt.xdata,dt.ydata);
    }
    j.write();

    return 0;
}
