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

#include "FuzzyBugController.h"
#include "LaserScanner.h"
#include "json.h"

#define ESC_KEY 27
#define PI 3.14159265
static boost::mutex mutex;
static boost::mutex mutex2;
LaserScanner controllerScan;

struct Possison{
    float x;
    float y;
    Possison() {
         x = 0.0;
         y = 0.0;
    }
    Possison(float xIn,float yIn) {
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
Possison robotPos[2];

std::vector<float> robot_xvalues;
std::vector<float> robot_yvalues;

double angle = 0;

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

      angle = atan2(_msg->pose(i).orientation().w(), _msg->pose(i).orientation().z()) * 180 / PI * 2;
      if (angle < 0) angle += 360;
      //std::cout << angle << std::endl;
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

bool ballCollected = false;
int distanceToCenter = 200;
int bluePercentage = 0;
int biggestCircle = 0;
int distanceToBall = 0;

void cameraCallback(ConstImageStampedPtr &msg
                    ) {

  std::size_t width = msg->image().width();
  std::size_t height = msg->image().height();
  const char *data = msg->image().data().c_str();
  cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

  im = im.clone();
  cv::cvtColor(im, im, CV_RGB2BGR);
  cv::Mat hls;
  cv::cvtColor(im, hls, CV_BGR2HLS);
  cv::Mat hlsch[3];
  cv::split(hls, hlsch);
  cv::Mat binary;

  cv::threshold(hlsch[2], binary, 50, 255, cv::THRESH_BINARY);

  GaussianBlur( binary, binary, cv::Size(9, 9), 2, 0);

  std::vector<cv::Vec3f> circles;
  cv::HoughCircles(binary, circles, cv::HOUGH_GRADIENT,1.2,1,100,60, 10);

  if (circles.size() != 0) {
      for (int i = circles.size() - 1; i > 0; i--)
      {
          for (int j = 1; j < i + 1; j++)
          {
              if (sqrt(pow(circles[i][0]-circles[i - j][0],2)+pow(circles[i][1]-circles[i - j][1],2)) < circles[i - j][2]) {
                  circles.erase(circles.begin() + i);
                  break;
              }
          }
      }
  }

  int circleIndex = 0;
  biggestCircle = 0;
  for (size_t i = 0; i < circles.size(); i++)
  {
      cv::Vec3i c = circles[i];
      cv::Point center = cv::Point(c[0], c[1]);
      // circle center
      circle(im, center, 1, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
      // circle outline
      int radius = c[2];
      circle(im, center, radius, cv::Scalar(255, 0, 255), 1, cv::LINE_AA);
      if (radius > biggestCircle) {
          biggestCircle = radius;
          circleIndex = i;
      }
  }
  if (biggestCircle > 5) {
      distanceToCenter = circles[circleIndex][0] - (hlsch[2].cols/2);
      //Distance to ball is found using the camera FOV, the radius of the ball and simple geometry.
      distanceToBall = biggestCircle / tan(60/2) / 2;
  }
  else distanceToBall++;
  std::cout << distanceToBall << std::endl;
  //Finds blue pixels in image, in case no circles are detected.
  mutex2.lock();
  bluePercentage = 0;
      for (cv::MatIterator_<cv::Vec3b> p = hls.begin<cv::Vec3b>(); p != hls.end<cv::Vec3b>(); p++)
      {
          if ((*p)[2] > 100)
              bluePercentage++;
      }
      bluePercentage = bluePercentage * 100.0 / 72000.0;
  //std::cout << "Amount of blue: " << bluePercentage << "%" << std::endl;
  mutex2.unlock();

  mutex.lock();
  cv::imshow("camera", im);
  mutex.unlock();
}

Possison ballCoordinates(double angle, double distance){
    Possison temp;
    temp.x = distance * cos(angle) + robotPos[0].x;
    temp.y = distance * sin(angle) + robotPos[0].y;
    return temp;
}

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

float calDist(Possison pos1, Possison pos2){
    float xDif = pos2.x - pos1.x;
    float yDif = pos2.y - pos1.y;
    return std::sqrt((std::pow(xDif, 2))+std::pow(yDif, 2));
}

float calDotVec(Possison vec1, Possison vec2){
    return vec1.x*vec2.x+vec1.y*vec2.y;
}

float calCrossVec(Possison vec1, Possison vec2){
    return vec1.x*vec2.y-vec1.y*vec2.x;
}

float angleVec(Possison vec1, Possison vec2){
    return std::atan2(calCrossVec(vec1,vec2),calDotVec(vec1,vec2));
}

float calAngleError(Possison *posHist, Possison goal){
    Possison headingVector;
    headingVector.x = posHist[0].x - posHist[1].x;
    headingVector.y = posHist[0].y - posHist[1].y;

    Possison goalVector;
    goalVector.x = goal.x - posHist[1].x;
    goalVector.y = goal.y - posHist[1].y;

    return angleVec(headingVector,goalVector);
}


struct pointManger{
    std::vector<Possison> poss;
    int index;
};

bool getPointI = true;
Possison getpoint(pointManger &pm, Possison pos, float mindist){
    Possison goal = pm.poss.at(pm.index);

    float dist = calDist(goal,pos);
    if(dist<=mindist){
        if(getPointI) pm.index++;
        else pm.index--;

        if(pm.index==pm.poss.size()-1) getPointI=false;
        else if (pm.index==0) getPointI=true;
    }
    Possison newGoal = pm.poss.at(pm.index);
    return newGoal;
}

bool atState(Possison goal, Possison pos, float mindist){
    float dist = calDist(goal,pos);
    if(dist<=mindist) return true;
    return false;
}

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

    //filename, startState, discount_rate, stepSize, greedy, qInitValue
    QLearning q("../QLearning/stats.txt","S0",0.7,0.4,0.0,0.0, true);
    enum statsStateMachine {onTheWay, atState_, findMarbles, returnToPoint, pickupMarble};

    statsStateMachine statemc = onTheWay;
    QLearning::state* currentstate = q.getNewState();
    Possison goal = Possison(currentstate->x,currentstate->y);
    ControlOutput controllerOut;
    struct ControlOutput
    {
        float direction;
        float speed;
    };
    // Loop
    int run = 0;

    int rot = 999;
    std::vector<int> pos = {0,0};
    int lastImBlue = 0;

    float angleError;
    float goalDistance;

    bool hasTurned = false;

    while (true) {
        gazebo::common::Time::MSleep(10);

        switch (statemc) {
            case onTheWay:
            {
                if(atState(goal,robotPos[0],0.5)) statemc = atState_;
                angleError = calAngleError(robotPos,goal);
                goalDistance = calDist(robotPos[0],goal);
                controllerOut = controller.getControlOutput(angleError,goalDistance, center_angle_pct);
                break;
            }

            case atState_:
            {
                //std::cout << "Run : " << run++ << " at state: " << currentstate->name << std::endl;
                float r = q.runNormal_distribution(currentstate->mean,currentstate->stddev);
                q.giveReward(r);
                currentstate = q.getNewState();
                goal = Possison(currentstate->x,currentstate->y);
                //statemc = onTheWay;
                statemc = findMarbles;
                q.print_stats();
                controllerOut.direction = 0.0;
                controllerOut.speed = 0.0;
                break;
            }

            case returnToPoint:
            {
                angleError = calAngleError(robotPos, goal);
                goalDistance = calDist(robotPos[0], goal);
                controllerOut = controller.getControlOutput(angleError,goalDistance, center_angle_pct);

                if (abs(goal.x - robotPos[0].x) == 0 && abs(goal.x - robotPos[0].y) == 0) {
                        controllerOut.speed = 0;
                        rot = angle;
                        statemc = findMarbles;
                    }
                }

            case findMarbles:
            {
                mutex2.lock();
                int bluetemp = bluePercentage;
                mutex2.unlock();
                if ((distanceToCenter == 0) || (bluetemp > 90)) {
                   goal = Possison(robotPos[0].x, robotPos[0].y);
                   hasTurned = false;

                   Possison ballPosition;
                   ballPosition = Possison(robotPos[0].x + distanceToBall * cos(angle), robotPos[0].y + distanceToBall * sin(angle));
                   angleError = calAngleError(robotPos, ballPosition);
                   goalDistance = calDist(robotPos[0], ballPosition);
                   controllerOut = controller.getControlOutput(angleError,goalDistance, center_angle_pct);

                   statemc = pickupMarble;
                }
                else {
                    if (abs(rot - angle) > 20) hasTurned = true;
                    if (abs(rot - angle) < 10 && hasTurned) {
                        hasTurned = false;
                        //goal = Possison(currentstate->x,currentstate->y);
                        statemc = onTheWay;
                    }
                    if ((double)distanceToCenter / 200 > 0.5) controllerOut.direction = 0.5;
                    else controllerOut.direction = (double)distanceToCenter / 200.0;
                    controllerOut.speed = 0.0;
                }
                break;
            }
            case pickupMarble:
            {
                if (biggestCircle > 50) {
                    controllerOut.speed = 1;
                    if (distanceToCenter == 0) controllerOut.direction = 0;
                    else if (distanceToCenter > 0) controllerOut.direction = 0.01;
                    else if (distanceToCenter < 0) controllerOut.direction = -0.01;
                }
                mutex2.lock();
                if (lastImBlue - bluePercentage > 50)
                {
                    statemc = returnToPoint;
                }
                lastImBlue = bluePercentage;
                mutex2.unlock();
                break;
            }
        }

        //Bredde = 0.2770;

        //Få control signal


        //std::cout << std::setprecision(3) << std::fixed << "Angle error: " << angleError << ", " << std::setw(6) << "goalDistance: " << goalDistance << " ::: " << std::setw(6) << "Goal: " << goal.x << ", " << goal.y << ", " << std::setw(6) << "Pos: " << robotPos[0].x << ", " << robotPos[0].y << std::endl;


        //FL_LOG("SenM" << Op::str(senM)<<" : "<< Op::str(sM->getValue())<< " dif: " << Op::str(senM-sM->getValue())
        //       << "; Speed.output = " << Op::str(speed->getValue()));

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


/*
int main()
{
    //std::string filename, float learningRate, float stepSize, float greedy, float qInitValue
    QLearning q("stats.txt","S0",0.7,0.4,0.7,1.0);
    std::cout << "Start: " << std::endl;
    q.print_stats();

    for(int i=0;i<5;i++){
        std::cout << "Run: " << i << std::endl;
        q.simulateActionReward();
        q.print_stats();
    }
    std::cout << "Run 5 to 98: " << std::endl;
    for(int i=5;i<100;i++){
        q.simulateActionReward();
    }
    std::cout << "Run: 99" << std::endl;
    q.print_stats();
    q.wirteJSON("statsjson.json");
    return 0;
}
*/
