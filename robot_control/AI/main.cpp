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

#include "FuzzyBugController.h"
#include "LaserScanner.h"

#define ESC_KEY 27
#define PI 3.14159265
static boost::mutex mutex;
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

void poseCallback(ConstPosesStampedPtr &_msg) {
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();

  for (int i = 0; i < _msg->pose_size(); i++) {
    if (_msg->pose(i).name() == "pioneer2dx") {
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

int main(int _argc, char **_argv) {
    //Zero start pos
    robotPos[0].x = 0.0;
    robotPos[0].y = 0.0;
    robotPos[1].x = 0.0;
    robotPos[1].y = 0.0;

    pointManger pm;
    pm.index = 0;


    Possison goal0(0,0);
    Possison goal1(23.2,4);
    Possison goal2(29.5,20.7);
    Possison goal3(37,6);
    Possison goal4(34.1,-20.6);
    Possison goal5(12,-19.5);
    Possison goal6(3.9,-24.5);
    Possison goal7(-6.9,-21);
    Possison goal8(3.9,-24.5);
    Possison goal9(12,-19.5);
    Possison goal10(34.1,-20.6);
    Possison goal11(29.8,-12.2);
    Possison goal12(26.9,3.9);
    Possison goal13(-37.2,-0.42);

    pm.poss.push_back(goal0);
    pm.poss.push_back(goal1);
    pm.poss.push_back(goal2);
    pm.poss.push_back(goal3);
    pm.poss.push_back(goal4);
    pm.poss.push_back(goal5);
    pm.poss.push_back(goal6);
    pm.poss.push_back(goal7);
    pm.poss.push_back(goal8);
    pm.poss.push_back(goal9);
    pm.poss.push_back(goal10);
    pm.poss.push_back(goal11);
    pm.poss.push_back(goal12);
    pm.poss.push_back(goal13);

    //Lav pos log
    myfile = new std::ofstream("pos.log");

    //Lav controller

    FuzzyBugController controller( & controllerScan, leftStartAngle, leftEndAngle, rightStartAngle, rightEndAngle, centerStartAngle, centerEndAngle);
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

    // Loop
    while (true) {
        gazebo::common::Time::MSleep(10);

        //Få control signal
        Possison goal = getpoint(pm,robotPos[0], 1.0);

        float angleError = calAngleError(robotPos,goal);
        float goalDistance = calDist(robotPos[0],goal);
        std::cout << std::setprecision(3) << std::fixed << "Angle error: " << angleError << ", " << std::setw(6) << "goalDistance: " << goalDistance << " ::: " << std::setw(6) << "Goal: " << goal.x << ", " << goal.y << ", " << std::setw(6) << "Pos: " << robotPos[0].x << ", " << robotPos[0].y << std::endl;
        ControlOutput controllerOut = controller.getControlOutput(angleError,goalDistance);

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

    myfile->close();
    delete myfile;
    // Make sure to shut everything down.
    gazebo::client::shutdown();
}
