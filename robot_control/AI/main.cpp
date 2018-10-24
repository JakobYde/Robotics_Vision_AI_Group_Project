 #include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>

#include <fl/Headers.h>

#include <array>
#include <iostream>
#include <math.h>

#include "FuzzyBugController.h"
#include "LaserScanner.h"

#define ESC_KEY 27

static boost::mutex mutex;
LaserScanner controllerScan;

void statCallback(ConstWorldStatisticsPtr &_msg) {
  (void)_msg;
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();
  //  std::cout << std::flush;
}

void poseCallback(ConstPosesStampedPtr &_msg) {
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();

  for (int i = 0; i < _msg->pose_size(); i++) {
    if (_msg->pose(i).name() == "pioneer2dx") {

      std::cout << std::setprecision(2) << std::fixed << std::setw(6)
                << _msg->pose(i).position().x() << std::setw(6)
                << _msg->pose(i).position().y() << std::setw(6)
                << _msg->pose(i).position().z() << std::setw(6)
                << _msg->pose(i).orientation().w() << std::setw(6)
                << _msg->pose(i).orientation().x() << std::setw(6)
                << _msg->pose(i).orientation().y() << std::setw(6)
                << _msg->pose(i).orientation().z() << std::endl;
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

float rightStartAngle = angle_min;
float rightEndAngle = angle_step*round((angle_min+2*total_angle_range/5)/angle_step);

float centerStartAngle = rightEndAngle;
float centerEndAngle = angle_step*round((centerStartAngle+total_angle_range/5)/angle_step);

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

int main(int _argc, char **_argv) {
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

  //gazebo::transport::SubscriberPtr poseSubscriber =
  //    node->Subscribe("~/pose/info", poseCallback);

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
    float angleError = 0;
    float goalDisttanse = 10;
    ControlOutput controllerOut = controller.getControlOutput(angleError,goalDisttanse);

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

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
