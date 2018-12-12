#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>

#include <array>
#include <iostream>
#include <math.h>
#include <sstream>      // std::stringstream
#include <fstream>
#include <vector>

#include "Point.h"
#include "Lidar.h"

#define ESC_KEY 27

void statCallback(ConstWorldStatisticsPtr &_msg) {
  (void)_msg;
}

void poseCallback(ConstPosesStampedPtr &_msg) {

}

void cameraCallback(ConstImageStampedPtr &msg) {

}

Lidar l;

void lidarCallbackImg(ConstLaserScanStampedPtr &msg) {
    std::vector<PolarPoint> polarPoints;
    const gazebo::msgs::LaserScan* scan = &(msg->scan());
    double maxRange = scan->range_max();
    double angleMin = scan->angle_min();
    double angleInc = scan->angle_step();
    int nRanges = scan->ranges_size();

    for (int i = 0; i < nRanges; i++) {
        if (scan->ranges(i) < maxRange) {
            double angle = angleMin + i * angleInc;
            double range = scan->ranges(i);
            l.setRadius(range);
            polarPoints.push_back(PolarPoint(range, angle));
        }
    }

    l.addMeasurements(polarPoints);
    l.drawProcess();
}

int main(int _argc, char **_argv) {
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

        int key = cv::waitKey(1);

        if (key == ESC_KEY)
          break;

        // Generate a pose
        //ignition::math::Pose3d pose(controllerOut.speed, 0, 0, 0, 0, controllerOut.direction);

        // Convert to a pose message
        gazebo::msgs::Pose msg;
        //gazebo::msgs::Set(&msg, pose);
        movementPublisher->Publish(msg);

    }
    gazebo::client::shutdown();
}
