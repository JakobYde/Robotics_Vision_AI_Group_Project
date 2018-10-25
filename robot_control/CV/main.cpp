#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>

static boost::mutex mutex;

void statCallback(ConstWorldStatisticsPtr &_msg) {
  (void)_msg;
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();
  //  std::cout << std::flush;
}

std::ofstream myfile;

void poseCallback(ConstPosesStampedPtr &_msg) {
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();

  for (int i = 0; i < _msg->pose_size(); i++) {
    if (_msg->pose(i).name() == "pioneer2dx") {

      myfile << std::setprecision(2) << std::fixed << std::setw(6)
                << _msg->pose(i).position().x() << std::setw(6) << ", "
                << _msg->pose(i).position().y() << std::setw(6) << ", "
                << _msg->pose(i).position().z() << std::setw(6) << ", "
                << _msg->pose(i).orientation().w() << std::setw(6) << ", "
                << _msg->pose(i).orientation().x() << std::setw(6) << ", "
                << _msg->pose(i).orientation().y() << std::setw(6) << ", "
                << _msg->pose(i).orientation().z() << std::endl;
      std::cout << std::setprecision(2) << std::fixed << std::setw(6)
                << _msg->pose(i).position().x() << std::setw(6) << ", "
                << _msg->pose(i).position().y() << std::setw(6) << ", "
                << _msg->pose(i).position().z() << std::setw(6) << ", "
                << _msg->pose(i).orientation().w() << std::setw(6) << ", "
                << _msg->pose(i).orientation().x() << std::setw(6) << ", "
                << _msg->pose(i).orientation().y() << std::setw(6) << ", "
                << _msg->pose(i).orientation().z() << std::endl;
    }
  }
}

void cameraCallback(ConstImageStampedPtr &msg) {

  std::size_t width = msg->image().width();
  std::size_t height = msg->image().height();
  const char *data = msg->image().data().c_str();
  cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

  //im = im.clone();
  cv::cvtColor(im, im, CV_RGB2BGR);

  //Jakobs snask
  //cv::Mat gray;
  cv::Mat detected;
  cv::Mat binary;
  cv::Mat hls;
  detected = im.clone();

  cv::cvtColor(im, hls, cv::COLOR_BGR2HLS);
  cv::Mat hlsch[3];
  cv::split(hls, hlsch);

  //cv::cvtColor(im, gray, cv::COLOR_RGB2GRAY);

  cv::threshold(hlsch[0], binary, 50, 255, cv::THRESH_BINARY);

  GaussianBlur( binary, binary, cv::Size(9, 9), 2, 0);

  std::vector<cv::Vec3f> circles;
  //for (int j = 2; j < 3; j++)
  //{
  HoughCircles(binary, circles, cv::HOUGH_GRADIENT, 1.2,
      binary.rows / 128,  // change this value to detect circles with different distances to each other
      100, 20, 0, 0 // change the last two parameters
                      // (min_radius & max_radius) to detect larger circles
  );

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

  for (size_t i = 0; i < circles.size(); i++)
  {
      cv::Vec3i c = circles[i];
      cv::Point center = cv::Point(c[0], c[1]);
      // circle center
      circle(detected, center, 1, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
      // circle outline
      int radius = c[2];
      circle(detected, center, radius, cv::Scalar(255, 0, 255), 1, cv::LINE_AA);
  }

  cv::resize(detected, detected, detected.size()*4);

  mutex.lock();
  cv::imshow("camera", im);
  cv::imshow("binary", binary);
  cv::imshow("detected circles", detected);
  mutex.unlock();
}

void lidarCallback(ConstLaserScanStampedPtr &msg) {
  myfile.open("/home/simonlbs/pos.cvs");
  //  std::cout << ">> " << msg->DebugString() << std::endl;
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
    cv::line(im, startpt * 16, endpt * 16, cv::Scalar(255, 255, 255, 255), 1,
             cv::LINE_AA, 4);

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
      node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", lidarCallback);

  // Publish to the robot vel_cmd topic
  gazebo::transport::PublisherPtr movementPublisher =
      node->Advertise<gazebo::msgs::Pose>("~/pioneer2dx/vel_cmd");

  // Publish a reset of the world
  gazebo::transport::PublisherPtr worldPublisher =
      node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
  gazebo::msgs::WorldControl controlMessage;
  controlMessage.mutable_reset()->set_all(true);
  worldPublisher->WaitForConnection();
  worldPublisher->Publish(controlMessage);

  const int key_left = 81;
  const int key_up = 82;
  const int key_down = 84;
  const int key_right = 83;
  const int key_esc = 27;

  float speed = 0.0;
  float dirspeed = 0.0;

  // Loop
  while (true) {
    gazebo::common::Time::MSleep(10);

    mutex.lock();
    int key = cv::waitKey(1);
    mutex.unlock();

    if (key == key_esc) break;

    dirspeed = 0;
    if (key == key_up) speed += 1.5;
    if (key == key_down) speed += -0.85;
    if (key == key_right) dirspeed = 5;
    if (key == key_left) dirspeed = -5;

    // Generate a pose
    ignition::math::Pose3d pose(double(speed), 0, 0, 0, 0, double(dirspeed));

    speed *= 0.95;

    // Convert to a pose message
    gazebo::msgs::Pose msg;
    gazebo::msgs::Set(&msg, pose);
    movementPublisher->Publish(msg);
  }

  // Make sure to shut everything down.
  gazebo::client::shutdown();
  myfile.close();
}
