#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>

#include <fl/Headers.h>

#include <array>
#include <iostream>
#include <math.h>

static boost::mutex mutex;

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

const int subRanges = 5;
std::array<float, subRanges> rangeArray;

struct line
{
    float alpha;
    float radius;
    line(float alphaInit, float radiusInit) {
        alpha = alphaInit;
        radius = radiusInit;
    }
    line() {}
};

struct point
{
    float tehta;
    float radius;
    point(float tehtaInit, float radiusInit) {
        tehta = tehtaInit;
        radius = radiusInit;
    }
    point() {}
};

line calLine(std::vector<point> poins){
    float sum1T1 = 0.0f;
    float sum1B1 = 0.0f;
    float sum1T2 = 0.0f;
    float sum1B2 = 0.0f;

    line lin;

    for(size_t i = 0; i<poins.size(); i++){
        float tehtai = poins.at(i).tehta;
        float radiusi = poins.at(i).radius;

        float power = pow(radiusi,2);
        sum1T1 += power*sin(2*tehtai);
        sum1B1 += power*cos(2*tehtai);

        for(size_t j = 0; j<poins.size(); j++){
            float tehtaj = poins.at(j).tehta;
            float radiusj = poins.at(j).radius;
            sum1T2 += radiusi*radiusj*cos(tehtai)*sin(tehtaj);
            sum1B2 += radiusi*radiusj*cos(tehtai+tehtaj);
        }
    }

    lin.alpha = 1/2.0*atan((sum1T1-(2.0f/poins.size())*sum1T2)/(sum1B1-(1.0f/poins.size())*sum1B2));

    float sum2T = 0.0f;

    for(size_t i = 0; i<poins.size(); i++){
        float tehtai = poins.at(i).tehta;
        float radiusi = poins.at(i).radius;

        sum2T += radiusi*cos(tehtai-lin.alpha);
    }

    lin.radius = sum2T/poins.size();

    return lin;
}

float dist(point pos, line lin){
    return (float)(pos.radius*cos(pos.tehta-lin.alpha)-lin.radius);
}

int findMaxDistIndex(std::vector<point> poins, line lin){
    float max = 0;
    int maxIndex = -1;
    for (size_t i = 0; i < poins.size(); i++) {
        float dis = dist(poins.at(i), lin);
        if(max < dis){
            max = dis;
            maxIndex = i;
        }
    }
    return maxIndex;
}

float calDistSum(std::vector<point> poins, line lin){
    float disSum = 0.0f;
    for(size_t i = 0; i<poins.size(); i++){
        disSum += pow(dist(poins.at(i), lin),2);
    }
    return disSum;
}

std::vector<line> splitAndMerge(std::vector<point> poins){
    float maxDistThreshold = 10;
    std::vector<line> doneLines;

    std::vector<std::vector<point>> l;

    l.push_back(poins);

    while(not l.empty()){

        std::vector<point> set = l.front();
        l.pop_back();

        line lin = calLine(set);
        int maxIndex = findMaxDistIndex(set,lin);
        float maxDist = dist(set.at(maxIndex),lin);
        std::cout << "Number of sets: " << l.size() << " max dist: " << maxDist << std::endl;
        if(maxDist >= maxDistThreshold){
            std::vector<point> set1;
            std::vector<point> set2;

            for(size_t i = 0; i<maxIndex; i++){
                set1.push_back(set.at(i));
            }
            for(size_t i = maxIndex; i<set.size(); i++){
                set2.push_back(set.at(i));
            }
            l.push_back(set1);
            l.push_back(set2);
        }else{
            doneLines.push_back(lin);
        }
    }
    return doneLines;
}

void lidarCallbackImg(ConstLaserScanStampedPtr &msg) {

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

    if(i%(nranges/subRanges)==0 or i==(nranges-1)) collor = cv::Scalar(0,0, 255, 255);
    else if(range==rangeArray.at(i/(nranges/subRanges))) collor = cv::Scalar(0,255, 0, 255);

    cv::line(im, startpt * 16, endpt * 16, collor, 1, cv::LINE_AA, 4);


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

void lidarCallbackArrayUpdate(ConstLaserScanStampedPtr &msg) {

  //std::cout << ">> " << msg->DebugString() << std::endl;
  /*double angle_min = msg->scan().angle_min();
  double angle_max = msg->scan().angle_max();
  std::cout<< "[" << angle_min << ", " << angle_max << "]" << std::endl;*/
  //float angle_increment = float(msg->scan().angle_step());

  //float range_min = float(msg->scan().range_min());
  float range_max = float(msg->scan().range_max());


  int nranges = msg->scan().ranges_size();
  int nintensities = msg->scan().intensities_size();

  assert(nranges == nintensities);

  rangeArray.fill(range_max);

  for(int subR = 0; subR < subRanges; subR++){
      for(int i = (nranges/subRanges)*subR; i < (nranges/subRanges)*(subR+1); i++){
        float range = std::min(float(msg->scan().ranges(i)), range_max);
        if(range < rangeArray.at(subR)) rangeArray.at(subR) = range;
      }
  }
}

void lidarCallbackLines(ConstLaserScanStampedPtr &msg) {
    float angle_min = float(msg->scan().angle_min());
    float range_max = float(msg->scan().range_max());
    int nranges = msg->scan().ranges_size();
    int nintensities = msg->scan().intensities_size();
    float angle_increment = float(msg->scan().angle_step());
    assert(nranges == nintensities);

    std::vector<point> poins;

    for(int i = 0; i < nranges; i++){
        float angle = angle_min + i * angle_increment;
        float range = std::min(float(msg->scan().ranges(i)), range_max);
        poins.push_back(point(angle,range));
    }

    std::vector<line> lines = splitAndMerge(poins);

    for(line li : lines){
        std::cout << li.alpha << ", " << li.radius<< "; ";
    }
    std::cout << std::endl;
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

  //gazebo::transport::SubscriberPtr poseSubscriber =
  //    node->Subscribe("~/pose/info", poseCallback);

  gazebo::transport::SubscriberPtr cameraSubscriber =
      node->Subscribe("~/pioneer2dx/camera/link/camera/image", cameraCallback);

  gazebo::transport::SubscriberPtr lidarSubscriber2 =
      node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", lidarCallbackArrayUpdate);
  gazebo::transport::SubscriberPtr lidarSubscriber =
      node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", lidarCallbackImg);

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

  using namespace fl;
  Engine* engine = FllImporter().fromFile("/home/simonlbs/rb-rca5-group2/robot_control/AI/ObstacleAvoidance.fll");//Skal laves dymainsk

  std::string status;
  if (not engine->isReady(&status))
      throw Exception("[engine error] engine is not ready:\n" + status, FL_AT);

  InputVariable* sL2 = engine->getInputVariable("sL2");
  InputVariable* sL1 = engine->getInputVariable("sL1");
  InputVariable* sM = engine->getInputVariable("sM");
  InputVariable* sR1 = engine->getInputVariable("sR1");
  InputVariable* sR2 = engine->getInputVariable("sR2");

  OutputVariable* speed = engine->getOutputVariable("speed");
  OutputVariable* dir = engine->getOutputVariable("dir");

  const int key_esc = 27;
  /*
  const int key_left = 81;
  const int key_up = 82;
  const int key_down = 84;
  const int key_right = 83;
  //*/

  double speedOut = 0.0;
  double dirOut = 0.0;
  // Loop
  while (true) {
    gazebo::common::Time::MSleep(10);

    scalar senR2 = rangeArray.at(0);
    sR2->setValue(senR2);
    scalar senR1 = rangeArray.at(1);
    sR1->setValue(senR1);
    scalar senM = rangeArray.at(2);
    sM->setValue(senM);
    scalar senL1 = rangeArray.at(3);
    sL1->setValue(senL1);
    scalar senL2 = rangeArray.at(4);
    sL2->setValue(senL2);

    engine->process();

    FL_LOG("Speed.output = " << Op::str(speed->getValue()) << " Dir.output = " << Op::str(dir->getValue()));

    speedOut = speed->getValue();
    dirOut = dir->getValue();

    //Show range array
    //for(int i = 0; i < subRanges-1; i++) std::cout << rangeArray.at(i) << ", ";
    //std::cout<<rangeArray.at(subRanges-1)<<std::endl;

    mutex.lock();
    int key = cv::waitKey(1);
    mutex.unlock();

    if (key == key_esc)
      break;
    //Speed  -1.2 til 1.2
    //Dir -0.4 til 0.4
    /*
    if ((key == key_up) && (speedOut <= 1.2f))
      speedOut += 0.05;
    else if ((key == key_down) && (speedOut >= -1.2f))
      speedOut -= 0.05;
    else if ((key == key_right) && (dirOut <= 0.4f))
      dirOut += 0.05;
    else if ((key == key_left) && (dirOut >= -0.4f))
      dirOut -= 0.05;
    else {
      // slow down
      //      speed *= 0.1;
      //      dir *= 0.1;
    }
    /*/




    // Generate a pose
    ignition::math::Pose3d pose(speedOut, 0, 0, 0, 0, dirOut);

    // Convert to a pose message
    gazebo::msgs::Pose msg;
    gazebo::msgs::Set(&msg, pose);
    movementPublisher->Publish(msg);

  }

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
