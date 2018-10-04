#ifndef LASERSCANNER_H
#define LASERSCANNER_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <vector>

/*************************************************************/
/*************************************************************/

#define LASERSCANNER_DEFAULT_MAX 10
#define LASERSCANNER_DEFAULT_MIN 0.08

/*************************************************************/
/*************************************************************/

struct LaserScannerReading
{
    float fDistance;
    float fAngle;
};

/*************************************************************/
/*************************************************************/

class LaserScanner
{
public:
    LaserScanner();
    virtual ~LaserScanner() = default;

    virtual void parseLaserScannerMessage(ConstLaserScanStampedPtr& msg);
    virtual float getClosestDistance(float f_start_angle, float f_end_angle);
    virtual float getClosestDirection(float f_start_angle, float f_end_angle);

protected:
    boost::mutex                     m_mutex;
    std::vector<LaserScannerReading> m_vsReadings;
    int                              m_nLastUpdateSeconds;
    int                              m_nLastUpdateNanoSeconds;

    float   m_fRangeMax = LASERSCANNER_DEFAULT_MAX;
    float   m_fRangeMin = LASERSCANNER_DEFAULT_MIN;

    float   m_fAngleIncrement;
};

/*************************************************************/
/*************************************************************/

#endif // LASERSCANNER_H
