#include "FuzzyBugController.h"

/*************************************************************/
/*************************************************************/

FuzzyBugController::FuzzyBugController(LaserScanner *pc_laser_scanner) : m_pcLaserScanner(pc_laser_scanner)
{
}

/*************************************************************/
/*************************************************************/

ControlOutput FuzzyBugController::getControlOutput(float angleError, float goalDistance, float center_angle_pct)
{
    float angle_min = -2.26889;
    float angle_max = 2.2689;
    float angle_step = 0.0228029648241206;
    float total_angle_range = abs(angle_min)+abs(angle_max);

    float rightStartAngle = angle_min;
    float rightEndAngle = angle_step*round((angle_min+total_angle_range*(1-center_angle_pct)/2)/angle_step);
    float centerStartAngle = rightEndAngle;
    float centerEndAngle = angle_step*round((centerStartAngle+total_angle_range*center_angle_pct)/angle_step);

    float leftStartAngle= centerEndAngle;
    float leftEndAngle = angle_max;

    m_pflSensorLeft->setValue(m_pcLaserScanner->getClosestDistance(leftStartAngle, leftEndAngle));
    m_pflSensorCenter->setValue(m_pcLaserScanner->getClosestDistance(centerStartAngle, centerEndAngle));
    m_pflSensorRight->setValue(m_pcLaserScanner->getClosestDistance(rightStartAngle, rightEndAngle));
    m_pflAngleError->setValue(angleError);
    m_pflGoalDistance->setValue(goalDistance);

    m_pcFLEngine->process();

    ControlOutput out;
    out.direction = m_pflSteerDirection->getValue();
    out.speed     = m_pflSpeed->getValue();

    return out;
}

/*************************************************************/
/*************************************************************/

void FuzzyBugController::buildController()
{
    using namespace fl;
    m_pcFLEngine = FllImporter().fromFile("../AI/fuzzyObjAndGoalController.fll");

    std::string status;
    if (not m_pcFLEngine->isReady(&status))
        throw Exception("[engine error] engine is not ready:n" + status, FL_AT);

    m_pflSensorLeft        = m_pcFLEngine->getInputVariable("SensorLeft");
    m_pflSensorCenter      = m_pcFLEngine->getInputVariable("SensorCenter");
    m_pflSensorRight       = m_pcFLEngine->getInputVariable("SensorRight");
    m_pflAngleError        = m_pcFLEngine->getInputVariable("AngleError");
    m_pflGoalDistance     = m_pcFLEngine->getInputVariable("GoalDistance");

    m_pflSteerDirection    = m_pcFLEngine->getOutputVariable("SteerDirection");
    m_pflSpeed             = m_pcFLEngine->getOutputVariable("Speed");
}

/*************************************************************/
/*************************************************************/
