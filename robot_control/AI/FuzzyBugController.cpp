#include "FuzzyBugController.h"

/*************************************************************/
/*************************************************************/

FuzzyBugController::FuzzyBugController(LaserScanner *pc_laser_scanner, float leftStartAngle_in,float leftEnsAngle_in,float rightStartAngle_in,float rightEndAngle_in,float centerStartAngle_in,float centerEndAngle_in) : m_pcLaserScanner(pc_laser_scanner)
{
    leftStartAngle = leftStartAngle_in;
    leftEnsAngle = leftEnsAngle_in;
    rightStartAngle = rightStartAngle_in;
    rightEndAngle = rightEndAngle_in;
    centerStartAngle = centerStartAngle_in;
    centerEndAngle = centerEndAngle_in;

}

/*************************************************************/
/*************************************************************/

ControlOutput FuzzyBugController::getControlOutput(float angleError, float goalDistance)
{
    m_pflSensorLeft->setValue(m_pcLaserScanner->getClosestDistance(leftStartAngle, leftEnsAngle));
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
    m_pcFLEngine = FllImporter().fromFile("/home/mnj/rb-rca5-group2/robot_control/AI/fuzzyObjAndGoalController.fll");

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
