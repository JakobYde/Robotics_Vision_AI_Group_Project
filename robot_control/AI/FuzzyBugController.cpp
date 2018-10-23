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

ControlOutput FuzzyBugController::getControlOutput(float angleError, float goalDisttanse)
{
    m_pflSencorLeft->setValue(m_pcLaserScanner->getClosestDistance(leftStartAngle, leftEnsAngle));
    m_pflSencorCenter->setValue(m_pcLaserScanner->getClosestDistance(centerStartAngle, centerEndAngle));
    m_pflSencorRight->setValue(m_pcLaserScanner->getClosestDistance(rightStartAngle, rightEndAngle));
    m_pflAngleError->setValue(angleError);
    m_pflGoalDisttanse->setValue(goalDisttanse);

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
    m_pcFLEngine = FllImporter().fromFile("/home/simonlbs/rb-rca5-group2/robot_control/AI/fuzzyObjAndGoalController.fll");

    std::string status;
    if (not m_pcFLEngine->isReady(&status))
        throw Exception("[engine error] engine is not ready:n" + status, FL_AT);

    m_pflSencorLeft        = m_pcFLEngine->getInputVariable("SencorLeft");
    m_pflSencorCenter      = m_pcFLEngine->getInputVariable("SencorCenter");
    m_pflSencorRight       = m_pcFLEngine->getInputVariable("SencorRight");
    m_pflAngleError        = m_pcFLEngine->getInputVariable("AngleError");
    m_pflGoalDisttanse     = m_pcFLEngine->getInputVariable("GoalDisttanse");

    m_pflSteerDirection    = m_pcFLEngine->getOutputVariable("SteerDirection");
    m_pflSpeed             = m_pcFLEngine->getOutputVariable("Speed");
}

/*************************************************************/
/*************************************************************/
