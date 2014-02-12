#include "VFHStar.h"
#include <Eigen/Core>
#include <map>
#include <iostream>

using namespace vfh_star;
using namespace Eigen;

VFHStar::VFHStar()
{
}

VFHStar::~VFHStar()
{
}

const VFHStarConf& VFHStar::getCostConf() const
{
    return vfhStarConf;
}

void VFHStar::setCostConf(const VFHStarConf& conf)
{
    vfhStarConf = conf;
}

double VFHStar::getHeuristic(const TreeNode &node) const
{
    double d_to_goal = HorizonPlanner::getHeuristic(node);

    int steps = ceil(d_to_goal / search_conf.stepDistance);
    double result = 0;
    while (steps-- > 0)
        result = result * search_conf.discountFactor + 1;
    
    return result * search_conf.discountFactor * search_conf.stepDistance * vfhStarConf.distanceWeight;
}

void VFHStar::setNewTraversabilityGrid(const envire::TraversabilityGrid* trGrid)
{
    vfh.setNewTraversabilityGrid(trGrid);
}

double VFHStar::getCostForNode(const vfh_star::ProjectedPose& projection, const base::Angle &direction, const vfh_star::TreeNode& parentNode) const
{
    /**
    * cost is build from three factors:
    * a * difference of direction and heading direction
    * b * distance travelled
    * c * difference between current direction and last direction
    *
    * direction means the direction that was selected, in the previouse step, thus leading to this node
    */

    const double a = vfhStarConf.mainHeadingWeight;
    const double b = vfhStarConf.distanceWeight;
    const double c = vfhStarConf.turningWeight;
    
    const base::Pose &pose(projection.pose);
    
    double aPart = fabs((base::Angle::fromRad(pose.getYaw()) - mainHeading).getRad());
    double bPart = (pose.position - parentNode.getPose().position).norm();
    double cPart = fabs((direction - parentNode.getDirection()).getRad());

    double distToTarget = algebraicDistanceToGoalLine(pose.position);
    if(distToTarget < bPart)
        bPart = distToTarget;
    
    return a * aPart + b * bPart + c * cPart;
}

TreeSearch::AngleIntervals VFHStar::getNextPossibleDirections(const vfh_star::TreeNode& curNode) const
{
    const double &obstacleSafetyDist(vfhStarConf.vfhConf.obstacleSafetyDistance);
    const double &robotWidth(vfhStarConf.vfhConf.robotWidth);
    
    VFHDebugData dd;
    TreeSearch::AngleIntervals ret;
    ret = vfh.getNextPossibleDirections(curNode.getPose(), obstacleSafetyDist, robotWidth, &dd);
//     debugData.steps.push_back(dd);

    return ret;
}


