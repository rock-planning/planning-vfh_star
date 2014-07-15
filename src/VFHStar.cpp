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
    vfh.setConfig(vfhStarConf.vfhConf);
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

double VFHStar::getCostForNode(const ProjectedPose& projection, const base::Angle &direction, const TreeNode& parentNode) const
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

TreeSearch::AngleIntervals VFHStar::getNextPossibleDirections(const TreeNode& curNode) const
{
    return vfh.getNextPossibleDirections(curNode.getPose());
}

bool VFHStar::validateNode(const TreeNode& node) const
{
    return vfh.validPosition(node.getPose());
}

VFHStarDebugData VFHStar::getVFHStarDebugData(const std::vector< base::Waypoint >& trajectory)
{
    VFHStarDebugData dd_out;
    dd_out.horizonOrigin = getHorizonOrigin();
    dd_out.horizonVector = getHorizonVector();
//     for(std::vector<base::Waypoint>::const_iterator it = trajectory.begin(); it != trajectory.end(); it++)
//     {
//         bool found = false;
//         for(std::vector<VFHDebugData>::const_iterator it2 = debugData.steps.begin(); it2 != debugData.steps.end(); it2++) 
//         {
//             if(it->position == base::Vector3d(it2->pose.position))
//             {
//                 dd_out.steps.push_back(*it2);
//                 found = true;
//                 break;
//             }
//             
//         }
//         if(!found && (it + 1) != trajectory.end() )
//         {
//             std::cerr << "BAD debug data is fishy" << std::endl;
//             throw std::runtime_error("Could not build VFHStarDebugData");
//         }
//     }
    return dd_out;
}

// void VFHStar::clearDebugData()
// {
//     debugData.generatedTrajectory.clear();
//     debugData.steps.clear();
// }

