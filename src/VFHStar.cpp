#include "VFHStar.h"
#include <Eigen/Core>
#include <map>

using namespace vfh_star;
using namespace Eigen;

VFHStar::VFHStar()
{
}

const VFHStarConfiguration& VFHStar::getCostConfiguration() const
{
    return cost_conf;
}

void VFHStar::setCostConfiguration(const VFHStarConfiguration& conf)
{
    cost_conf = conf;
}

base::geometry::Spline<3> VFHStar::getTrajectory(base::Pose const& start, double mainHeading, double horizon)
{
    return TreeSearch::waypointsToSpline(getWaypoints(start, mainHeading, horizon));
}

std::vector<base::Waypoint> VFHStar::getWaypoints(base::Pose const& start, double mainHeading, double horizon)
{
    this->mainHeading = mainHeading;

    // Used for heuristics
    this->targetLineNormal =
        Eigen::Quaterniond(AngleAxisd(mainHeading, Vector3d::UnitZ())) * Vector3d::UnitY();
    this->targetLinePoint  =
        start.position + targetLineNormal * search_conf.stepDistance * search_conf.searchDepth;

    std::cout << "target:" << std::endl;
    std::cout << "  point: "  << targetLinePoint.x() << " " << targetLinePoint.y() << " " << targetLinePoint.z() << std::endl;
    std::cout << "  normal: " << targetLineNormal.x() << " " << targetLineNormal.y() << " " << targetLineNormal.z() << std::endl;

    return TreeSearch::getWaypoints(start);
}

double VFHStar::algebraicDistanceToGoalLine(const base::Position& pos) const
{
    return (targetLinePoint - pos).dot(targetLineNormal);
}

bool VFHStar::isTerminalNode(const TreeNode& node) const
{
    double d = algebraicDistanceToGoalLine(node.getPose().position);
    return d <= 0;
}

double VFHStar::getHeuristic(const TreeNode &node) const
{
    double d_to_goal = algebraicDistanceToGoalLine(node.getPose().position);
    if (d_to_goal < 0)
        return 0;

    int steps = ceil(d_to_goal / search_conf.stepDistance);
    double result = 0;
    while (steps-- > 0)
        result = result * search_conf.discountFactor + 1;
    return result * search_conf.discountFactor * search_conf.stepDistance * cost_conf.distanceWeight;
}

double VFHStar::getCostForNode(const TreeNode& curNode) const
{
    /**
    * cost is build from three factors:
    * a * difference of direction and heading direction
    * b * distance travelled
    * c * difference between current direction and last direction
    *
    * direction means the direction that was selected, in the previouse step, thus leading to this node
    */

    const double a = cost_conf.mainHeadingWeight;
    const double b = cost_conf.distanceWeight;
    const double c = cost_conf.turningWeight;
    
    const double direction = curNode.getDirection();
    
    double aPart = angleDiff(direction, mainHeading);
    double bPart = 0, cPart = 0;
    if(!curNode.isRoot())
    {
        bPart = (curNode.getPose().position - curNode.getParent()->getPose().position).norm();
        cPart = angleDiff(direction, curNode.getParent()->getDirection());
    }

    return a * aPart + b * bPart + c * cPart;
}

double VFHStar::angleDiff(const double &a1, const double &a2) const
{
    double d = a1-a2;
    return std::min(std::min(fabs(d), fabs(d - M_PI*2)),fabs(d + M_PI*2));
}

double VFHStar::getMotionDirection(const Vector3d &start, const Vector3d &end) const
{
    //note, we define y as our zero heading
    return atan2(end.y() - start.y(), end.x() - end.y()) + M_PI / 2.0;
}


