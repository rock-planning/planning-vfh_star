#include "VFHStar.h"
#include <Eigen/Core>
#include <map>

using namespace vfh_star;
using namespace Eigen;

VFHStar::VFHStar()
{
}

const VFHStarConf& VFHStar::getCostConf() const
{
    return cost_conf;
}

void VFHStar::setCostConf(const VFHStarConf& conf)
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
        start.position + targetLineNormal * horizon;
	
    this->targetLine = Eigen::Quaterniond(AngleAxisd(mainHeading, Vector3d::UnitZ())) * Vector3d::UnitX();

    targetLineNormal +=  Vector3d(0, 0, start.position.z());
    
    std::cout << "target:" << std::endl;
    std::cout << "  point: "  << targetLinePoint.x() << " " << targetLinePoint.y() << " " << targetLinePoint.z() << std::endl;
    std::cout << "  normal: " << targetLineNormal.x() << " " << targetLineNormal.y() << " " << targetLineNormal.z() << std::endl;

    return TreeSearch::getWaypoints(start);
}

double VFHStar::algebraicDistanceToGoalLine(const base::Position& pos) const
{
    //check weather we crossed the target line;
    
    //targetLineNormal is normalized
    //so this solves to |(targetLinePoint - pos)| * cos alpha = b
    //b is the adjecent which is the searched distance to the line.
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

double VFHStar::getCostForNode(const base::Pose& pose, double direction, const TreeNode& parentNode) const
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
    
    double aPart = angleDiff(direction, mainHeading);
    double bPart = (pose.position - parentNode.getPose().position).norm();
    double cPart = angleDiff(direction, parentNode.getDirection());

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
    return atan2(end.y() - start.y(), end.x() - start.x()) + M_PI / 2.0;
}


