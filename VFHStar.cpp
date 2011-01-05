#include "VFHStar.h"
#include <Eigen/Core>
#include <map>

using namespace vfh_star;
using namespace Eigen;

VFHStar::VFHStar()
{
    mainHeadingWeight = 100;
    distanceWeight    = 1;
    turningWeight     = 0;
}

std::vector<base::Waypoint> VFHStar::getTrajectory(base::Pose const& start, double mainHeading)
{
    this->mainHeading = mainHeading;

    // Used for heuristics
    this->targetLineNormal =
        Eigen::Quaterniond(AngleAxisd(mainHeading, Vector3d::UnitZ())) * Vector3d::UnitY();
    this->targetLinePoint  =
        start.position + targetLineNormal * stepDistance * maxTreeDepth;

    std::cout << "target:" << std::endl;
    std::cout << "  point: "  << targetLinePoint.x() << " " << targetLinePoint.y() << " " << targetLinePoint.z() << std::endl;
    std::cout << "  normal: " << targetLineNormal.x() << " " << targetLineNormal.y() << " " << targetLineNormal.z() << std::endl;

    return TreeSearch::getTrajectory(start);
}

double VFHStar::getHeuristic(const TreeNode &node) const
{
    // double d_to_goal = distanceWeight * (targetLinePoint - node.getPose().position).dot(targetLineNormal);
    // int steps = ceil(d_to_goal / stepDistance);
    int steps = (maxTreeDepth - node.getDepth());

    double result = 0;
    while (steps-- > 0)
        result = result * discountFactor + 1;
    return result * discountFactor * stepDistance * distanceWeight;
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

    const double a = mainHeadingWeight;
    const double b = distanceWeight;
    const double c = turningWeight;
    
    const double direction = curNode.getDirection();
    
    double aPart = angleDiff(direction, mainHeading);
    double bPart = 0, cPart = 0;
    if(!curNode.isRoot())
    {
        bPart = (curNode.getPose().position - curNode.getParent()->getPose().position).norm();
        cPart = angleDiff(direction, curNode.getParent()->getDirection());
    }

    std::cerr << direction << " " << mainHeading << " " << aPart << std::endl;
    std::cerr << stepDistance << " " << bPart << std::endl;
    
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


