#include "HorizonPlanner.hpp"
#include <Eigen/Core>
#include <map>
#include <iostream>

using namespace vfh_star;
using namespace Eigen;

HorizonPlanner::HorizonPlanner()
{
}

HorizonPlanner::~HorizonPlanner()
{
}

const base::Vector3d HorizonPlanner::getHorizonOrigin() const
{
    return startPose_w.position + Eigen::Quaterniond(AngleAxisd(mainHeading_w.getRad(), Vector3d::UnitZ())) * Vector3d::UnitX() * horizonDistance;
}

const base::Vector3d HorizonPlanner::getHorizonVector() const
{
    return Eigen::Quaterniond(AngleAxisd(mainHeading_w.getRad(), Vector3d::UnitZ())) * Vector3d::UnitY();
}


std::vector< base::Trajectory > HorizonPlanner::getTrajectories(const base::Pose& start, const base::Angle &mainHeading, double horizon, const Eigen::Affine3d &body2Trajectory)
{
    const TreeNode *node = computePath(start, mainHeading, horizon, body2Trajectory);
    return buildTrajectoriesTo(node, body2Trajectory);
}

const TreeNode* HorizonPlanner::computePath(base::Pose const& start, const base::Angle &mainHeading, double horizon, const Eigen::Affine3d &body2Trajectory)
{    
    startPose_w = start;
    mainHeading_w = mainHeading_i;
    horizonDistance = horizon;
    this->mainHeading = mainHeading + base::Angle::fromRad(base::Pose(getTreeToWorld().inverse()).getYaw());

    // Used for heuristics
    this->targetLineNormal =
        Eigen::Quaterniond(AngleAxisd(mainHeading.getRad(), Vector3d::UnitZ())) * Vector3d::UnitX();
    this->targetLinePoint  =
        start.position + targetLineNormal * horizon;
        
    this->targetLine = Eigen::Quaterniond(AngleAxisd(mainHeading.getRad(), Vector3d::UnitZ())) * Vector3d::UnitY();

    targetLineNormal +=  Vector3d(0, 0, start.position.z());
    
    std::cout << "target:" << std::endl;
    std::cout << "  point: "  << targetLinePoint.x() << " " << targetLinePoint.y() << " " << targetLinePoint.z() << std::endl;
    std::cout << "  normal: " << targetLineNormal.x() << " " << targetLineNormal.y() << " " << targetLineNormal.z() << std::endl;
    
    return compute(start);
}

HorizonPlannerDebugData HorizonPlanner::getDebugData() const
{
    HorizonPlannerDebugData ret;
    ret.horizonOrigin = getHorizonOrigin();
    ret.horizonVector = getHorizonVector();
    
    return ret;
}

double HorizonPlanner::algebraicDistanceToGoalLine(const base::Position& pos) const
{
    //check weather we crossed the target line;
    
    //targetLineNormal is normalized
    //so this solves to |(targetLinePoint - pos)| * cos alpha = b
    //b is the adjecent which is the searched distance to the line.
    return (targetLinePoint - pos).dot(targetLineNormal);
}

bool HorizonPlanner::isTerminalNode(const TreeNode& node) const
{
    double d = algebraicDistanceToGoalLine(node.getPose().position);
    return d <= 0;
}

double HorizonPlanner::getHeuristic(const TreeNode &node) const
{
    double d_to_goal = algebraicDistanceToGoalLine(node.getPose().position);
    if (d_to_goal < 0)
        return 0;
    
    return d_to_goal;
}



