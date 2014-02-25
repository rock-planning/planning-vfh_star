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

const TreeNode* HorizonPlanner::computePath(base::Pose const& start, const base::Angle &mainHeading_i, double horizon, const Eigen::Affine3d &body2Trajectory)
{    
    startPose_w = start;
    mainHeading_w = mainHeading_i;
    horizonDistance = horizon;

    //convert to tree frame
    const Affine3d world2Tree(getTreeToWorld().inverse());
    Vector3d startPos_tree = world2Tree * start.position;
    mainHeading = mainHeading_i + base::Angle::fromRad(base::Pose(world2Tree).getYaw());

    // Used for heuristics
    targetLineNormal = Eigen::Quaterniond(AngleAxisd(mainHeading.getRad(), Vector3d::UnitZ())) * Vector3d::UnitX();
    targetLinePoint  = startPos_tree + targetLineNormal * horizon;
    targetLine = Eigen::Quaterniond(AngleAxisd(mainHeading.getRad(), Vector3d::UnitZ())) * Vector3d::UnitY();

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



