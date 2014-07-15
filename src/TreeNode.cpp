#include "TreeNode.hpp"

namespace vfh_star {
    
TreeNode::TreeNode()
{
    clear();
}

TreeNode::TreeNode(const base::Pose& pose, const base::Angle& dir, const DriveMode* driveMode, uint8_t driveModeNr)
    : parent(this)
    , is_leaf(true)
    , pose(pose)
    , yaw(base::Angle::fromRad(pose.getYaw()))
    , direction(dir)
    , cost(0)
    , heuristic(0)
    , costFromParent(0)
    , driveMode(driveMode)
    , driveModeNr(driveModeNr)
    , depth(0)
    , index(0)
    , updated_cost(false)
    , positionTolerance(0)
    , headingTolerance(0)
{
    clear();
    direction = dir;
    this->pose = pose;
//     yaw = pose.getYaw();
    this->driveMode=driveMode;
    yaw = base::Angle::fromRad(pose.getYaw());
}

void TreeNode::clear()
{
    parent = this;
    pose = base::Pose();
    yaw = base::Angle();
    is_leaf = true;
    cost = 0;
    heuristic = 0;
    costFromParent = 0;
    driveMode = 0;
    driveModeNr = std::numeric_limits<uint8_t>::max();
    depth = 0;
    index = 0;
    updated_cost = false;
    positionTolerance = 0;
    headingTolerance = 0;
    direction = base::Angle();
    childs.clear();
}

const base::Vector3d &TreeNode::getPosition() const
{
    return pose.position;
}

const base::Angle &TreeNode::getYaw() const
{
    return yaw;
}

double TreeNode::getHeuristic() const
{
    return heuristic;
}

void TreeNode::setHeuristic(double value)
{
    heuristic = value;
}

double TreeNode::getHeuristicCost() const
{
    return cost + heuristic;
}

uint8_t TreeNode::getDriveModeNr() const
{
    return driveModeNr;
}

const DriveMode* TreeNode::getDriveMode() const
{
    assert(driveMode);
    return driveMode;
}

void TreeNode::setDriveMode(const DriveMode* dm)
{
    driveMode = dm;
}

double TreeNode::getCost() const
{
    return cost;
}
void TreeNode::setCost(double value)
{
    cost = value;
}

void TreeNode::setCostFromParent(double value)
{
    costFromParent = value;
}

double TreeNode::getCostFromParent() const
{
    return costFromParent;
}

void TreeNode::addChild(TreeNode* child)
{
    is_leaf = false;
    child->parent = this;
    childs.push_back(child);
}

void TreeNode::removeChild(TreeNode* child)
{
    std::vector<TreeNode *>::iterator it = std::find(childs.begin(), childs.end(), child);
    if(it != childs.end())
    {
        childs.erase(it);
    }
    if(childs.empty())
        is_leaf = true;
}

const std::vector< TreeNode* >& TreeNode::getChildren() const
{
    return childs;
}

const TreeNode* TreeNode::getParent() const
{
    return parent;
}

const base::Pose& TreeNode::getPose() const
{
    return pose;
}

int TreeNode::getDepth() const
{
    return depth;
}

bool TreeNode::isRoot() const
{
    return parent == this;
}

bool TreeNode::isLeaf() const
{
    return is_leaf;
}

const base::Angle &TreeNode::getDirection() const
{
    return direction;
}

int TreeNode::getIndex() const
{
    return index;
}

double TreeNode::getPositionTolerance() const
{
    return positionTolerance;
}
void TreeNode::setPositionTolerance(double tol)
{
    positionTolerance = tol;
}
double TreeNode::getHeadingTolerance() const
{
    return headingTolerance;
}
void TreeNode::setHeadingTolerance(double tol)
{
    headingTolerance = tol;
}

}