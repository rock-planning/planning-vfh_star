#ifndef TREENODE_H
#define TREENODE_H

#include <base/Pose.hpp>
#include "DriveMode.hpp"
#include <map>

namespace vfh_star {
    
class TreeNode
{
    friend class Tree;
    friend class TreeSearch;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        TreeNode();
        TreeNode(const base::Pose &pose, const base::Angle &dir, DriveMode const *driveMode, uint8_t driveModeNr);
        
        void clear();

        bool isRoot() const;
        bool isLeaf() const;
        
        const base::Pose &getPose() const;
        const TreeNode *getParent() const;
        
        void addChild(TreeNode *child);
        void removeChild(TreeNode *child);
        const std::vector<TreeNode *> &getChildren() const;
        
        const base::Angle &getDirection() const;
        int getDepth() const;

        const base::Angle &getYaw() const;
        const base::Vector3d &getPosition() const;
        
        int getIndex() const;

        double getCost() const;
        void setCost(double value);
        double getHeuristic() const;
        void setHeuristic(double value);
        double getHeuristicCost() const;
        
        void setDriveMode(DriveMode const *dm);
        DriveMode const *getDriveMode() const;

        uint8_t getDriveModeNr() const;
        void setDriveModeNr(uint8_t nr);
        
        void setCostFromParent(double value);
        double getCostFromParent() const;
        
        double getPositionTolerance() const;
        void setPositionTolerance(double tol);
        double getHeadingTolerance() const;
        void setHeadingTolerance(double tol);
        
    private:
        TreeNode *parent;
        bool is_leaf;
        
        ///pose of the node, note the orientation of the node and the direction may differ, 
        ///because of kinematic constrains of the robot
        base::Pose pose;

        ///yaw of the pose
        base::Angle yaw;
        
        ///direction, that was choosen, that lead to this node
        base::Angle direction;
        
        ///cost from start to this node
        double cost;
        
        ///heuristic from node to goal
        double heuristic;
        
        ///cost from parent to this node
        double costFromParent;
        
        ///the drive mode that was used to get to the current position
        DriveMode const *driveMode;
        
        ///Nr used to reference the drive mode in the tree.
        uint8_t driveModeNr;
        
        int depth;
        int index;
        bool updated_cost;

        float positionTolerance;
        float headingTolerance;

        std::vector<TreeNode *> childs;
        
        // Used by TreeSearch only
        mutable std::multimap<double, TreeNode *>::iterator candidate_it;
};

}
#endif // TREENODE_H
