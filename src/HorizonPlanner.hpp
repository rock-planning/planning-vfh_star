#ifndef HORIZONPLANNER_HPP
#define HORIZONPLANNER_HPP

#include "TreeSearch.h"
#include "Types.h"

namespace vfh_star
{
    
class HorizonPlanner : public TreeSearch
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        HorizonPlanner();
        virtual ~HorizonPlanner();

        std::vector<base::Trajectory> getTrajectories(const base::Pose& start, const base::Angle& mainHeading, double horizon, const Eigen::Affine3d& body2Trajectory = Eigen::Affine3d::Identity());
        const TreeNode* computePath(const base::Pose& start, const base::Angle& mainHeading, double horizon, const Eigen::Affine3d& body2Trajectory = Eigen::Affine3d::Identity());
        
        const base::Vector3d getHorizonOrigin() const;

        const base::Vector3d getHorizonVector() const;

        HorizonPlannerDebugData getDebugData() const;
        
    protected:
        base::Angle mainHeading;
        //start pose in world coordinates
        base::Pose startPose_w;
        //target heading in world frame
        base::Angle mainHeading_w;
        double horizonDistance;
        base::Vector3d targetLinePoint;
        base::Vector3d targetLineNormal;
        base::Vector3d targetLine;

        /** Returns true if \c node is behind the goal line
         */
        virtual bool isTerminalNode(const TreeNode& node) const;

        /** Returns the estimated cost from the given node to the optimal node
         * reachable from that node. Note that this estimate must be a minorant,
         * i.e. must be smaller or equal than the actual value
         */
        virtual double getHeuristic(const TreeNode &node) const;

        /** Returns the algebraic distance from \c pos to the goal line. If
         * the returned distance is negative, it means we crossed it.
         */
        double algebraicDistanceToGoalLine(const base::Position& pos) const;
};

}
#endif // HORIZONPLANNER_HPP
