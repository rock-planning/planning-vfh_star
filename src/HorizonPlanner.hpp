#ifndef HORIZONPLANNER_HPP
#define HORIZONPLANNER_HPP

#include "TreeSearch.h"

namespace vfh_star
{
    
class HorizonPlanner : public TreeSearch
{
    public:
        HorizonPlanner();
        virtual ~HorizonPlanner();

        std::vector<base::Waypoint> getWaypoints(const base::Pose& start, const base::Angle& mainHeading, double horizon);
        base::geometry::Spline<3> getTrajectory(const base::Pose& start, const base::Angle& mainHeading, double horizon);
        std::vector<base::Trajectory> getTrajectories(const base::Pose& start, const base::Angle& mainHeading, double horizon, const Eigen::Affine3d& body2Trajectory);
        const TreeNode* computePath(const base::Pose& start, const base::Angle& mainHeading, double horizon, const Eigen::Affine3d& body2Trajectory);
        
        base::Vector3d getHorizonOrigin() 
        {
            return targetLinePoint;
        }

        base::Vector3d getHorizonVector() 
        {
            return targetLine;
        }

    protected:
        base::Angle mainHeading;
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
