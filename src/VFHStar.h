#ifndef VFHSTAR_H
#define VFHSTAR_H

#include "TreeSearch.h"

namespace vfh_star {
struct VFHStarConfiguration
{
    double mainHeadingWeight;
    double distanceWeight;
    double turningWeight;

    VFHStarConfiguration()
        : mainHeadingWeight(1)
        , distanceWeight(1)
        , turningWeight(1) {}
};

class VFHStar : public TreeSearch
{
    public:
        VFHStar();

        std::vector<base::Waypoint> getWaypoints(base::Pose const& start, double mainHeading, double horizon);
        base::geometry::Spline<3> getTrajectory(base::Pose const& start, double mainHeading, double horizon);

        void setCostConfiguration(const VFHStarConfiguration& config);
        const VFHStarConfiguration& getCostConfiguration() const;

    private:
        double angleDiff(const double &a1, const double &a2) const;
        double getMotionDirection(const Eigen::Vector3d &start, const Eigen::Vector3d &end) const;

        double mainHeading;
        VFHStarConfiguration cost_conf;

    protected:
        /** Returns true if \c node is behind the goal line
         */
        virtual bool isTerminalNode(const TreeNode& node) const;

        /** Returns the estimated cost from the given node to the optimal node
         * reachable from that node. Note that this estimate must be a minorant,
         * i.e. must be smaller or equal than the actual value
         */
        virtual double getHeuristic(const TreeNode &node) const;

        /** Returns the cost of travelling from \c parent to \c node. It might
         * include a cost of "being at" \c node as well
         */
        virtual double getCostForNode(const TreeNode& node) const;

        /** Returns the algebraic distance from \c pos to the goal line. If
         * the returned distance is negative, it means we crossed it.
         */
        double algebraicDistanceToGoalLine(const base::Position& pos) const;
};
} // vfh_star namespace

#endif // VFHSTAR_H
