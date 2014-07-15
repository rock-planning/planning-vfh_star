#ifndef VFHSTAR_H
#define VFHSTAR_H

#include "HorizonPlanner.hpp"
#include "VFH.h"
#include "Types.h"

namespace vfh_star {
class VFHStar : public HorizonPlanner
{
    public:
        VFHStar();
        virtual ~VFHStar();
	
        void setCostConf(const VFHStarConf& config);
        const VFHStarConf& getCostConf() const;


	
        /**
         * Sets a new traversability map.
         * The map is used while computing the optimal path
         * to the horizon.
         * */
	void setNewTraversabilityGrid(const envire::TraversabilityGrid *trGrid);

        VFHStarDebugData getVFHStarDebugData(const std::vector< base::Waypoint >& trajectory);
        
    protected:
        VFHStarConf vfhStarConf;
        VFH vfh;
    
        /** Returns the estimated cost from the given node to the optimal node
         * reachable from that node. Note that this estimate must be a minorant,
         * i.e. must be smaller or equal than the actual value
         */
        virtual double getHeuristic(const TreeNode& node) const;

        /** Returns the cost of travelling from \c parent to \c node. It might
         * include a cost of "being at" \c node as well
         */
        virtual double getCostForNode(const ProjectedPose& projection, const base::Angle& direction, const TreeNode& parentNode) const;
    
        /**
         * Uses the VFH Algorithm to determine the next possible direction were the 
         * robot could drive without colliding with the environment. 
         * 
         * One could say this is a clever heuristic for reducing the sample space of the planner.
         * */
        AngleIntervals getNextPossibleDirections(const TreeNode& curNode) const;
        
        virtual bool validateNode(const TreeNode& node) const;
};
} // vfh_star namespace

#endif // VFHSTAR_H
