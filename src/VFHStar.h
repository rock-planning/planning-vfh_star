#ifndef VFHSTAR_H
#define VFHSTAR_H

#include "HorizonPlanner.hpp"
#include "VFH.h"

namespace vfh_star {
class VFHStar : public HorizonPlanner
{
    public:
        VFHStar();
        virtual ~VFHStar();
	
        void setCostConf(const VFHStarConf& config);
        const VFHStarConf& getCostConf() const;

	base::Vector3d getHorizonOrigin() 
	{
	    return targetLinePoint;
	}

	base::Vector3d getHorizonVector() 
	{
	    return targetLine;
	}

	void setNewTraversabilityGrid(const envire::TraversabilityGrid *trGrid);

        
    protected:
        VFHStarConf vfhStarConf;
        VFH vfh;
    
        /** Returns the estimated cost from the given node to the optimal node
         * reachable from that node. Note that this estimate must be a minorant,
         * i.e. must be smaller or equal than the actual value
         */
//         virtual double getHeuristic(const TreeNode &node) const;

        /** Returns the cost of travelling from \c parent to \c node. It might
         * include a cost of "being at" \c node as well
         */
        virtual double getCostForNode(const base::Pose& pose, const base::Angle& direction, const vfh_star::TreeNode& parentNode) const;

        /**
         * Uses the VFH Algorithm to determine the next possible direction were the 
         * robot could drive without colliding with the environment. 
         * 
         * One could say this is a clever heuristic for reducing the sample space of the planner.
         * */
        AngleIntervals getNextPossibleDirections(const vfh_star::TreeNode& curNode) const;
        
};
} // vfh_star namespace

#endif // VFHSTAR_H
