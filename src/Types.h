#ifndef VFHSTAR_TYPES_H
#define VFHSTAR_TYPES_H

#include <boost/cstdint.hpp>
#include <base/pose.h>
#include <vector>
#include <base/waypoint.h>
#include <base/time.h>

namespace vfh_star
{
    struct TreeSearchConf {
        int maxTreeSize;
        double stepDistance; //! the distance in meters between two steps in the search

        /** Minimum angle between two projected poses */
        double angularSamplingMin;
        /** Maximum angle between two projected poses */
        double angularSamplingMax;
        /** Nominal number of samples in a given interval */
        int angularSamplingNominalCount;

        double discountFactor; //! the cost discount factor applied on the cost of nodes at depth D + 1 w.r.t. the node at depth D
        double obstacleSafetyDistance; //! the margin distance between the robot and the obstacles
        double robotWidth; //! the radius of the circle used to model the robot

	/** Maximum number of nodes that may be evaluated */
	int maxSeekSteps;
	
	/** if two nodes are below this threshhold in position
	 * and 'parralletity' they are considered the same
	 * and one get's removed
	 * */
	double identityThreshold;

	/**
	 * Max height of step between two cells in height map,
	 * that will caount as traversable
	 * */
	double maxStepSize;

	base::Time maxSeekTime;
	
        TreeSearchConf();
    };

    struct VFHStarConf
    {
	
	/**
	 * Radius in which obstacles are sensed
	 * per step.
	 * */
	double obastacleSenseRadius;
        double mainHeadingWeight;
        double distanceWeight;
        double turningWeight;

        VFHStarConf()
            : mainHeadingWeight(1)
            , distanceWeight(1)
            , turningWeight(1) {}
    };

    class VFHDebugData
    {
    public:
        base::Pose pose;
        std::vector< uint8_t > histogram;
        double senseRadius;
        double obstacleSafetyDist;
        double robotWidth;
    };

}

#endif

