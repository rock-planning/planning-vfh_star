#ifndef VFHSTAR_TYPES_H
#define VFHSTAR_TYPES_H

#include <boost/cstdint.hpp>
#include <base/pose.h>
#include <vector>
#include <base/waypoint.h>
#include <base/time.h>

namespace vfh_star
{
    struct AngleSampleConf
    {
        AngleSampleConf() : angularSamplingMin(0), angularSamplingMax(0), angularSamplingNominalCount(1) {}
        /** Minimum angle between two projected poses */
        double angularSamplingMin;
        /** Maximum angle between two projected poses */
        double angularSamplingMax;
        /** Nominal number of samples in a given interval */
        int angularSamplingNominalCount;
    };
    
    struct TreeSearchConf {
        int maxTreeSize;
        double stepDistance; //! the distance in meters between two steps in the search

        /**
         * Configuration for direction sampling outside of
         * 'forward' direction of the robot/curNode.
         * */
        AngleSampleConf directionSampleConf;
        
        /**
         * Configuration for direction sampling in the
         * 'forward' direction of the robot / curNode.
         * */
        AngleSampleConf directionOversampleConf;
        
        //! the cost discount factor applied on the cost of nodes at depth D + 1 w.r.t. the node at depth D
        double discountFactor; 

        /** if two nodes are below this threshhold in position
	 * and are pointing in the same direction, they are 
	 * considered the same and one get's removed
	 * */
	double identityPositionThreshold;
	/**
	 * Maximum yaw diviantion of two node for identitiy check
	 * */
	double identityYawThreshold;
	
        /**
         * Width of the area in front of the robot, 
         * were more directions are sampled 
         * */
        double oversamplingWidth;


	base::Time maxSeekTime;
	
        TreeSearchConf()
            : maxTreeSize(0)
            , stepDistance(0.5)
            , discountFactor(1.0)
            , identityPositionThreshold(-1)
            , identityYawThreshold(-1),
            oversamplingWidth(0.0)
    {};
                
        /**
         * This function computes the position and yaw threshold
         * from the stepdistance and angularSamplingMin parameters
         * */
        void computePosAndYawThreshold();
    };

    struct VFHConf
    {
        VFHConf(): obstacleSafetyDistance(0.0),
                    robotWidth(0.0),
                    obstacleSenseRadius(0.0)
        {
        }
        
        //! the margin distance between the robot and the obstacles
        double obstacleSafetyDistance; 

        //! the radius of the circle used to model the robot
        double robotWidth; 

        /**
         * Radius in which obstacles are sensed
         * per step.
         * */
        double obstacleSenseRadius;
    };
    
    struct VFHStarConf
    {
        VFHConf vfhConf;
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

