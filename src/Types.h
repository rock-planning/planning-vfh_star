#ifndef VFHSTAR_TYPES_H
#define VFHSTAR_TYPES_H

#include <boost/cstdint.hpp>
#include <base/Pose.hpp>
#include <vector>
#include <base/Waypoint.hpp>
#include <base/Time.hpp>

namespace vfh_star
{
    struct AngleSampleConf
    {
        AngleSampleConf() : angularSamplingMin(0), angularSamplingMax(0), angularSamplingNominalCount(1), intervalStart(0), intervalWidth(-1) {}
        
        /** Minimum angle between two projected poses */
        double angularSamplingMin;
        
        /** Maximum angle between two projected poses */
        double angularSamplingMax;
        
        /** Nominal number of samples in a given interval */
        int angularSamplingNominalCount;
        
        /** Start angle in robot frame, were the sampling applies */
        double intervalStart;
        
        /** Width of the sampling interval */
        double intervalWidth;
    };
    
    struct TreeSearchConf {
        int maxTreeSize;
        double stepDistance; //! the distance in meters between two steps in the search

        /**
         * A vector containing the different sampling policies around the robot
         * Note, the sampling areas should not overlap
         * */
        std::vector<AngleSampleConf> sampleAreas;
        
        //! the cost discount factor applied on the cost of nodes at depth D + 1 w.r.t. the node at depth D
        double discountFactor; 

        //! the margin distance between the robot and the obstacles
        double obstacleSafetyDistance; 

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
         * Max height of step between two cells in height map,
         * that will caount as traversable
         * */
        double maxStepSize;

        /**
         * Maximum slope of the terrain, the robot is allowed
         * to drive onto.
         * */
        double maxSlop;

        base::Time maxSeekTime;
        
        TreeSearchConf()
            : maxTreeSize(0)
            , stepDistance(0.5)
            , discountFactor(1.0)
            , identityPositionThreshold(-1)
            , identityYawThreshold(-1)
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

