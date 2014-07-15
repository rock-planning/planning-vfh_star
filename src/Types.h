#ifndef VFHSTAR_TYPES_H
#define VFHSTAR_TYPES_H

#include <boost/cstdint.hpp>
#include <base/Pose.hpp>
#include <vector>
#include <base/Waypoint.hpp>
#include <base/Time.hpp>

namespace vfh_star
{
    class DebugNode
    {
    public:
        DebugNode(int creationOrder, base::Pose pose) : creationOrder(creationOrder), expansionOrder(-1), pose(pose), cost(0), isValid(true), wasRemoved(false), parent(0) {}
        DebugNode() : creationOrder(-1), expansionOrder(-1), cost(0), isValid(true), wasRemoved(false), parent(0) {}
        
        //order in which the tree nodes were created (sampled)
        int creationOrder;
        //oder in which the tree was searched
        int expansionOrder;
        base::Pose pose;
        ///cost from start to this node
        float cost;
        
        bool isValid;
        bool wasRemoved;
        
        int parent;
        std::vector<int> childs;
        
        bool wasExpanded() const
        {
            return expansionOrder > 0;
        }
        
    };

    class DebugTree
    {
    public:
        DebugTree(): startNode(-1), finalNode(-1) {}; 
        
        std::vector<DebugNode > nodes;
        int startNode;
        int finalNode;
        
        bool hasFinalNode() const
        {
            return finalNode > 0;
        }
        
        base::Pose treePos;
    };
    
    class Tree;
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
        ///maximum number of expanded nodes
        int maxTreeSize;
        
        //! the distance in meters between two steps in the search
        double stepDistance; 

        /**
         * A vector containing the different sampling policies around the robot
         * Note, the sampling areas should not overlap
         * */
        std::vector<AngleSampleConf> sampleAreas;
        
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

        base::Time maxSeekTime;
        
        TreeSearchConf()
            : maxTreeSize(0)
            , stepDistance(0.5)
            , discountFactor(1.0)
            , identityPositionThreshold(-1)
            , identityYawThreshold(-1)
    {
        sampleAreas.push_back(AngleSampleConf());
    };
                
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
                    obstacleSenseRadius(0.0), 
                    narrowThreshold(10), 
                    lowThreshold(6.0),
                    histogramSize(90)
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
        
        /**
         * If the historgram contains a 'slot' of directions
         * which is narrower than this threshhold, it is considered
         * narrow. In this case only one drive direction (the middle of 
         * the slot) is generatred instead of an interval of drive directions
         * */
        int narrowThreshold;
        
        /**
         * Any part of the histogram, were the magnitude value
         * is smaller than this is considered drivable
         * */
        double lowThreshold;

        /**
         * Number of directions covered by the histogram
         * */
        int histogramSize;
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

    struct VFHStarDebugData {
        std::vector<VFHDebugData> steps;
        std::vector<base::Waypoint> generatedTrajectory;
        base::Vector3d horizonOrigin;
        base::Vector3d horizonVector;   
    };

    struct HorizonPlannerDebugData {
        base::Vector3d horizonOrigin;
        base::Vector3d horizonVector;   
    };
}

#endif

