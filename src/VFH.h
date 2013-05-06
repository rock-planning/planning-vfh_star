#ifndef VFH_H
#define VFH_H

#include <envire/maps/Grid.hpp>
#include <envire/maps/TraversabilityGrid.hpp>
#include <vector>
#include <base/pose.h>
#include "RadialLookUpTable.hpp"

#include "Types.h"
#include <base/angle.h>

namespace vfh_star
{
    enum Traversability {
        UNCLASSIFIED = 0,
        TRAVERSABLE = 1,
        OBSTACLE = 2,
        UNKNOWN_OBSTACLE = 3,
    };
    
    class TerrainStatistic
    {
	public:
	    friend class VFH;
	    TerrainStatistic() {
		statistic.resize(4);
		minDistance.resize(4);
		for(int i = 0; i < 4; i++) 
		{
		    minDistance[i] = std::numeric_limits<double>::max();
		}
		count = 0;
	    }
	    
	    double getMinDistanceToTerrain(const Traversability &terrainType) const
	    {
		return minDistance[terrainType];
	    }

	    int getTerrainCount() const {
		return count;
	    }
	    
	    int getObstacleCount() const {
		return statistic[OBSTACLE];
	    }
	    
	    int getUnknownCount() const {
		return statistic[UNCLASSIFIED];
	    }
	    
	    int getUnknownObstacleCount() const {
		return statistic[UNKNOWN_OBSTACLE];
	    }
	    
	    int getTraversableCount() const {
		return statistic[TRAVERSABLE];
	    }
	    
	private:
	    std::vector<int> statistic;
	    std::vector<double> minDistance;
	    int count;
    };
    
    class VFH
    {
    public:
        VFH();
        virtual std::vector< std::pair<base::Angle, base::Angle> >
            getNextPossibleDirections(const base::Pose& curPose,
                    double obstacleSafetyDist,
                    double robotWidth, VFHDebugData* dd) const;

        const VFHDebugData &getDebugData();	

	/**
	 * Return weather vfh can return possible directions for this position
	 * */
	bool validPosition(const base::Pose& curPose) const;
	
	void setSenseRadius(double radius)
	{
	    senseRadius = radius;
	}
	
	void setNewTraversabilityGrid(const envire::TraversabilityGrid *trGrid);
	
	Traversability getWorstTerrainInRadius(const base::Pose& curPose, double robotWidth) const;
	std::pair<TerrainStatistic, TerrainStatistic> getTerrainStatisticsForRadius(const base::Pose& curPose, double innerRadius, double outerRadius) const;

    private:
        void generateHistogram(std::vector< double > &histogram,
                const base::Pose &curPose,
                double senseRadius, double obstacleSafetyDist, double robotRadius) const;

        void getBinaryHistogram(const std::vector< double > &histogram, std::vector< bool > &binHistogram, double lowThreshold, double highThreshold) const;
	RadialLookUpTable lut;
        const envire::TraversabilityGrid *traversabillityGrid;

        bool debugActive;
        VFHDebugData debugData;
        double senseRadius;
        int narrowThreshold;
        int histogramSize;
        double lowThreshold;
        double highThreshold;
    };

}

#endif // VFH_H
