#ifndef VFH_H
#define VFH_H

#include <envire/maps/Grid.hpp>
#include "TraversabilityGrid.h"
#include <vector>
#include <base/pose.h>

#include "Types.h"

namespace vfh_star
{
    class TerrainStatistic
    {
	public:
	    friend class VFH;
	    TerrainStatistic() {
		statistic.resize(4);
		count = 0;
	    }
	    
	    int getTerrainCount() const {
		return count;
	    };
	    
	    int getObstacleCount() const {
		return statistic[OBSTACLE];
	    };
	    
	    int getUnknownCount() const {
		return statistic[UNCLASSIFIED];
	    };
	    
	    int getUnknownObstacleCount() const {
		return statistic[UNKNOWN_OBSTACLE];
	    };
	    
	    int getTraversableCount() const {
		return statistic[TRAVERSABLE];
	    };
	    
	private:
	    std::vector<int> statistic;
	    int count;
    };
    
    class VFH
    {
    public:
        VFH();
        virtual std::vector< std::pair<double, double> >
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
	
	void setNewTraversabilityGrid(const envire::Grid<Traversability> *trGrid);
	
	Traversability getWorstTerrainInRadius(const base::Pose& curPose, double robotWidth) const;
	std::pair<TerrainStatistic, TerrainStatistic> getTerrainStatisticsForRadius(const base::Pose& curPose, double innerRadius, double outerRadius) const;

    private:
        void generateHistogram(std::vector< double > &histogram,
                const base::Pose &curPose,
                double senseRadius, double obstacleSafetyDist, double robotRadius) const;

        void getBinaryHistogram(const std::vector< double > &histogram, std::vector< bool > &binHistogram, double lowThreshold, double highThreshold) const;

        const envire::Grid<Traversability> *traversabillityGrid;

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
