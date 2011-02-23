#ifndef VFH_H
#define VFH_H

#include <envire/maps/Grid.hpp>
#include "TraversabilityGrid.h"
#include <vector>
#include <base/pose.h>

#include <vfh_star/Types.h>

namespace vfh_star
{

    class VFH
    {
    public:
        VFH(const envire::Grid<Traversability> *trGrid);
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
