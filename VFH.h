#ifndef VFH_H
#define VFH_H

#include <envire/maps/Grid.hpp>
#include "TraversabilityGrid.h"
#include <vector>
#include <base/pose.h>

namespace vfh_star
{

class VFHDebugData
{
    public:
	base::Pose pose;
	std::vector< bool > histogram;
	double senseRadius;
	double obstacleSafetyDist;
	double robotWidth;
};

class VFH
{
    public:
	VFH(const envire::Grid<Traversability> *trGrid);
	virtual std::vector< std::pair<double, double> > getNextPossibleDirections(const base::Pose& curPose, const double &obstacleSafetyDist, const double &robotWidth, VFHDebugData* dd) const;
	
	const VFHDebugData &getDebugData();	
    private:
	void generateHistogram(std::vector< double > &histogram, const base::Pose &curPose, double senseRadius, double obstacleSafetyDist, double robotRadius) const;
	
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
