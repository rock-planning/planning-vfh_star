#ifndef ELEVATIONGRID_H
#define ELEVATIONGRID_H

#include "Grid.h"
#include <Eigen/Core>
#include <vector>

namespace vfh_star
{
class ElevationEntry {
    public:
	ElevationEntry();
	void addHeightMeasurement(double measurement);
	void setInterpolatedMeasurement(double measurement);
	void setMaximumHeight(double measurement);
	
	int getMeasurementCount() const {
	    return heights.size();
	}

	double getMedian() const {
	    return median;
	};
	
	bool isInterpolated() const {
	    return interpolated;
	}
	
	double getMaximum() const {
	    return max;
	}
	
    private:
	std::vector<double> heights;
	bool interpolated;
	double sum;
	int count;
	double median;
	double min;
	double max;
};

class ElevationGrid: public Grid<ElevationEntry, 600, 6>
{
    public:
	ElevationGrid();
	
	void addLineBetweenPoints(const Eigen::Vector3d &start,const Eigen::Vector3d &end);
	void addLaserScan(const std::vector<Eigen::Vector3d> &laserPoints_world);
	
	
    private:
};
}

#endif // ELEVATIONGRID_H
