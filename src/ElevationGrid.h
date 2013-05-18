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
	void addHeightMeasurementMeanStd(double measurement, double k_std);
	void addHeightMeasurementMedian(double measurement);
	void setInterpolatedMeasurement(double measurement);
	void setMaximumHeight(double measurement);
	void setMinimumHeight(double measurement);
	
	void setEntryWindowSize(int window_size);
	void setHeightMeasureMethod(int entry_height_conf);

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

	double getMinimum() const {
	    return min;
	}
	
    private:
	std::vector<double> heights;
	bool interpolated;
	double sum;
	int count;
	double median;
	double min;
	double max;
	double mean;
	int entryWindowSize;
	double stDev;

	int entryHeightConf;
};

class ElevationGrid: public Grid<ElevationEntry, 600, 6>
{
    public:
	ElevationGrid();
	
	void addLineBetweenPoints(const Eigen::Vector2i &start_g, double start_val, const Eigen::Vector2i &end_g, double end_val);
	void addLineBetweenPoints(const Eigen::Vector3d &start,const Eigen::Vector3d &end);
	void addLaserScan(const std::vector<Eigen::Vector3d> &laserPoints_world);
	
	void setEntriesWindowSize(int window_size);
	void setHeightMeasureMethod(int entry_height_conf);
	
    private:
};
}

#endif // ELEVATIONGRID_H
