#ifndef ELEVATIONGRID_H
#define ELEVATIONGRID_H

#include "Grid.h"
#include <Eigen/Core>
#include <vector>
#include <boost/cstdint.hpp>

namespace vfh_star
{
class ElevationEntry {
    public:
	ElevationEntry();
	void addHeightMeasurement(double measurement, boost::uint64_t age);
	void setMaximumHeight(double measurement);
	void setMinimumHeight(double measurement);
        void setInterpolatedMeasurement(bool interpolated);
        void setHeight(double value, boost::uint64_t age);
	
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
	std::vector<bool> hasSample;
	bool interpolated;
	double sum;
	double median;
	double min;
	double max;
	double mean;
	int entryWindowSize;
	double stDev;

	int entryHeightConf;

        boost::uint64_t currentAge;

        void updateHeightWindow(double measurement, uint64_t age);
        double computeHeightVariance() const;
};

class ElevationGrid: public Grid<ElevationEntry, GRIDSIZE, GRIDRESOLUTION>
{
    public:
	ElevationGrid();
	
	void addLineBetweenPoints(const Eigen::Vector2i &start_g, double start_val, const Eigen::Vector2i &end_g, double end_val);
	void addLineBetweenPoints(const Eigen::Vector3d &start,const Eigen::Vector3d &end);
	void addLaserScan(const std::vector<Eigen::Vector3d> &laserPoints_world);
	
	void setEntriesWindowSize(int window_size);
	void setHeightMeasureMethod(int entry_height_conf);

        boost::uint64_t getCurrentAge() const;
	
    private:
        boost::uint64_t currentAge;
};
}

#endif // ELEVATIONGRID_H
