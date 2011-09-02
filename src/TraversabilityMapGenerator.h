#ifndef GRIDMAPSEGMENTER_H
#define GRIDMAPSEGMENTER_H

#include <Eigen/Geometry>
#include "ElevationGrid.h"
#include "TraversabilityGrid.h"
#include "DebugTypes.h"
#include <base/samples/laser_scan.h>
#include <base/pose.h>
#include <stdint.h>

namespace vfh_star {

struct ConsistencyStats
{
    int cellCnt;
    int noMeasurementCnt;
    int measurementCnt;
    double averageCertainty;
};
    
class TraversabilityMapGenerator
{
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	TraversabilityMapGenerator();
		
	bool addLaserScan(const base::samples::LaserScan& ls, const Eigen::Affine3d& body2Odo, const Eigen::Affine3d& laser2Body);
	
	void getGridDump(GridDump &gd) const;
	
	const TraversabilityGrid &getTraversabilityMap() const 
	{
	    return traversabilityGrid;
	};

	/**
	 * Deletes all information in the maps
	 **/
	void clearMap();
	
	/**
	 * Manual trigger for map generation
	 * */
	void computeNewMap();
	
	/**
	 * The map is moved if the robot position
	 * is inside the outer boundary of the map.
	 * */
	void setBoundarySize(double size);

	void setMaxStepSize(double size);
	
	void markUnknownInRadiusAsObstacle(const base::Pose& pose, double radius);
	void markUnknownInRadiusAsTraversable(const base::Pose& pose, double radius);
	void markUnknownInRectangeAsTraversable(const base::Pose& pose, double width, double height, double forwardOffset);
	void markUnknownInRectangeAsObstacle(const base::Pose& pose, double width, double height, double forwardOffset);
	
	ConsistencyStats checkMapConsistencyInArea(const base::Pose& pose, double width, double height);
	
    private:
	
	void markUnknownInRadiusAs(const base::Pose& pose, double radius, vfh_star::Traversability type);
	void markUnknownInRectangeAs(const base::Pose& pose, double width, double height, double forwardOffset, Traversability type);

	void filterLaserScan(std::vector< Eigen::Vector3d >& result,const base::samples::LaserScan& ls, const Eigen::Affine3d& filterFrame, const Eigen::Affine3d& resultFrame, const std::vector< Eigen::AlignedBox< double, 3 > >& maskedAreas);
	
	/**
	* This function updates the TraversabilityGrid in respect to the given
	* ElevationGrid.
	**/
	void updateTraversabilityGrid(const ElevationGrid &elGrid, TraversabilityGrid &trGrid);
	
	/**
	* Applys the Conservative Interpolation on the whole grid
	**/
	void smoothElevationGrid(const ElevationGrid &source, ElevationGrid &target);
	
	/**
	* The conservative interpolation fills holes in the Elevation grid, if the 
	* sourounding cells of a hole are known.
	**/
	void doConservativeInterpolation(const ElevationGrid& source, ElevationGrid& target, Eigen::Vector2i p);
	
	/**
	* This function calculates the slope to the sourounding cells of p
	* and sets the traversibillity of the cell p in the TraversabilityGrid
	**/
	void testNeighbourEntry(Eigen::Vector2i p, const ElevationGrid &elGrid, TraversabilityGrid &trGrid);

	/**
	* This function test if the robot is near the outer bound of
	* the grid and thus the grid need to be moved
	**/
	bool moveGridIfRobotNearBoundary(ElevationGrid& grid, const Eigen::Vector3d& robotPosition_world);

	ElevationGrid laserGrid;
	ElevationGrid interpolatedGrid;
	TraversabilityGrid traversabilityGrid;
	Eigen::Affine3d lastBody2Odo;
	Eigen::Affine3d lastLaser2Odo;
	double boundarySize;
	double maxStepSize;
	double lastHeight;
};

}

#endif // GRIDMAPSEGMENTER_H
