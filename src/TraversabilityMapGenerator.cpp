#include "TraversabilityMapGenerator.h"
#include <Eigen/LU>
#include <iostream>
#include <envire/maps/MLSGrid.hpp>
#include <numeric/PlaneFitting.hpp>

using namespace Eigen;

namespace vfh_star {

TraversabilityMapGenerator::TraversabilityMapGenerator()
{
    boundarySize = 0;
    maxStepSize = 0.2;
    maxSlope = 0.0;
    lastBody2Odo = Affine3d::Identity();
    lastLaser2Odo = Affine3d::Identity();
    lastHeight = 0.0;
    heightToGround = 0.0;
}

void TraversabilityMapGenerator::setHeightToGround(double value)
{
    heightToGround = value;
}

double TraversabilityMapGenerator::getHeightToGround() const
{
    return heightToGround;
}

void TraversabilityMapGenerator::setBoundarySize(double size)
{
    boundarySize = size;
}

void TraversabilityMapGenerator::setMaxStepSize(double size)
{
    maxStepSize = size;
}

void TraversabilityMapGenerator::setMaxSlope(double slope)
{
    maxSlope = slope;
}

bool TraversabilityMapGenerator::getZCorrection(Eigen::Affine3d& body2Odo)
{
    //correct body2Odo z measurement
    Vector2i pInGrid;
    if(!laserGrid.getGridPoint(body2Odo.translation(), pInGrid))
    {
	return false;
    }

    const ElevationEntry entry(laserGrid.getEntry(pInGrid));

    double curHeight;
    
    if(entry.getMeasurementCount())
	curHeight = entry.getMedian();
    else
	curHeight = lastHeight;
    
    Vector3d vecToGround = body2Odo.rotation() * Vector3d(0,0, heightToGround);
    
    body2Odo.translation().z() = curHeight + vecToGround.z();
    
    lastHeight = curHeight;

    return true;
}

void TraversabilityMapGenerator::addPointVector(const std::vector< Vector3d >& rangePoints_odo)
{
    laserGrid.addLaserScan(rangePoints_odo);
}


bool TraversabilityMapGenerator::addLaserScan(const base::samples::LaserScan& ls, const Eigen::Affine3d& body2Odo2, const Eigen::Affine3d& laser2Body)
{
    Eigen::Affine3d body2Odo(body2Odo2);
    //std::cout << "TraversabilityMapGenerator: Got laserScan" << std::endl;
    //std::cout << "body2Odo " << std::endl << body2Odo.matrix() << " laser2Body " << std::endl << laser2Body.matrix() << std::endl;

    moveGridIfRobotNearBoundary(laserGrid, body2Odo.translation());
    
    //correct body2Odo z measurement
    Vector2i pInGrid;
    if(!getZCorrection(body2Odo))
    {
	std::cout << "Odometry position not in Grid" <<std::endl;
	throw std::runtime_error("Odometry position not in Grid");
    }
    
    Affine3d laser2Odo(body2Odo * laser2Body);
    Affine3d  body2LastBody(lastBody2Odo.inverse() * body2Odo);

    double distanceBodyToLastBody = body2LastBody.translation().norm();
    Vector3d Ylaser2Odo = laser2Odo * Vector3d::UnitY() - laser2Odo.translation();
    Ylaser2Odo.normalize();
    Vector3d YlastLaser2Odo = lastLaser2Odo * Vector3d::UnitY() - lastLaser2Odo.translation();
    YlastLaser2Odo.normalize();
    double laserChange = acos(Ylaser2Odo.dot(YlastLaser2Odo));

    
    std::vector<Vector3d> currentLaserPoints = ls.convertScanToPointCloud(laser2Odo);
    
    laser2Map = laser2Odo;
    
    laserGrid.addLaserScan(currentLaserPoints);
    
    //    std::cout << "GridMapSegmenter: Odometry distance : " << distanceBodyToLastBody << " Orientation change is " << orientationChange / M_PI * 180.0 << std::endl;
    if(distanceBodyToLastBody < 0.05 && laserChange < M_PI / 36.0) {
	return false;
    }

    
    lastBody2Odo = body2Odo;
    lastLaser2Odo = laser2Odo;
    return true;
}

bool TraversabilityMapGenerator::moveMapIfRobotNearBoundary(const Eigen::Vector3d& robotPosition_world)
{
    return moveGridIfRobotNearBoundary(laserGrid, robotPosition_world);
}

void TraversabilityMapGenerator::addKnowMap(envire::MLSGrid const *mls, const Affine3d &mls2LaserGrid)
{
    const Eigen::Vector3d gridPos(laserGrid.getPosition());
    const Affine3d laserGrid2mls(mls2LaserGrid.inverse());
    
    for(int x = 0; x < laserGrid.getWidth(); x++)
    {
	for(int y = 0; y < laserGrid.getHeight(); y++)
	{
	    Eigen::Vector3d posLaserGrid;
	    Eigen::Vector2i posLG(x,y);
	    
	    if(!laserGrid.fromGridPoint(posLG, posLaserGrid))
	    {
		std::cout << "WARNING point that should be in the grid is not in grid" <<std::endl;
		continue;
	    }
	    const Eigen::Vector3d posMls = laserGrid2mls * posLaserGrid;

	    size_t mlsX;
	    size_t mlsY;
	    if(mls->toGrid(posMls, mlsX, mlsY))
	    {
		vfh_star::ElevationEntry &entry(laserGrid.getEntry(posLG));

		if(!entry.getMeasurementCount())
		{
		    envire::MLSGrid::const_iterator cellIt = mls->beginCell(mlsX, mlsY);
		    envire::MLSGrid::const_iterator cellEndIt = mls->endCell();
		    
		    for(; cellIt != cellEndIt; cellIt++)
		    {
			entry.addHeightMeasurement(cellIt->mean + mls2LaserGrid.translation().z());
		    }
		}
	    }
	}
    }
}

void TraversabilityMapGenerator::setGridEntriesWindowSize(int window_size)
{
	laserGrid.setEntriesWindowSize(window_size);
	interpolatedGrid.setEntriesWindowSize(window_size);
}

void TraversabilityMapGenerator::setHeightMeasureMethod(int entry_height_conf){
	laserGrid.setHeightMeasureMethod(entry_height_conf);
	interpolatedGrid.setHeightMeasureMethod(entry_height_conf);
}

void TraversabilityMapGenerator::computeNewMap()
{
    //interpolate grid
    smoothElevationGrid(laserGrid, interpolatedGrid);
    
    computeSmoothElevelationGrid(interpolatedGrid, smoothedGrid);
    
    updateTraversabilityGrid(interpolatedGrid, traversabilityGrid);    
}

void TraversabilityMapGenerator::clearMap()
{
    lastBody2Odo = Affine3d::Identity();
    lastLaser2Odo = Affine3d::Identity();
    lastHeight = 0.0;
    laserGrid.clear();
    interpolatedGrid.clear();
    traversabilityGrid.clear();
    smoothedGrid.clear();
}

void TraversabilityMapGenerator::getGridDump(GridDump& gd) const
{
    assert(laserGrid.getHeight() == interpolatedGrid.getHeight() &&
    laserGrid.getHeight() == traversabilityGrid.getHeight() &&
    laserGrid.getWidth() == interpolatedGrid.getWidth() &&
    laserGrid.getWidth() == traversabilityGrid.getWidth() );
    
    for(int x = 0; x < laserGrid.getWidth(); x++) {
	for(int y = 0; y < laserGrid.getHeight(); y++) {
	    if(interpolatedGrid.getEntry(x, y).getMeasurementCount()) {
		gd.height[x*laserGrid.getWidth()+y] = interpolatedGrid.getEntry(x, y).getMedian();
	    } else {
		gd.height[x*laserGrid.getWidth()+y] = std::numeric_limits<double>::infinity();		
	    }
	    
	    gd.max[x*laserGrid.getWidth()+y] = interpolatedGrid.getEntry(x, y).getMaximum();
	    
	    gd.interpolated[x*laserGrid.getWidth()+y] = interpolatedGrid.getEntry(x, y).isInterpolated();
	    
	    gd.traversability[x*laserGrid.getWidth()+y] = traversabilityGrid.getEntry(x, y);
	}
    }
    
    gd.gridPositionX = traversabilityGrid.getGridPosition().x();
    gd.gridPositionY = traversabilityGrid.getGridPosition().y();
    gd.gridPositionZ = traversabilityGrid.getGridPosition().z();
}

bool TraversabilityMapGenerator::moveGridIfRobotNearBoundary(ElevationGrid &grid, const Eigen::Vector3d& robotPosition_world)
{
    Vector3d posInGrid = robotPosition_world - grid.getPosition();
    posInGrid.z() = 0;
    
    double width = grid.getWidth() * grid.getGridResolution() / 2.0;
    double height = grid.getHeight() * grid.getGridResolution() / 2.0;
    
    if(fabs(posInGrid.x()) > (width - boundarySize) || fabs(posInGrid.y()) > (height - boundarySize)) {
    
        if(fabs(posInGrid.x()) > width || fabs(posInGrid.y()) > height)
        {
            //inital case, robot might be out of grid
            posInGrid = Vector3d(0, 0, 0);
        }

        //we assume the robot keeps moving into the same direction
        grid.moveGrid(robotPosition_world + posInGrid * 2.0 / 3.0);

        return true;
    }
    return false;
}

void TraversabilityMapGenerator::updateTraversabilityGrid(const ElevationGrid &elGrid, TraversabilityGrid &trGrid)
{
    trGrid.setGridPosition(elGrid.getGridPosition());
    
    for(int x = 0;x < elGrid.getWidth(); x++) {
	for(int y = 0;y < elGrid.getHeight(); y++) {
	    testNeighbourEntry(Eigen::Vector2i(x,y), elGrid, trGrid);
	}
    }
}

void TraversabilityMapGenerator::testNeighbourEntry(Eigen::Vector2i p, const ElevationGrid &elGrid, TraversabilityGrid &trGrid) {
    if(!elGrid.inGrid(p))
	return;
    
    const ElevationEntry &entry = elGrid.getEntry(p);
    Traversability cl = TRAVERSABLE;
    double curHeight;

    if(!entry.getMeasurementCount() && entry.getMaximum() == -std::numeric_limits< double >::max()) 
    {
	trGrid.getEntry(p) = UNCLASSIFIED;
	return;
    }

    if(!entry.getMeasurementCount())
    {
	curHeight = entry.getMaximum();
	cl = UNKNOWN_OBSTACLE;
    }
    else
    {
	curHeight = entry.getMedian();
    }
    
    int neighbourCnt = 0;

    base::PlaneFitting<double> fitter;
    //center point
    fitter.update(Vector3d(0,0,0));
    double curHeightSmooth = smoothedGrid.getEntry(p);
    
    for(int x = -1; x <= 1; x++) {
	for(int y = -1; y <= 1; y++) {
            //skip onw entry
            if(x == 0 && y == 0)
                continue;

            const int rx = p.x() + x;
	    const int ry = p.y() + y;
	    if(elGrid.inGrid(rx, ry)) {
		const ElevationEntry &neighbourEntry = elGrid.getEntry(rx, ry);
		
		double neighbourHeight;
		
		if(neighbourEntry.getMeasurementCount()) 
		{
		    //use real measurement if available
		    neighbourHeight = neighbourEntry.getMedian();
		}
		else
		{
		    if(neighbourEntry.getMaximum() == -std::numeric_limits<double>::max())
			continue;
		    
		    neighbourHeight = neighbourEntry.getMinimum();
		}
				
		//only make the higher one an obstacle
		if(fabs(neighbourHeight - curHeight) > maxStepSize && curHeight > neighbourHeight) {
		    cl = OBSTACLE;
                    break;
		} 

		fitter.update(base::PlaneFitting<double>::Vector3(x * elGrid.getGridResolution(), y * elGrid.getGridResolution(), curHeightSmooth - smoothedGrid.getEntry(rx, ry)));
		
                neighbourCnt++;
	    }
	}
    }
    
    if((cl != OBSTACLE) && (neighbourCnt > 5))
    {        
        Vector3d fit(fitter.getCoeffs());
        const double divider = sqrt(fit.x() * fit.x() + fit.y() * fit.y() + 1);
        double slope = acos(1 / divider);
                
        if(fabs(slope) > maxSlope)
        {
            cl = OBSTACLE;
        }
    }
    
    trGrid.getEntry(p) = cl;
}

void TraversabilityMapGenerator::markUnknownInRadiusAs(const base::Pose& pose, double radius, Traversability type)
{
    Eigen::Vector2i gridp;
    if(!traversabilityGrid.getGridPoint(pose.position, gridp))
    {
	std::cout << "Pose " << pose.position.transpose() << " Gridpos " << traversabilityGrid.getPosition().transpose() << " Boundary Size " << boundarySize << std::endl;
	throw std::runtime_error("markUnknownInRadiusAsObstacle: Pose out of grid");
    }
    
    int posX = gridp.x();
    int posY = gridp.y();
    
    int radiusGrid = radius / traversabilityGrid.getGridResolution();
    for(int x = -radiusGrid; x < radiusGrid; x++)
    {
	for(int y = -radiusGrid; y < radiusGrid; y++)
	{
	    const double xd = x * traversabilityGrid.getGridResolution();
	    const double yd = y * traversabilityGrid.getGridResolution();
	    double distToRobot = sqrt(xd*xd + yd*yd);

	    //check if outside circle
	    if(distToRobot > radius)
		continue;

	    const int rx = posX + x;
	    const int ry = posY + y;
	    
	    if(!traversabilityGrid.inGrid(rx, ry))
	    {
		std::cout << "x " << x << " y " << y << " posX " << posX << " posY " << posY << " radGrid " << radiusGrid << std::endl;
		throw std::runtime_error("markUnknownInRadiusAsObstacle: Access out of grid");
	    }
	    
	    Traversability &entry(traversabilityGrid.getEntry(rx, ry));
	    
	    if(entry == UNCLASSIFIED || entry == UNKNOWN_OBSTACLE) 
	    {
		entry = type;
		if(type == TRAVERSABLE)
		{
		    laserGrid.getEntry(rx, ry).addHeightMeasurement(laserGrid.getEntry(rx, ry).getMedian());
		}
	    }
	}
    }
}

ConsistencyStats TraversabilityMapGenerator::checkMapConsistencyInArea(const base::Pose& pose, double width, double height)
{
    double heading = pose.orientation.toRotationMatrix().eulerAngles(2,1,0)[0];
    AngleAxisd rot = AngleAxisd(heading, Vector3d::UnitZ());
    
    ConsistencyStats stats;
    
    stats.cellCnt = 0;
    stats.noMeasurementCnt = 0;
    stats.measurementCnt = 0;
    stats.averageCertainty = 0.0;
    
    for(double x = -width / 2.0; x <= (width / 2.0); x += 0.03)
    {
	for(double y = -height / 2.0; y <= (height / 2.0); y += 0.03)
	{
	    Vector2i p_g;
	    Vector3d p_w = pose.position + rot * Vector3d(x, y, 0);
	    
	    if(laserGrid.getGridPoint(p_w, p_g))
	    {
		vfh_star::ElevationEntry &entry(laserGrid.getEntry(p_g));
		stats.cellCnt++;
		
		if(entry.getMeasurementCount())
		{
		    stats.measurementCnt++;
		    //FIXME hard coded
		    stats.averageCertainty += entry.getMeasurementCount() / 50.0;
		}
		else 
		{
		    stats.noMeasurementCnt++;
		}
	    }
	    else 
	    {
		std::cout << "Error point not in grid " << std::endl;
	    }
	}
    }
    
    stats.averageCertainty /= stats.measurementCnt;
    
//     std::cout << "CellCnt " << stats.cellCnt << " Cells with Measurement " << stats.measurementCnt << " Cells without Measurement " << stats.noMeasurementCnt << " average certainty " << stats.averageCertainty << std::endl;    
    return stats;
}


void TraversabilityMapGenerator::markUnknownInRectangeAs(const base::Pose& pose, double width, double height, double forwardOffset, Traversability type)
{
    Vector3d vecToGround = pose.orientation * Vector3d(0,0, heightToGround);

    for(double x = -width / 2.0; x <= (width / 2.0); x += 0.03)
    {
	for(double y = -height / 2.0; y <= (height / 2.0 + forwardOffset); y += 0.03)
	{
	    Vector2i p_g;
	    Vector3d p_w = pose.position + pose.orientation * Vector3d(x, y, 0);
	    
	    if(laserGrid.getGridPoint(p_w, p_g))
	    {
		Traversability &entry(traversabilityGrid.getEntry(p_g));
		if(entry == UNCLASSIFIED || entry == UNKNOWN_OBSTACLE) 
		{
		    entry = type;
		    if(type == TRAVERSABLE)
		    {		
			vfh_star::ElevationEntry &entry(laserGrid.getEntry(p_g));

			if(!entry.getMeasurementCount())
			{
			    entry.addHeightMeasurement(p_w.z() - vecToGround.z());
			}
		    }
		}
	    }
	    else 
	    {
		std::cout << "Error point not in grid " << std::endl;
	    }
	}
    }
}

void TraversabilityMapGenerator::markUnknownInRectangeAsObstacle(const base::Pose& pose, double width, double height, double forwardOffset)
{
    markUnknownInRectangeAs(pose, width, height, forwardOffset, OBSTACLE);
}

void TraversabilityMapGenerator::markUnknownInRectangeAsTraversable(const base::Pose& pose, double width, double height, double forwardOffset)
{
    markUnknownInRectangeAs(pose, width, height, forwardOffset, TRAVERSABLE);
}

void TraversabilityMapGenerator::markUnknownInRadiusAsTraversable(const base::Pose& pose, double radius)
{
    markUnknownInRadiusAs(pose, radius, TRAVERSABLE);
}

void TraversabilityMapGenerator::markUnknownInRadiusAsObstacle(const base::Pose& pose, double radius)
{
    markUnknownInRadiusAs(pose, radius, OBSTACLE);
}

void TraversabilityMapGenerator::doConservativeInterpolation(const vfh_star::ElevationGrid& source, vfh_star::ElevationGrid& target, Eigen::Vector2i p) const {
    if(!source.inGrid(p))
	return;

    target.getEntry(p) = source.getEntry(p);

    //point has a measurement
    if(source.getEntry(p).getMeasurementCount()) {
	return;
    }

    //two possible valid cases for interpolation:
    // XXX    XOX
    // OOO or XOX
    // XXX    XOX

    bool firstFound = false;
    bool secondFound = false;

    for(int x = -1; x <= 1; x++) {
	int rx = p.x() + x;
	int ry = p.y() -1;
	if(source.inGrid(rx, ry)) {
	    firstFound |= source.getEntry(rx, ry).getMeasurementCount();
	}

	ry = p.y() + 1;
	if(source.inGrid(rx, ry)) {
	    secondFound |= source.getEntry(rx, ry).getMeasurementCount();
	}
    }
    
    if(!firstFound || !secondFound) {
	firstFound = false;
	secondFound = false;
	for(int y = -1; y <= 1; y++) {
	    int rx = p.x() - 1;
	    int ry = p.y() + y;
	    if(source.inGrid(rx, ry)) {
		firstFound |= source.getEntry(rx, ry).getMeasurementCount();
	    }

	    ry = p.x() + 1;
	    if(source.inGrid(rx, ry)) {
		secondFound |= source.getEntry(rx, ry).getMeasurementCount();
	    }   
	}
    }

    if(firstFound && secondFound) {
// 	std::cout << "Starting interpolation" << std::endl;
	for(int x = -1; x <= 1; x++) {
	    for(int y = -1; y <= 1; y++) {
		int rx = p.x() + x;
		int ry = p.y() + y;
		if(source.inGrid(rx, ry)) {
		    if(source.getEntry(rx, ry).getMeasurementCount()) {
			target.getEntry(p).addHeightMeasurement(source.getEntry(rx, ry).getMedian());
// 			std::cout << "Adding height " << source.getEntry(rx, ry).getMedian() << std::endl;
		    }
		}
	    }
	}
	target.getEntry(p).setInterpolatedMeasurement(target.getEntry(p).getMedian());
// 	std::cout << "Resulting height " << target.getEntry(p).getMedian() << std::endl;
    }
}

bool TraversabilityMapGenerator::getMeanHeightOfNeighbourhood(const vfh_star::ElevationGrid& grid, Eigen::Vector2i p, double &meanHeight) const
{
    double height = 0;
    int neighbourCnt = 0;
    bool gotValue = false;
    
    for(int x = -1; x <= 1; x++) {
        for(int y = -1; y <= 1; y++) {
            int rx = p.x() + x;
            int ry = p.y() + y;
            if(grid.inGrid(rx, ry)) {
                const ElevationEntry &neighbourEntry = grid.getEntry(rx, ry);
                
                double neighbourHeight;
                
                if(neighbourEntry.getMeasurementCount()) 
                {
                    //use real measurement if available
                    neighbourHeight = neighbourEntry.getMedian();
                }
                else
                {
                    if(neighbourEntry.getMaximum() == -std::numeric_limits<double>::max())
                        continue;
                    
                    neighbourHeight = neighbourEntry.getMinimum();
                }
                
                neighbourCnt++;
                height += neighbourHeight;
                
                gotValue = true;
            }
        }
    }
    
    meanHeight = height / neighbourCnt;
    
    return gotValue;
}


void TraversabilityMapGenerator::computeSmoothElevelationGrid(const vfh_star::ElevationGrid& source, vfh_star::Grid< double, 600, 12 >& target) const
{
    target.setGridPosition(source.getGridPosition());
    
    for(int x = 0;x < source.getWidth(); x++) {
        for(int y = 0;y < source.getHeight(); y++) {
            double mean = std::numeric_limits< double >::quiet_NaN();
            getMeanHeightOfNeighbourhood(source, Eigen::Vector2i(x, y), mean);
            target.getEntry(x, y) = mean;;
        }
    }
}


void TraversabilityMapGenerator::smoothElevationGrid(const ElevationGrid& source, ElevationGrid& target) const
{
    target.setGridPosition(source.getGridPosition());
    
    for(int x = 0;x < source.getWidth(); x++) {
	for(int y = 0;y < source.getHeight(); y++) {
	    doConservativeInterpolation(source, target, Eigen::Vector2i(x, y));
	}
    } 
}




}
