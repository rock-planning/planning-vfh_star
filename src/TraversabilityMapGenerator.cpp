#include "TraversabilityMapGenerator.h"
#include <Eigen/LU>
#include <iostream>
 
using namespace Eigen;

namespace vfh_star {

TraversabilityMapGenerator::TraversabilityMapGenerator()
{
    boundarySize = 0;
    maxStepSize = 0.2;
    lastBody2Odo = Transform3d::Identity();
    lastLaser2Odo = Transform3d::Identity();
}

void TraversabilityMapGenerator::setBoundarySize(double size)
{
    boundarySize = size;
}

void TraversabilityMapGenerator::setMaxStepSize(double size)
{
    maxStepSize = size;
}

bool TraversabilityMapGenerator::addLaserScan(const base::samples::LaserScan& ls, const Eigen::Affine3d& body2Odo, const Eigen::Affine3d& laser2Body)
{
    static double lastHeight = 0.0;
    Eigen::Transform3d body2Odo(body2Odo2);
//     std::cout << "TraversabilityMapGenerator: Got laserScan" << std::endl;

    moveGridIfRobotNearBoundary(laserGrid, body2Odo.translation());
    
    //correct body2Odo z measurement
    Vector2i pInGrid;
    if(!laserGrid.getGridPoint(body2Odo.translation(), pInGrid))
    {
	std::cout << "Odometry position not in Grid" <<std::endl;
	throw std::runtime_error("Odometry position not in Grid");
    }
    
    const ElevationEntry entry(laserGrid.getEntry(pInGrid));

    double curHeight;
    
    if(entry.getMeasurementCount())
	curHeight = entry.getMedian();
    else
	curHeight = lastHeight;
    
    Vector3d vecToGround = body2Odo.rotation() * Vector3d(0,0, 0.18);
    
     body2Odo.translation().z() = curHeight + vecToGround.z();
    
    Affine3d laser2Odo(body2Odo * laser2Body);
    Affine3d  body2LastBody(lastBody2Odo.inverse() * body2Odo);

    double distanceBodyToLastBody = body2LastBody.translation().norm();
    Vector3d Ylaser2Odo = laser2Odo * Vector3d::UnitY() - laser2Odo.translation();
    Ylaser2Odo.normalize();
    Vector3d YlastLaser2Odo = lastLaser2Odo * Vector3d::UnitY() - lastLaser2Odo.translation();
    YlastLaser2Odo.normalize();
    double laserChange = acos(Ylaser2Odo.dot(YlastLaser2Odo));


    lastHeight = curHeight;
    
    //filter out wheels
    AlignedBox<double, 3> leftWheel(Vector3d(0.20, -0.25, -0.2), Vector3d(0.4, 0.25, 0.5));
    AlignedBox<double, 3> rightWheel(Vector3d(-0.4, -0.25, -0.2), Vector3d(-0.20, 0.25, 0.5));
    
    std::vector<AlignedBox<double, 3> > maskedAreas;
    maskedAreas.push_back(leftWheel);
    maskedAreas.push_back(rightWheel);
    
    std::vector<Vector3d> currentLaserPoints;
    filterLaserScan(currentLaserPoints, ls, laser2Body, laser2Odo, maskedAreas);
    
    laserGrid.addLaserScan(currentLaserPoints);
    
    //    std::cout << "GridMapSegmenter: Odometry distance : " << distanceBodyToLastBody << " Orientation change is " << orientationChange / M_PI * 180.0 << std::endl;
    if(distanceBodyToLastBody < 0.05 && laserChange < M_PI / 36.0) {
	return false;
    }

    
    lastBody2Odo = body2Odo;
    lastLaser2Odo = laser2Odo;
    return true;
}

void TraversabilityMapGenerator::filterLaserScan(std::vector< Eigen::Vector3d>& result,const base::samples::LaserScan& ls, const Eigen::Transform3d& filterFrame, const Eigen::Transform3d& resultFrame, const std::vector<AlignedBox<double, 3> >& maskedAreas)
{
    std::vector<Eigen::Vector3d> pointCloud;
    std::vector<bool> maskedPoints;

    int lastRange = ls.ranges.at(0);
    const int nrPoints = ls.ranges.size();

    maskedPoints.resize(nrPoints);
    
    result.clear();

    for(unsigned int i = 0; i < maskedPoints.size(); i++) {
	maskedPoints[i] = false;
    }
    
    for(unsigned int i = 0; i < ls.ranges.size(); i++) {
	//this is a filter for false readings that do occur if one scannes over edgeds of objects
	bool isMasked = abs(lastRange - ls.ranges[i]) > 30 && ls.ranges[i] < 1500;

	lastRange = ls.ranges[i];

	//convert reading to cartesian coordinates
	Vector3d curPoint;
	if(!ls.getPointFromScanBeam(i, curPoint))
	    continue;
	
	//transform into filter frame
	curPoint = filterFrame * curPoint;

	//check for intersection with masked areas
	for(std::vector<AlignedBox<double, 3> >::const_iterator it = maskedAreas.begin(); it != maskedAreas.end();it++)
	{
	    if(it->contains(curPoint))
	    {
		isMasked = true;
		break;
	    }
	}

	//second filter for ghost readings
	if(isMasked) {
	    //mask previous 5 and following 5 points
	    for(int j = -5; j < 5; j++)
	    {
		if((i +j < 0) || (i+j > nrPoints))
		{
		    continue;
		}
		maskedPoints[i + j] = true;
	    }
	}
    }

    //create result from all valid readings
    for(unsigned int i = 0; i < ls.ranges.size(); i++) {
	if(maskedPoints[i]) {
	    continue;
	}
	
	Eigen::Vector3d point;
	if(ls.getPointFromScanBeam(i, point)) {
	    point = resultFrame * point;
	    result.push_back(point);
	}
    }    
}

void TraversabilityMapGenerator::computeNewMap()
{
    //interpolate grid
    smoothElevationGrid(laserGrid, interpolatedGrid);
    
    updateTraversabilityGrid(interpolatedGrid, traversabilityGrid);    
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
    

    for(int x = -1; x <= 1; x++) {
	for(int y = -1; y <= 1; y++) {
	    int rx = p.x() + x;
	    int ry = p.y() + y;
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
		
		//TODO correct formula
		if(fabs(neighbourHeight - curHeight) > maxStepSize) {
		    cl = OBSTACLE;
		} 
	    }
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
    double heading = pose.orientation.toRotationMatrix().eulerAngles(2,1,0)[0];
    AngleAxisd rot = AngleAxisd(heading, Vector3d::UnitZ());
    
    for(double x = -width / 2.0; x <= (width / 2.0); x += 0.03)
    {
	for(double y = -height / 2.0; y <= (height / 2.0 + forwardOffset); y += 0.03)
	{
	    Vector2i p_g;
	    Vector3d p_w = pose.position + rot * Vector3d(x, y, 0);
	    
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
			    entry.addHeightMeasurement(0);
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

void TraversabilityMapGenerator::doConservativeInterpolation(const ElevationGrid& source, ElevationGrid& target, Eigen::Vector2i p) {
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


void TraversabilityMapGenerator::smoothElevationGrid(const ElevationGrid& source, ElevationGrid& target)
{
    target.setGridPosition(source.getGridPosition());
    
    for(int x = 0;x < source.getWidth(); x++) {
	for(int y = 0;y < source.getHeight(); y++) {
	    doConservativeInterpolation(source, target, Eigen::Vector2i(x, y));
	}
    } 
}




}
