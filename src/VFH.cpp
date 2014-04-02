#include "VFH.h"
#include <iomanip>

using namespace Eigen;
namespace vfh_star
{

VFH::VFH() : angularResolution(2*M_PI / config.histogramSize)
{
}

void VFH::setConfig(const VFHConf& conf)
{
    config = conf;
    angularResolution = 2*M_PI / config.histogramSize;
}


void VFH::addDir(std::vector<base::AngleSegment> &drivableDirections, int start, int end) const
{
//     std::cout << "adding area " << start << " end " << end << " is narrow " << (abs(end-start) < narrowThreshold) <<  std::endl;
    
    int gapSize = end-start;
    
    if(gapSize < 0)
	gapSize += config.histogramSize;
	
    if(gapSize < config.narrowThreshold) {
	double middle = start + (gapSize / 2.0);
	if(middle > config.histogramSize)
	    middle -= config.histogramSize;
	
	const base::Angle dir = base::Angle::fromRad((middle * angularResolution));
	drivableDirections.push_back(base::AngleSegment(dir, angularResolution));
    } else {
	const base::Angle left = base::Angle::fromRad(start * angularResolution);
        double width = (gapSize ) * angularResolution;
	drivableDirections.push_back(base::AngleSegment(left, width));
    }    
}

void VFH::setNewTraversabilityGrid(const envire::TraversabilityGrid* trGrid)
{
    traversabillityGrid = trGrid;
    
    assert(traversabillityGrid->getScaleX() == traversabillityGrid->getScaleY());
    
    //precompute distances
    lut.recompute(traversabillityGrid->getScaleX(), 5.0);

    gridWidthHalf = traversabillityGrid->getWidth() / 2.0 * traversabillityGrid->getScaleX();
    gridHeightHalf = traversabillityGrid->getHeight() / 2.0 * traversabillityGrid->getScaleY();
    
    const std::vector<envire::TraversabilityClass> &trClasses(traversabillityGrid->getTraversabilityClasses());
    obstacleLookup.clear();
    obstacleLookup.reserve(trClasses.size());
    for(std::vector<envire::TraversabilityClass>::const_iterator it = trClasses.begin(); it != trClasses.end();it++)
    {
        obstacleLookup.push_back(!it->isTraversable());
    }
}

const envire::TraversabilityGrid* VFH::getTraversabilityGrid() const
{
    return traversabillityGrid;
}

std::vector<base::AngleSegment> VFH::getNextPossibleDirections(const base::Pose& curPose) const
{
    std::vector<base::AngleSegment> drivableDirections;
    std::vector<double> histogram;
    std::vector<bool> bHistogram;

    //4 degree steps
    histogram.resize(config.histogramSize);

    generateHistogram(histogram, curPose);

    //we ignore one obstacle
    getBinaryHistogram(histogram, bHistogram);

    
    int start = -1;
    int end = -1;
    int firstEnd = bHistogram.size();
    for(unsigned int i = 0; i < bHistogram.size(); i++) 
    {
	if(bHistogram[i] && start < 0)   
	{	    
	    start = i;
	}
	
	if(!bHistogram[i] && start >= 0)
	{
	    if(start == 0)
	    {
		firstEnd = i;
		start = -1;
		continue;
	    }
	    
	    end = i;
	    
	    addDir(drivableDirections, start, end);
	    start = -1;
	}   
    }
    
    if(start >= 0)
    {
	addDir(drivableDirections, start, firstEnd);
    }
    else 
    {
	if(firstEnd != static_cast<signed int>(bHistogram.size()))
	{
	    addDir(drivableDirections, 0, firstEnd);
	}
    }
    
    {
    }
    
    return drivableDirections;
}

double normalize(double ang)
{
    if(ang < 0)
	return ang + 2*M_PI;
    
    if(ang > 2*M_PI)
	return ang - 2*M_PI;
    
    return ang;
}

bool VFH::validPosition(const base::Pose& curPose) const
{
    //curPose is in map coordinates, as we converted it at the beginning
    //of the planning
    const Vector3d curPos(curPose.position.x(), curPose.position.y(), 0);     
    double distanceToCenter = curPos.norm();
    
    return !(gridHeightHalf  - (distanceToCenter + config.obstacleSenseRadius) < 0) && !(gridWidthHalf - (distanceToCenter + config.obstacleSenseRadius) < 0);    
}

void VFH::generateHistogram(std::vector< double >& histogram, const base::Pose& curPose) const
{
    const int nrDirs = histogram.size();
    
    const double a = 2.0;
    const double b = 1.0/ (config.obstacleSenseRadius * config.obstacleSenseRadius);

    const double radius = config.robotWidth / 2.0 + config.obstacleSafetyDistance;
    
    std::vector<double> &dirs(histogram);    
    
    //calculate robot pos in grid coordinates
    size_t robotX, robotY;    
    assert(traversabillityGrid->toGrid(curPose.position.x(), curPose.position.y(), robotX, robotY));
    
    const envire::TraversabilityGrid::ArrayType &gridData = traversabillityGrid->getGridData();    
    const int senseSize = config.obstacleSenseRadius / traversabillityGrid->getScaleX();

//     std::cout << "senseSize " << senseSize << std::endl;
    
    //Debug code for printing local obstacle map  

//     std::cout <<  std::endl;
// 
//     for(int y = senseSize; y >= -senseSize; y--)
//     {
// 	std::cout << std::setw(4) << y;
// 	for(int x = -senseSize; x <= senseSize; x++)
// 	{
// 	    int rx = robotX + x;
// 	    int ry = robotY + y;
// 	    
// 	    if(!traversabillityGrid->inGrid(rx, ry))
// 		continue;
// 
// 	    //go safe, if we do not know anything about something, it is an obstacle
// 	    if(obstacleLookup[gridData[ry][rx]])
//                 std::cout <<  "O";		
//             else
//                 std::cout <<  "T";
// 		
// 	}
// 	std::cout <<  std::endl;
//     }
    
    //walk over area of grid within of circle with radius config.obstacleSenseRadius around the robot
    for(int y = -senseSize; y <= senseSize; y++)
    {
	for(int x = -senseSize; x <= senseSize; x++)
	{
	    double distToRobot = lut.getDistance(x, y);
	    
	    //check if outside circle
	    if(distToRobot > config.obstacleSenseRadius)
		continue;
	  
	    int rx = robotX + x;
	    int ry = robotY + y;
	    
// 	    std::cout << "rx " << rx << " ry " << ry << std::endl;

            bool inGrid = traversabillityGrid->inGrid(rx, ry);
            //go safe, if we do not know anything about something, it is an obstacle
            if(!inGrid || obstacleLookup[gridData[ry][rx]])
	    {
		double angleToObstace = lut.getAngle(x, y); // atan2(y, x);
		
		//convert to ENU
// 		angleToObstace -= M_PI / 2.0;
		
		//move to range 0 to 2*M_PI
		angleToObstace = normalize(angleToObstace);
		
		//calculate magnitude m = c*(a-b*d²)
		//ci is 1
		double magnitude = a - b * distToRobot*distToRobot;

// 		std::cout << "Magnitude is " << magnitude << std::endl;
		
		//in case we are allready hit the obstacle, we set distToRobot
		//to (robotWidth + obstacleSafetyDist) which results in a 90 degree masking
		//of the area where the collided obstable is
		if((radius) > distToRobot)
		    distToRobot = (radius);
		
		//boundary of obstacle including robot width and safety distance
		double y_t = asin(radius / distToRobot);
		
		//add to histogramm
		int s = (angleToObstace - y_t) / angularResolution;
		int e = (angleToObstace + y_t) / angularResolution;
		for(int a = s; a <= e; a++) {
		    int ac = a;
		    if(ac < 0)
			ac += nrDirs;
		    
		    if(ac >= nrDirs)
			ac -=nrDirs;
		    
		    dirs[ac] += magnitude;
		}
	    }
	}
    }
}


void VFH::getBinaryHistogram(const std::vector< double >& histogram, std::vector< bool >& binHistogram) const
{
    binHistogram.clear();
    binHistogram.reserve(histogram.size());
    for(std::vector<double>::const_iterator it = histogram.begin(); it != histogram.end(); it++) {
	bool drivable = false;
	
	if(*it <= config.lowThreshold)
	    drivable = true;
	
	//FIXME we are missing out the last binary histogramm
	
	binHistogram.push_back(drivable);
    }
}
}
