#include "ElevationGrid.h"
#include "Bresenham.h"

ElevationEntry::ElevationEntry()
{
    min = std::numeric_limits< double >::max();
    max = -std::numeric_limits< double >::max();
 
    interpolated = false;
    sum = 0;
    count = 0;
    median = 0;
}

void ElevationEntry::addHeightMeasurement(double measurement)
{
    if(min > measurement)
	min = measurement;
    
    if(max < measurement)
	max = measurement;
    
    heights.push_back(measurement);

    sum += measurement;
    count++;
        
    median = sum / count;
}

void ElevationEntry::setMaximumHeight(double measurement)
{
    if(max < measurement)
	max = measurement;
}

void ElevationEntry::setInterpolatedMeasurement(double measurement)
{
    interpolated = true;
    median = measurement;
}

ElevationGrid::ElevationGrid()
{
   
}

void ElevationGrid::addLineBetweenPoints(const Eigen::Vector3d &start, const Eigen::Vector3d &end)
{
    Eigen::Vector2i start_g;
    Eigen::Vector2i end_g;    
    
    bool startInGrid = getGridPoint(start, start_g);
    bool endInGrid = getGridPoint(end, end_g); 
    if(endInGrid)
	getEntry(end_g).addHeightMeasurement(end.z());
    
    const Eigen::Vector2f start_gf(start_g.x(), start_g.y());
    const Eigen::Vector2f end_gf(end_g.x(), end_g.y());

    if((!startInGrid && !endInGrid) || start_g == end_g || (start_gf - end_gf).norm() < 2)
	return;

    visual_servoing::Bresenham nextSegmentLine(start_g, end_g);
    
    const Eigen::Vector2f startToEnd = end_gf-start_gf;
    
    const double heightDiff = end.z() - start.z();
    
    const double inclination = heightDiff / startToEnd.norm();

    Eigen::Vector2i p;
    
    while(nextSegmentLine.getNextPoint(p)) 
    {
	if(p == start_g || p == end_g)
	    continue;
	
	if(inGrid(p))
	{
	    const Eigen::Vector2f pf(p.x(), p.y());
	    const double interpolatedHeight = start.z() + (pf - start_gf).norm() * inclination;
	    getEntry(p).setMaximumHeight(interpolatedHeight);	    
	}
    }
}

void ElevationGrid::addLaserScan(const std::vector< Eigen::Vector3d>& laserPoints_world)
{
    Eigen::Vector2i p_g;
    
    std::vector< Eigen::Vector3d >::const_iterator last_p = laserPoints_world.begin();
    if(getGridPoint(*last_p, p_g))
	getEntry(p_g).addHeightMeasurement(last_p->z());
    
    for(std::vector< Eigen::Vector3d >::const_iterator it = last_p + 1; it != laserPoints_world.end(); it++) {

	addLineBetweenPoints(*last_p, *it);	
	last_p = it;
    }

}
