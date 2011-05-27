#include "ElevationGrid.h"
#include "Bresenham.h"

using namespace vfh_star;

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
    const int maxPoints = 50;
    if(min > measurement)
	min = measurement;
    
    if(max < measurement)
	max = measurement;

    int num_points = heights.size();

    if(num_points < 50)
    {
	heights.push_back(measurement);

	sum += measurement;
	count++;
        
	median = sum / count;
    } else {
	count++;
	count = count % 50;
	
	heights[count] = measurement;
	
	sum = 0;
	for(int i = 0; i < num_points; i++)
	    sum += heights[i];

	median = sum / num_points;
    }
}

void ElevationEntry::setMaximumHeight(double measurement)
{
    if(max < measurement)
	max = measurement;
}

void ElevationEntry::setMinimumHeight(double measurement)
{
    if(min > measurement)
	min = measurement;
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

    vfh_star::Bresenham nextSegmentLine(start_g, end_g);
    
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
	    getEntry(p).setMinimumHeight(interpolatedHeight);
	}
    }
}

void ElevationGrid::addLineBetweenPoints(const Eigen::Vector2i &start_g, double start_val, const Eigen::Vector2i &end_g, double end_val)
{
    getEntry(end_g).addHeightMeasurement(end_val);    
    
    const Eigen::Vector2f start_gf(start_g.x(), start_g.y());
    const Eigen::Vector2f end_gf(end_g.x(), end_g.y());

    if(start_g == end_g || (start_gf - end_gf).norm() < 2)
	return;

    vfh_star::Bresenham nextSegmentLine(start_g, end_g);
    
    const Eigen::Vector2f startToEnd = end_gf-start_gf;
    
    const double heightDiff = end_val - start_val;
    
    const double inclination = heightDiff / startToEnd.norm();

    Eigen::Vector2i p;
    
    while(nextSegmentLine.getNextPoint(p)) 
    {
	if(p == start_g || p == end_g)
	    continue;
	
	if(inGrid(p))
	{
	    const Eigen::Vector2f pf(p.x(), p.y());
	    const double interpolatedHeight = start_val + (pf - start_gf).norm() * inclination;
	    getEntry(p).setMaximumHeight(interpolatedHeight);
	    getEntry(p).setMinimumHeight(interpolatedHeight);
	}
    }
}

struct binnedPoint
{
    Eigen::Vector2i gridPos;
    double heightValue;
    bool validNeighbour;
};
    
void ElevationGrid::addLaserScan(const std::vector< Eigen::Vector3d>& laserPoints_world)
{
    Eigen::Vector2i p_g, lastP_g;
        
    bool gotGridPoint = false;
    
    double heightSum = 0;
    int cnt = 0;
    

    
    std::vector< struct binnedPoint > binnedPoints;
    
    bool directNeighbour = false;
    
    //first bin the points
    for(std::vector< Eigen::Vector3d >::const_iterator it = laserPoints_world.begin(); it != laserPoints_world.end(); it++) {	
	if(getGridPoint(*it, p_g))
	{
	    if(gotGridPoint) {
		if(lastP_g == p_g)
		{
		    heightSum += it->z();
		    cnt++;
		}
		else
		{
		    binnedPoint bp;
		    bp.gridPos = lastP_g;
		    bp.heightValue = heightSum / cnt;
		    bp.validNeighbour = directNeighbour;
		    //add binned value
		    binnedPoints.push_back(bp);
		    heightSum = it->z();
		    cnt = 1;
		    gotGridPoint = true;
		    directNeighbour = true;
		}
	    }
	    else
	    {
		directNeighbour = false;
		heightSum = it->z();
		cnt = 1;
		gotGridPoint = true;
	    }
	    lastP_g = p_g;
	}
	else
	{
	    if(gotGridPoint)
	    {
		binnedPoint bp;
		bp.gridPos = lastP_g;
		bp.heightValue = heightSum / cnt;
		bp.validNeighbour = directNeighbour;
		
		//add binned value
		binnedPoints.push_back(bp);
	    }
	    
	    heightSum = 0;
	    cnt = 0;
	    gotGridPoint = false;
	    directNeighbour = false;
	}
    }
    
    
    std::vector< binnedPoint >::const_iterator last_p = binnedPoints.begin();
    getEntry(last_p->gridPos).addHeightMeasurement(last_p->heightValue);
    
    for(std::vector< binnedPoint >::const_iterator it = last_p + 1; it != binnedPoints.end(); it++) {
	if(it->validNeighbour)
	    addLineBetweenPoints(last_p->gridPos, last_p->heightValue, it->gridPos, it->heightValue);
	else
	    getEntry(it->gridPos).addHeightMeasurement(it->heightValue);
	last_p = it;
    }

}
