#include "ElevationGrid.h"
#include "Bresenham.h"

#include <algorithm>
#include <math.h>
#include <base/Float.hpp>
using namespace vfh_star;

ElevationEntry::ElevationEntry()
{
    min = std::numeric_limits< double >::max();
    max = -std::numeric_limits< double >::max();
    currentAge = 0;
 
    interpolated = false;

    sum = base::unset<double>();
    median = 0;
    stDev = 0;
    mean = 0;
    setEntryWindowSize(50);

    entryHeightConf = 0;
}

void ElevationEntry::setInterpolatedMeasurement(bool interpolated)
{
    this->interpolated = interpolated;
}

void ElevationEntry::setHeight(double value, boost::uint64_t age)
{
    std::fill(hasSample.begin(), hasSample.end(), false);
    heights.clear();
    addHeightMeasurement(value, age);
}

void ElevationEntry::addHeightMeasurement(double measurement, boost::uint64_t age)
{
    //median is the attribute that is finally used as elevation
    if(min > measurement)
	min = measurement;
    
    if(max < measurement)
	max = measurement;

    updateHeightWindow(measurement, age);

    int count = heights.size();
    mean = 0;
    std::vector<double> aux_heights = heights;
    for(int i = 0; i < count; i++)
        mean += heights[i];
    mean = mean / count;
    stDev = sqrt(computeHeightVariance());

    std::nth_element(
            aux_heights.begin(),
            aux_heights.begin()+(count/2),
            aux_heights.end());
    median = aux_heights[count/2];
}

void ElevationEntry::updateHeightWindow(double measurement, uint64_t age)
{
    boost::uint64_t time_shift = age - currentAge;
    if (time_shift > entryWindowSize)
    {
        heights.clear();
        hasSample.clear();
    }
    else
    {
        int count = heights.size();
        for (boost::uint64_t i = 0; count && (i < time_shift); ++i)
        {
            if (hasSample[i])
                count--;
        }

        heights.erase(heights.begin(), heights.end() - count); 
        hasSample.erase(hasSample.begin(), hasSample.begin() + time_shift);
    }
    hasSample.resize(entryWindowSize, false);
    heights.push_back(measurement);
    hasSample.back() = true;
    currentAge = age;

    sum = base::unset<double>();
    median = base::unset<double>();
    stDev = base::unset<double>();
    mean = base::unset<double>();
}

double ElevationEntry::computeHeightVariance() const
{
    // Online algorithm at
    // http://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
    int count = 0;
    double mean = 0;
    double mean2 = 0;
    for(int i = 0; i < heights.size(); i++)
    {
        double h = heights[i];

        count++;
        double delta = h - mean;
        mean += delta / count;
        // NOTE: mean below is NOT the same than mean above, so
        // the next line is NOT delta * delta !!!
        mean2 += delta * (h - mean);
    }
    return mean2 / (count - 1);
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

void ElevationEntry::setEntryWindowSize(int window_size)
{
    entryWindowSize = window_size;
    hasSample.clear();
    hasSample.resize(entryWindowSize, false);
    heights.clear();
}

void ElevationEntry::setHeightMeasureMethod(int entry_height_conf){
    entryHeightConf = entry_height_conf;
}

ElevationGrid::ElevationGrid()
    : currentAge(0)
{
   
}

void ElevationGrid::addLineBetweenPoints(const Eigen::Vector3d &start, const Eigen::Vector3d &end)
{
    Eigen::Vector2i start_g;
    Eigen::Vector2i end_g;
    
    bool startInGrid = getGridPoint(start, start_g);
    bool endInGrid = getGridPoint(end, end_g); 
    if(endInGrid)
	getEntry(end_g).addHeightMeasurement(end.z(), currentAge);
    
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
    getEntry(end_g).addHeightMeasurement(end_val, currentAge);
    
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
    
    if(binnedPoints.empty())
	return;
    
    ++currentAge;
    std::vector< binnedPoint >::const_iterator last_p = binnedPoints.begin();
    getEntry(last_p->gridPos).addHeightMeasurement(last_p->heightValue, currentAge);
    
    for(std::vector< binnedPoint >::const_iterator it = last_p + 1; it != binnedPoints.end(); it++) {
	if(it->validNeighbour)
	    addLineBetweenPoints(last_p->gridPos, last_p->heightValue, it->gridPos, it->heightValue);
	else
	    getEntry(it->gridPos).addHeightMeasurement(it->heightValue, currentAge);
	last_p = it;
    }

}

void ElevationGrid::setEntriesWindowSize(int window_size){
	for(int x = 0;x < getWidth(); x++){
		for(int y = 0;y < getHeight(); y++){
			getEntry(x,y).setEntryWindowSize(window_size);
		}
	}
}

void ElevationGrid::setHeightMeasureMethod(int entry_height_conf){
	for(int x = 0;x < getWidth(); x++){
			for(int y = 0;y < getHeight(); y++){
				getEntry(x,y).setHeightMeasureMethod(entry_height_conf);
			}
		}
}

boost::uint64_t ElevationGrid::getCurrentAge() const
{
    return currentAge;
}

