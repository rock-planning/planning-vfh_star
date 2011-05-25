#include "TraversabilityGrid.h"

using namespace vfh_star;
TraversabilityGrid::TraversabilityGrid()
{
    for(int x = 0; x < getWidth(); x++) {
	for(int y = 0; y < getHeight(); y++) {
	    getEntry(x ,y) = UNCLASSIFIED;
	}
    }
}



void TraversabilityGrid::updateFromElevationGrid(const ElevationGrid& eg)
{

}
