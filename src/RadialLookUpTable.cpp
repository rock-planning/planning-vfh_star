#include "RadialLookUpTable.hpp"
#include <math.h>

RadialLookUpTable::RadialLookUpTable() : numElementsPerLine(0), distanceTable(0), scale(0), maxRadius(0)
{
    
}

void RadialLookUpTable::computeDistances(double scale, double maxRadius)
{
    if(this->scale == scale && this->maxRadius == maxRadius && numElementsPerLine == maxRadius / scale)
	return;
    
    this->maxRadius = maxRadius;
    this->scale = scale;
    numElementsPerLine = maxRadius / scale;
    
    if(distanceTable)
	delete[] distanceTable;
    
    distanceTable = new double[numElementsPerLine * numElementsPerLine];
    for(int y = 0; y < numElementsPerLine; y++)
    {
	for(int x = 0; x < numElementsPerLine;x++)
	{
	    const double xd = x * scale;
	    const double yd = y * scale;
	    distanceTable[numElementsPerLine * y + x] = sqrt(xd*xd + yd*yd);
	}
    }
}


const double &RadialLookUpTable::getDistance(unsigned int x, unsigned int y) const
{
    return distanceTable[numElementsPerLine * y + x];
}

