#include "RadialLookUpTable.hpp"
#include <math.h>

RadialLookUpTable::RadialLookUpTable() : numElementsPerLine(0), distanceTable(0), angleTable(0), scale(0), maxRadius(0)
{
    
}

void RadialLookUpTable::recompute(double scale, double maxRadius)
{
    if(this->scale == scale && this->maxRadius == maxRadius)
	return;
    
    this->maxRadius = maxRadius;
    this->scale = scale;
    numElementsPerLine = maxRadius / scale * 2;
    numElementsPerLineHalf = maxRadius / scale;

    computeAngles();
    computeDistances();
}


void RadialLookUpTable::computeDistances()
{
    if(distanceTable)
	delete[] distanceTable;
    
    distanceTable = new double[numElementsPerLine * numElementsPerLine];
    for(int y = 0; y < numElementsPerLine; y++)
    {
	for(int x = 0; x < numElementsPerLine;x++)
	{
	    const double xd = (x - numElementsPerLineHalf) * scale;
	    const double yd = (y - numElementsPerLineHalf) * scale;
	    distanceTable[numElementsPerLine * y + x] = sqrt(xd*xd + yd*yd);
	}
    }
}

void RadialLookUpTable::computeAngles()
{
    if(angleTable)
	delete[] angleTable;
    
    angleTable = new double[numElementsPerLine * numElementsPerLine];
    for(int y = 0; y < numElementsPerLine; y++)
    {
	for(int x = 0; x < numElementsPerLine;x++)
	{
	    const double xd = (x - numElementsPerLineHalf) * scale;
	    const double yd = (y - numElementsPerLineHalf) * scale;
	    angleTable[numElementsPerLine * y + x] = atan2(yd, xd);
	}
    }
}

const double& RadialLookUpTable::getAngle(int x, int y) const
{
    unsigned int xd = x + numElementsPerLineHalf;
    unsigned int yd = y + numElementsPerLineHalf;
    return angleTable[numElementsPerLine * yd + xd];   
}


const double &RadialLookUpTable::getDistance(int x, int y) const
{
    unsigned int xd = x + numElementsPerLineHalf;
    unsigned int yd = y + numElementsPerLineHalf;
    return distanceTable[numElementsPerLine * yd + xd];
}

