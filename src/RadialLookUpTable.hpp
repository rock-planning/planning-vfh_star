#ifndef RADIALLOOKUPTABLE_HPP
#define RADIALLOOKUPTABLE_HPP

class RadialLookUpTable
{

public:
    RadialLookUpTable();
    
    const double& getDistance(unsigned int x, unsigned int y) const;
    void computeDistances(double scale, double maxRadius);    
private:
    
    
    int numElementsPerLine;
    double *distanceTable;
    double scale;
    double maxRadius;
};

#endif // RADIALLOOKUPTABLE_HPP
