#ifndef RADIALLOOKUPTABLE_HPP
#define RADIALLOOKUPTABLE_HPP

class RadialLookUpTable
{

public:
    RadialLookUpTable();
    
    const double& getDistance(int x, int y) const;
    const double& getAngle(int x, int y) const;
    
    void recompute(double scale, double maxRadius);
    
private:
    void computeDistances();
    void computeAngles();
    
    int numElementsPerLine;
    int numElementsPerLineHalf;
    double *distanceTable;
    double *angleTable;
    double scale;
    double maxRadius;
};

#endif // RADIALLOOKUPTABLE_HPP
