#ifndef VFH_STAR_TRAVERSABILITYGRID_H
#define VFH_STAR_TRAVERSABILITYGRID_H

#include "Grid.h"
#include "ElevationGrid.h"

namespace vfh_star
{
enum Traversability {
    UNCLASSIFIED = 0,
    TRAVERSABLE = 1,
    OBSTACLE = 2,
    UNKNOWN_OBSTACLE = 3,
};

class TraversabilityGrid: public Grid<Traversability, GRIDSIZE, GRIDRESOLUTION>
{
    public:
        TraversabilityGrid();
        
        void updateFromElevationGrid(const ElevationGrid &eg);
};
}

#endif // TRAVERSABILITYGRID_H
