#ifndef VFH_STAR_TRAVERSABILITYGRID_H
#define VFH_STAR_TRAVERSABILITYGRID_H

#include "Grid.h"
#include "ElevationGrid.h"
#include "vfh_star/VFH.h"

namespace vfh_star
{

class TraversabilityGrid: public Grid<Traversability, GRIDSIZE, GRIDRESOLUTION>
{
    public:
        TraversabilityGrid();
        
        void updateFromElevationGrid(const ElevationGrid &eg);
};
}

#endif // TRAVERSABILITYGRID_H
