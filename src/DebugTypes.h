#ifndef VFH_DEBUG_TYPES_H
#define VFH_DEBUG_TYPES_H

#include <stdint.h>

namespace vfh_star {

#define GRIDSIZE 600
#define GRIDRESOLUTION 6
    
struct GridDump {
    float height[GRIDSIZE*GRIDSIZE/GRIDRESOLUTION];
    double max[GRIDSIZE*GRIDSIZE/GRIDRESOLUTION];
    bool interpolated[GRIDSIZE*GRIDSIZE/GRIDRESOLUTION];
    uint8_t traversability[GRIDSIZE*GRIDSIZE/GRIDRESOLUTION];
    float gridPositionX;
    float gridPositionY;
    float gridPositionZ;
};

}

#endif
