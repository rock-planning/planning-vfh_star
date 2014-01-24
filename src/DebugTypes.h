#ifndef VFH_DEBUG_TYPES_H
#define VFH_DEBUG_TYPES_H

#include <stdint.h>
#include <vector>
#include "Grid.h"

namespace vfh_star {
    
struct GridDump {
    std::vector<float> height;
    std::vector<double> max;
    std::vector<uint8_t> interpolated;
    std::vector<uint8_t> traversability;
    float gridPositionX;
    float gridPositionY;
    float gridPositionZ;

    GridDump()
    {
        height.resize(GRIDSIZE*GRIDSIZE/GRIDRESOLUTION);
        max.resize(GRIDSIZE*GRIDSIZE/GRIDRESOLUTION);
        interpolated.resize(GRIDSIZE*GRIDSIZE/GRIDRESOLUTION);
        traversability.resize(GRIDSIZE*GRIDSIZE/GRIDRESOLUTION);
    }
};

}

#endif
