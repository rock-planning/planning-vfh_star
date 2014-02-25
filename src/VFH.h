#ifndef VFH_H
#define VFH_H

#include <envire/maps/Grid.hpp>
#include <envire/maps/TraversabilityGrid.hpp>
#include <vector>
#include <base/Pose.hpp>
#include "RadialLookUpTable.hpp"

#include "Types.h"
#include <base/Angle.hpp>

namespace vfh_star
{
    class VFH
    {
    public:
        VFH();
        std::vector< base::AngleSegment >
            getNextPossibleDirections(const base::Pose& curPose) const;

        void setConfig(const VFHConf &conf);
        
	/**
	 * Return weather vfh can return possible directions for this position
	 * */
	bool validPosition(const base::Pose& curPose) const;
	
        void setNewTraversabilityGrid(const envire::TraversabilityGrid *trGrid);
        const envire::TraversabilityGrid *getTraversabilityGrid() const;
        
    private:
        void generateHistogram(std::vector< double >& histogram, const base::Pose& curPose) const;

        void getBinaryHistogram(const std::vector< double >& histogram, std::vector< bool >& binHistogram) const;
        void addDir(std::vector< base::AngleSegment >& drivableDirections, int start, int end) const;
	RadialLookUpTable lut;
        std::vector<bool> obstacleLookup;
        const envire::TraversabilityGrid *traversabillityGrid;
        double gridWidthHalf;
        double gridHeightHalf;

        VFHConf config;
        double angularResolution;
        bool debugActive;
    };

}

#endif // VFH_H
