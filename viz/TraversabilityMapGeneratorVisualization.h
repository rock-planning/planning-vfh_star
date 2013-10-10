#ifndef GRIDMAPSEGMENTERVISUALIZATION_H
#define GRIDMAPSEGMENTERVISUALIZATION_H

#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <vfh_star/TraversabilityMapGenerator.h>
#include <osg/Vec4>
#include <osg/Vec3>

namespace vizkit3d {

class TraversabilityMapGeneratorVisualization: public vizkit3d::Vizkit3DPlugin<vfh_star::GridDump >
{
    public:
	TraversabilityMapGeneratorVisualization();
    private:
	virtual osg::ref_ptr< osg::Node > createMainNode();
	virtual void updateMainNode(osg::Node* node);
	
	virtual void updateDataIntern(const vfh_star::GridDump& data);
	double **grid;
	osg::Vec4 **gridColor;
	int width;
	int height;
	double gridEntrySize;
	osg::Vec3 robotPos;
	osg::Vec3 gridPos;
};

}
#endif // GRIDMAPSEGMENTERVISUALIZATION_H
