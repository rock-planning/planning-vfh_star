#include "VFHTreeVisualization.hpp"
#include <osg/Geometry>
#include <osg/Geode>

using namespace vizkit;

struct VFHTreeVisualization::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    vfh_star::Tree data;
};


VFHTreeVisualization::VFHTreeVisualization()
    : p(new Data)
{
}

VFHTreeVisualization::~VFHTreeVisualization()
{
    delete p;
}

osg::ref_ptr<osg::Node> VFHTreeVisualization::createMainNode()
{
    // Geode is a common node used for vizkit plugins. It allows to display
    // "arbitrary" geometries
    return new osg::Geode();
}

void VFHTreeVisualization::updateMainNode ( osg::Node* node )
{
    osg::Geode* geode = static_cast<osg::Geode*>(node);
    // Update the main node using the data in p->data
}

void VFHTreeVisualization::updateDataIntern(vfh_star::Tree const& value)
{
    p->data = value;
}

