#include "VFHTreeVisualization.hpp"
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Point>
#include <boost/tuple/tuple.hpp>

using namespace vizkit;
using namespace std;
using vfh_star::TreeNode;

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

pair<double, double> VFHTreeVisualization::computeColorMapping() const
{
    // NOTE: we assume that nodes is not empty. This is checked by
    // updateMainNode
    list<TreeNode*> const& nodes = p->data.getNodes();

    double min_cost, max_cost;
    min_cost = max_cost = nodes.front()->getCost();
    
    for (list<TreeNode*>::const_iterator it = nodes.begin();
            it != nodes.end(); ++it)
    {
        double c = (*it)->getCost();
        if (c < min_cost)
            min_cost = c;
        if (c > max_cost)
            max_cost = c;
    }


    return make_pair(1.0/max_cost, -min_cost);
}

void VFHTreeVisualization::updateMainNode ( osg::Node* node )
{
    osg::Geode* geode = static_cast<osg::Geode*>(node);
    while(geode->removeDrawables(0));

    list<TreeNode*> const& nodes = p->data.getNodes();
    if (nodes.empty())
        return;

    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;

    // Gets the mapping from cost to color
    double cost_a, cost_b;
    boost::tie(cost_a, cost_b) = computeColorMapping();
    std::cerr << "cost mapping: " << cost_a << " " << cost_b << std::endl;

    // Create the color and vertex arrays
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec4Array> colors   = new osg::Vec4Array;
    for(list<TreeNode*>::const_iterator it = nodes.begin();
            it != nodes.end(); it++)
    {
        TreeNode const* node = (*it);
        base::Position parent_p = node->getParent()->getPose().position;
        base::Position p = node->getPose().position;
	vertices->push_back(osg::Vec3(parent_p.x(), parent_p.y(), parent_p.z()));
	vertices->push_back(osg::Vec3(p.x(), p.y(), p.z()));

        double cost = node->getCost();
        double red = cost_a * (cost + cost_b);
        double green = 1.0 - red;
        double blue = 0.5;
        double alpha = 1.0;
	colors->push_back(osg::Vec4(red, green, blue, alpha));
    }
    geom->setColorArray(colors);
    geom->setColorBinding( osg::Geometry::BIND_PER_PRIMITIVE );
    geom->setVertexArray(vertices);

    // Draw the vertices as points
    osg::ref_ptr<osg::DrawArrays> drawArrays =
        new osg::DrawArrays( osg::PrimitiveSet::LINES, 0, vertices->size() );
    geom->addPrimitiveSet(drawArrays.get());

    // Finally, setup the point attributes
    osg::ref_ptr<osg::Point> point = new osg::Point();
    point->setSize(10.0);
    point->setDistanceAttenuation( osg::Vec3(0.5, 0.5, 0.5 ) );
    point->setMinSize( 0.2 );
    point->setMaxSize( 5.0 );
    geom->getOrCreateStateSet()->setAttribute( point, osg::StateAttribute::ON );
    geode->addDrawable(geom);
}

void VFHTreeVisualization::updateDataIntern(vfh_star::Tree const& value)
{
    p->data = value;
}

