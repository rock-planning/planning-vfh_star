#include "VFHTreeVisualization.hpp"
#include <../../../src/TreeSearch.h>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Point>
#include <boost/tuple/tuple.hpp>
#include <osg/LineWidth>
#include <base/float.h>
#include <vizkit3d/Vizkit3DHelper.hpp>

#include <iostream>

using namespace vizkit3d;
using namespace std;
using namespace vfh_star;

struct VFHTreeVisualization::Data {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    vfh_star::DebugTree data;

    // If true, only the nodes that are not eaves will be displayed
    // (true by default)
    bool removeLeaves;

    bool hasHorizon;
    base::Vector3d horizonOrigin;
    base::Vector3d horizonVector;
    
    int treeNodeCount;

    Data()
        : removeLeaves(false), hasHorizon(false), treeNodeCount(0) {}
};


VFHTreeVisualization::VFHTreeVisualization()
    : p(new Data)
{
}

VFHTreeVisualization::~VFHTreeVisualization()
{
    delete p;
}

void VFHTreeVisualization::removeLeaves(bool enable)
{
    p->removeLeaves = enable;
    setDirty();
}

bool VFHTreeVisualization::areLeavesRemoved()
{
    return p->removeLeaves;
}

void VFHTreeVisualization::setMaxNodeCount(int count)
{
    p->treeNodeCount = count;
    setDirty();
}

int VFHTreeVisualization::getMaxNodeCount()
{
    return p->treeNodeCount;
}

osg::ref_ptr<osg::Node> VFHTreeVisualization::createMainNode()
{
    // Geode is a common node used for vizkit plugins. It allows to display
    // "arbitrary" geometries
    osg::ref_ptr<osg::PositionAttitudeTransform> transform = new osg::PositionAttitudeTransform();
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    transform->addChild(geode);
    osg::StateSet *state = geode->getOrCreateStateSet();
    state->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
    return transform;
}

pair<double, double> VFHTreeVisualization::computeColorMapping(const vfh_star::DebugTree& tree) const
{
    if (tree.nodes.empty())
        return make_pair(0, 0);

    double max_cost = 0;

    for(std::vector<vfh_star::DebugNode>::const_iterator nodeIt = tree.nodes.begin(); nodeIt != tree.nodes.end(); nodeIt++)
    {
        const double curCost = nodeIt->cost;
        if(isinf(curCost))
            continue;
        
        if(curCost > max_cost)
        {
            max_cost = curCost;
        }
    }

    if (max_cost == 0)
        return make_pair(0, 0);

    return make_pair(1.0/max_cost, 0);
}


osg::Geometry* VFHTreeVisualization::createSolutionNode(const vfh_star::DebugTree& tree, double color_a, double color_b)
{
    osg::Geometry* geom = new osg::Geometry;
    
    if(!tree.hasFinalNode())
        return geom;

    // Create the color and vertex arrays
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec4Array> colors   = new osg::Vec4Array;

    // Used later to represent the initial orientation
    double step = 0;

    const vfh_star::DebugNode *curNode(&(tree.nodes[tree.finalNode]));
    
    
    while (curNode->creationOrder != tree.startNode)
    {
        const vfh_star::DebugNode &parent(tree.nodes[curNode->parent]);
        base::Position parent_p = parent.pose.position;
        base::Position p = curNode->pose.position;
        if (step == 0)
            step = (parent_p - p).norm();
        
        
        vertices->push_back(osg::Vec3(parent_p.x(), parent_p.y(), parent_p.z() + 0.001));
        vertices->push_back(osg::Vec3(p.x(), p.y(), p.z() + 0.001));
        Eigen::Vector3d halfWay((p - parent_p ) / 2.0);
        Eigen::Vector3d middle(parent_p + halfWay);
        Eigen::Vector3d driveToPoint(middle + curNode->pose.orientation * Eigen::Vector3d(step / 2.0, 0, 0));
        
        //make it an arrow
        Eigen::Vector3d left(middle - driveToPoint);
        left = left.normalized() * step * 0.3;
        left = Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitZ()) * left;
        left += driveToPoint;
        vertices->push_back(osg::Vec3(driveToPoint.x(), driveToPoint.y(), driveToPoint.z() + 0.001));
        vertices->push_back(osg::Vec3(left.x(), left.y(), left.z() + 0.001));

        Eigen::Vector3d right(middle - driveToPoint);
        right = right.normalized() * step * 0.3;
        right = Eigen::AngleAxisd(-M_PI/4, Eigen::Vector3d::UnitZ()) * right;
        right += driveToPoint;
        vertices->push_back(osg::Vec3(driveToPoint.x(), driveToPoint.y(), driveToPoint.z() + 0.001));
        vertices->push_back(osg::Vec3(right.x(), right.y(), right.z() + 0.001));


        double cost = curNode->cost;

        double red = color_a * (cost + color_b) / 2 + 0.5;
        double green = 1.0 - red;
        double blue = 0.5;
        double alpha = 1.0;
        colors->push_back(osg::Vec4(red, green, blue, alpha));
        colors->push_back(osg::Vec4(red, green, blue, alpha));
        colors->push_back(osg::Vec4(red, green, blue, alpha));
        colors->push_back(osg::Vec4(red, green, blue, alpha));
        colors->push_back(osg::Vec4(red, green, blue, alpha));
        colors->push_back(osg::Vec4(red, green, blue, alpha));

        curNode = &(tree.nodes[curNode->parent]);
    }

    // Add a node for the root's direction
    Eigen::Quaterniond q(Eigen::AngleAxisd(curNode->pose.getYaw(), Eigen::Vector3d::UnitZ()));
    base::Vector3d root_dir = q * Eigen::Vector3d::UnitX();
    base::Position parent_p = curNode->pose.position;
    base::Position p = parent_p - root_dir * step;
    vertices->push_back(osg::Vec3(parent_p.x(), parent_p.y(), parent_p.z() + 0.001));
    vertices->push_back(osg::Vec3(p.x(), p.y(), p.z() + 0.001));

    Eigen::Vector3d halfWay((p - parent_p ) / 2.0);
    Eigen::Vector3d middle(parent_p + halfWay);
    Eigen::Vector3d driveToPoint(middle + curNode->pose.orientation * Eigen::Vector3d( step / 2.0, 0, 0));
    
    //make it an arrow
    Eigen::Vector3d left(middle - driveToPoint);
    left = left.normalized() * step * 0.3;
    left = Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitZ()) * left;
    left += driveToPoint;
    vertices->push_back(osg::Vec3(driveToPoint.x(), driveToPoint.y(), driveToPoint.z() + 0.001));
    vertices->push_back(osg::Vec3(left.x(), left.y(), left.z() + 0.001));

    Eigen::Vector3d right(middle - driveToPoint);
    right = right.normalized() * step * 0.3;
    right = Eigen::AngleAxisd(-M_PI/4, Eigen::Vector3d::UnitZ()) * right;
    right += driveToPoint;
    vertices->push_back(osg::Vec3(driveToPoint.x(), driveToPoint.y(), driveToPoint.z() + 0.001));
    vertices->push_back(osg::Vec3(right.x(), right.y(), right.z() + 0.001));
        
    colors->push_back(osg::Vec4(1.0, 1.0, 1.0, 1.0));
    colors->push_back(osg::Vec4(1.0, 1.0, 1.0, 1.0));
    colors->push_back(osg::Vec4(1.0, 1.0, 1.0, 1.0));
    colors->push_back(osg::Vec4(1.0, 1.0, 1.0, 1.0));
    colors->push_back(osg::Vec4(1.0, 1.0, 1.0, 1.0));
    colors->push_back(osg::Vec4(1.0, 1.0, 1.0, 1.0));

    geom->setColorArray(colors);
    geom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
    geom->setVertexArray(vertices);

    // Draw the vertices as points
    osg::ref_ptr<osg::DrawArrays> drawArrays =
        new osg::DrawArrays( osg::PrimitiveSet::LINES, 0, vertices->size() );
    geom->addPrimitiveSet(drawArrays.get());

    // Finally, setup the point attributes
    osg::ref_ptr<osg::LineWidth> lw = new osg::LineWidth();
    lw->setWidth(10.0);
    geom->getOrCreateStateSet()->setAttribute( lw, osg::StateAttribute::ON );

    return geom;
}

osg::Geometry* VFHTreeVisualization::createTreeNode(DebugTree const &tree, double color_a, double color_b)
{
    osg::Geometry* geom = new osg::Geometry;

    // Create the color and vertex arrays
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec4Array> colors   = new osg::Vec4Array;

    int count = p->treeNodeCount;

    int nodeCounter = 0;
    
    for(std::vector<DebugNode>::const_iterator nodeIt = tree.nodes.begin(); nodeIt != tree.nodes.end(); nodeIt++)
    {
        if (count != 0 && nodeCounter > count)
            break;

        nodeCounter++;

        //skip root node
        if(nodeIt->creationOrder == tree.startNode)
            continue;

        if(p->removeLeaves && nodeIt->childs.empty())
            continue;
        
        //draw line to parent node
        
//         for(std::vector<size_t>::const_iterator childIt = nodeIt->childs.begin(); childIt != nodeIt->childs.end(); childIt++)
//         {
        const DebugNode &child(*nodeIt);
        const DebugNode &parent(tree.nodes[nodeIt->parent]);
        base::Position parent_p = parent.pose.position;
        base::Position p = child.pose.position;
        vertices->push_back(osg::Vec3(parent_p.x(), parent_p.y(), parent_p.z() + 0.001));
        vertices->push_back(osg::Vec3(p.x(), p.y(), p.z() + 0.001));

//         std::cout << "Child Id " << nodeIt->creationOrder << " Pos " << p.transpose() << " Parent Id " << nodeIt->parent << " Pos " << parent_p.transpose() << " " << std::endl;
        
        if (!child.isValid)
        {
            colors->push_back(osg::Vec4(0, 0, 0, 1));
            colors->push_back(osg::Vec4(0, 0, 0, 1));
        }
        else
        {
            double cost = child.cost;

            double red = color_a * (cost + color_b);
            double green = 1.0 - red;
            double blue = 0.5;
            double alpha = 1.0;
            colors->push_back(osg::Vec4(red, green, blue, alpha));
            colors->push_back(osg::Vec4(red, green, blue, alpha));
        }        
//         }
        geom->setColorArray(colors);
        geom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
        geom->setVertexArray(vertices);
    }
    
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

    return geom;
}

// void VFHTreeVisualization::addRecursive(std::multimap<double, TreeNode const*> &sorted_nodes, TreeNode const* curNode) const
// {
//     for(std::vector<TreeNode *>::const_iterator it = curNode->getChildren().begin(); it != curNode->getChildren().end();it++)
//     {
// 	if (p->removeLeaves && (*it)->isLeaf())
// 	    continue;
//         
// 	addRecursive(sorted_nodes, *it);
//     }
//     
//     sorted_nodes.insert(std::make_pair(curNode->getIndex(), curNode));
// }

void VFHTreeVisualization::updateMainNode ( osg::Node* node )
{
    osg::PositionAttitudeTransform* transform = dynamic_cast<osg::PositionAttitudeTransform*>(node);
    transform->setPosition(eigenVectorToOsgVec3(p->data.treePos.position));
    transform->setAttitude(eigenQuatToOsgQuat(p->data.treePos.orientation));
    
    std::cout << "VFHTreeVisualization : Tree2World " << p->data.treePos.position.transpose() << std::endl;
    
    osg::Geode* geode = transform->getChild(0)->asGeode();
    while(geode->removeDrawables(0));

    if (p->hasHorizon)
    {
        osg::Geometry* geom = new osg::Geometry;
        osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
        osg::ref_ptr<osg::Vec4Array> colors   = new osg::Vec4Array;
        vertices->push_back(eigenVectorToOsgVec3(p->horizonOrigin + p->horizonVector * 0.5));
        vertices->push_back(eigenVectorToOsgVec3(p->horizonOrigin - p->horizonVector * 0.5));
        colors->push_back(osg::Vec4(1.0, 1.0, 1.0, 1.0));

        geom->setColorArray(colors);
        geom->setColorBinding( osg::Geometry::BIND_OVERALL );
        geom->setVertexArray(vertices);
        osg::ref_ptr<osg::DrawArrays> drawArrays =
            new osg::DrawArrays( osg::PrimitiveSet::LINES, 0, vertices->size() );
        geom->addPrimitiveSet(drawArrays.get());
        geode->addDrawable(geom);
    }

    // Gets the mapping from cost to color
    double cost_a, cost_b;
    boost::tie(cost_a, cost_b) = computeColorMapping(p->data);

    geode->addDrawable(createTreeNode(p->data, cost_a, cost_b));
    if (p->data.hasFinalNode())
        geode->addDrawable(createSolutionNode(p->data, cost_a, cost_b));
}


void VFHTreeVisualization::updateDataIntern(const DebugTree& data)
{
    p->data = data;
}

void VFHTreeVisualization::updateDataIntern(const vfh_star::HorizonPlannerDebugData& data)
{
    p->horizonOrigin = data.horizonOrigin;
    p->horizonVector = data.horizonVector;
    p->hasHorizon = true;
}

