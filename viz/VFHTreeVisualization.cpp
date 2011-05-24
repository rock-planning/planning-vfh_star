#include "VFHTreeVisualization.hpp"
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Point>
#include <boost/tuple/tuple.hpp>
#include <osg/LineWidth>
#include <osgText/Text>
#include <osg/PositionAttitudeTransform>
#include <QWidget>
#include <QSpinBox>
#include <QObject>


using namespace vizkit;
using namespace std;
using vfh_star::TreeNode;

class VFHTreeVisualization::Data {
public:
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    vfh_star::Tree data;

    // If true, only the nodes that are not eaves will be displayed
    // (true by default)
    bool removeLeaves;

    // If true, the color will depend on the heuristics, otherwise on the
    // actual node cost is displayed. False by default
    VFHTreeVisualization::COST_MODE costMode;

    bool hasSegment;
    std::pair< base::Vector3d, base::Vector3d > segment;

    int treeNodeCount;

    QTAdapter *adapter;
    
    Data()
        : removeLeaves(true), costMode(VFHTreeVisualization::SHOW_COST), hasSegment(false), treeNodeCount(0), adapter(0) {
	    
	}
	
    virtual ~Data() {};
    
};

void QTAdapter::numberChanged(int x)
{
    std::cout << "number changed " << std::endl;
    boost::mutex::scoped_lock lockit(viz->updateMutex);
    viz->setDirty();
    viz->p->treeNodeCount = x;
}

QTAdapter::QTAdapter()
{
    spinBox = new QSpinBox(); 
    this->connect(spinBox, SIGNAL(valueChanged(int)), this, SLOT(numberChanged(int)) );
    spinBox->setMinimum(0);
    spinBox->setMaximum(10000);
}

QTAdapter::~QTAdapter()
{

}

QWidget* VFHTreeVisualization::getSideWidget()
{
    if(!p->adapter)
    {
	p->adapter = new QTAdapter();
	p->adapter->viz = this;
    }

    return p->adapter->spinBox;
}



VFHTreeVisualization::VFHTreeVisualization()
    : p(new Data)
{
    VizPluginRubyAdapter(VFHTreeVisualization, vfh_star::Tree, Tree);
}

VFHTreeVisualization::~VFHTreeVisualization()
{
    delete p;
}

void VFHTreeVisualization::setCostMode(COST_MODE mode)
{
    p->costMode = mode;
    setDirty();
}

void VFHTreeVisualization::removeLeaves(bool enable)
{
    p->removeLeaves = enable;
    setDirty();
}

void VFHTreeVisualization::setMaxNodeCount(int count)
{
    p->treeNodeCount = count;
    setDirty();
}

osg::ref_ptr<osg::Node> VFHTreeVisualization::createMainNode()
{
    // Geode is a common node used for vizkit plugins. It allows to display
    // "arbitrary" geometries
    osg::PositionAttitudeTransform *treeGroup = new osg::PositionAttitudeTransform;
    treeGroup->setPosition(osg::Vec3(0,0,0.1));
    return treeGroup;
}

static double getDisplayCost(VFHTreeVisualization::COST_MODE mode, vfh_star::TreeNode const& node)
{
    if (mode == VFHTreeVisualization::SHOW_COST)
        return node.getCost();
    if (mode == VFHTreeVisualization::SHOW_HEURISTICS)
        return node.getHeuristic();
    if (mode == VFHTreeVisualization::SHOW_BOTH)
        return node.getHeuristicCost();
    return 0;
}

pair<double, double> VFHTreeVisualization::computeColorMapping(std::set<TreeNode const*> const& nodes) const
{
    // NOTE: we assume that nodes is not empty. This is checked by
    // updateMainNode
    if (nodes.empty())
        return make_pair(0, 0);

    double min_cost, max_cost;
    min_cost = max_cost = getDisplayCost(p->costMode, **nodes.begin());
    
    for (std::set<TreeNode const*>::const_iterator it = nodes.begin();
            it != nodes.end(); ++it)
    {
        if ((*it)->getHeuristic() < 0)
            continue;

        double c = getDisplayCost(p->costMode, **it);

        if (c < min_cost)
            min_cost = c;
        if (c > max_cost)
            max_cost = c;
    }


    if (max_cost == 0)
        return make_pair(0, 0);

    return make_pair(1.0/max_cost, -min_cost);
}

osg::Geometry* VFHTreeVisualization::createSolutionNode(TreeNode const* node, double color_a, double color_b)
{
    osg::Geometry* geom = new osg::Geometry;

    // Create the color and vertex arrays
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec4Array> colors   = new osg::Vec4Array;

    // Used later to represent the initial orientation
    double step = 0;
    while (!node->isRoot())
    {
        base::Position parent_p = node->getParent()->getPose().position;
        base::Position p = node->getPose().position;
        if (step == 0)
            step = (parent_p - p).norm();
	vertices->push_back(osg::Vec3(parent_p.x(), parent_p.y(), parent_p.z() + 0.001));
	vertices->push_back(osg::Vec3(p.x(), p.y(), p.z() + 0.001));

        double cost = getDisplayCost(this->p->costMode, *node);

        double red = color_a * (cost + color_b) / 2 + 0.5;
        double green = 1.0 - red;
        double blue = 0.5;
        double alpha = 1.0;
	colors->push_back(osg::Vec4(red, green, blue, alpha));

        node = node->getParent();
    }

    // Add a node for the root's direction
    Eigen::Quaterniond q(Eigen::AngleAxisd(node->getDirection(), Eigen::Vector3d::UnitZ()));
    base::Vector3d root_dir = q * Eigen::Vector3d::UnitY();
    base::Position parent_p = node->getPose().position;
    base::Position p = parent_p - root_dir * step;
    vertices->push_back(osg::Vec3(parent_p.x(), parent_p.y(), parent_p.z() + 0.001));
    vertices->push_back(osg::Vec3(p.x(), p.y(), p.z() + 0.001));
    colors->push_back(osg::Vec4(1.0, 1.0, 1.0, 1.0));

    geom->setColorArray(colors);
    geom->setColorBinding( osg::Geometry::BIND_PER_PRIMITIVE );
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

osg::Group* VFHTreeVisualization::createTreeNode(std::set<TreeNode const*> const& nodes, double color_a, double color_b)
{
    osg::Group *treeGroup = new osg::Group;
    osg::Geode* geode = new osg::Geode();
    osg::Geometry* geom = new osg::Geometry;

    // Create the color and vertex arrays
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec4Array> colors   = new osg::Vec4Array;

    int count = p->treeNodeCount;
    for(set<TreeNode const*>::const_iterator it = nodes.begin();
            it != nodes.end(); ++it)
    {
        if (count != 0 && (*it)->getIndex() > count)
            continue;

        TreeNode const* node = (*it);

        base::Position parent_p = node->getParent()->getPose().position;
        base::Position p = node->getPose().position;

// 	std::stringstream cost;
// 	cost << "C: " <<  node->getCost() << std::endl << "H: " << node->getHeuristic();
// 	
// 	osgText::Text *costText = new osgText::Text();
// 	costText->setCharacterSize(0.02);
// 	costText->setAxisAlignment(osgText::Text::SCREEN);
// 	costText->setText(cost.str());
// 	base::Position halfDistance = (parent_p - p) * 0.5 + p; 
// 	costText->setPosition(osg::Vec3(halfDistance.x(), halfDistance.y(), halfDistance.z()));
// 
// 	geode->addDrawable(costText);
// 	
	vertices->push_back(osg::Vec3(parent_p.x(), parent_p.y(), parent_p.z()));
	vertices->push_back(osg::Vec3(p.x(), p.y(), p.z()));

        if (node->getHeuristic() < 0) // used to mark invalid nodes
        {
	    if(node->getHeuristic() == -1)
	    {
		colors->push_back(osg::Vec4(0, 0, 0, 1));
// 		costText->setColor(osg::Vec4(0, 0, 0, 1));
	    } else
	    {
		colors->push_back(osg::Vec4(1, 1, 1, 0.5));
// 		costText->setColor(osg::Vec4(0, 0, 1, 1));
	    }
	}
	else
        {
            double cost = getDisplayCost(this->p->costMode, *node);

            double red = color_a * (cost + color_b);
            double green = 1.0 - red;
            double blue = 0.5;
            double alpha = 1.0;
            colors->push_back(osg::Vec4(red, green, blue, alpha));
// 	    costText->setColor(osg::Vec4(red, green, blue, alpha));
        }
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
    treeGroup->addChild(geode);
    return treeGroup;
}

void VFHTreeVisualization::updateMainNode ( osg::Node* node )
{
    std::cout << "Updating Node VFHTreeVisualization" << std::endl;
    osg::Group *mainNode = dynamic_cast<osg::Group *>(node);
    assert(mainNode);
    
    //Remove old childs
    mainNode->removeChildren(0, mainNode->getNumChildren());
    
    osg::Geode* geode = new osg::Geode();

    if (p->hasSegment)
    {
        osg::Geometry* geom = new osg::Geometry;
        osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
        osg::ref_ptr<osg::Vec4Array> colors   = new osg::Vec4Array;
        base::Vector3d pt = p->segment.first;
        vertices->push_back(osg::Vec3(pt.x(), pt.y(), pt.z()));
        pt = p->segment.second;
        vertices->push_back(osg::Vec3(pt.x(), pt.y(), pt.z()));
        colors->push_back(osg::Vec4(1.0, 1.0, 1.0, 1.0));

        geom->setColorArray(colors);
        geom->setColorBinding( osg::Geometry::BIND_OVERALL );
        geom->setVertexArray(vertices);
        osg::ref_ptr<osg::DrawArrays> drawArrays =
            new osg::DrawArrays( osg::PrimitiveSet::LINES, 0, vertices->size() );
        geom->addPrimitiveSet(drawArrays.get());
        geode->addDrawable(geom);
    }

    list<TreeNode> const& nodes = p->data.getNodes();
    if (nodes.empty())
        return;

    if (p->costMode == VFHTreeVisualization::SHOW_COST)
        std::cerr << "vfh_viz: displaying cost" << std::endl;
    else if (p->costMode == VFHTreeVisualization::SHOW_HEURISTICS)
        std::cerr << "vfh_viz: displaying heuristic" << std::endl;
    else if (p->costMode == VFHTreeVisualization::SHOW_BOTH)
        std::cerr << "vfh_viz: displaying cost+heuristic" << std::endl;

    std::cerr << "vfh_viz: " << p->data.getSize() << " nodes in tree" << std::endl;
    std::set<TreeNode const*> enabled_nodes;
    if (p->removeLeaves)
    {
        for(list<TreeNode>::const_iterator it = nodes.begin();
                it != nodes.end(); it++)
        {
            if (!it->isRoot() && !it->getParent()->isRoot())
                enabled_nodes.insert(it->getParent());
        }
    }
    else
    {
        for(list<TreeNode>::const_iterator it = nodes.begin();
                it != nodes.end(); it++)
            if (!it->isRoot())
                enabled_nodes.insert(&(*it));
    }
    // Add the final node regardless of the leaf mode
    if (p->data.getFinalNode())
        enabled_nodes.insert(p->data.getFinalNode());

    // Gets the mapping from cost to color
    double cost_a, cost_b;
    boost::tie(cost_a, cost_b) = computeColorMapping(enabled_nodes);
    std::cerr << "vfh_viz: cost mapping " << cost_a << " " << cost_b << std::endl;

    mainNode->addChild(createTreeNode(enabled_nodes, cost_a, cost_b));
    mainNode->addChild(geode);
    if (p->data.getFinalNode())
        geode->addDrawable(createSolutionNode(p->data.getFinalNode(), cost_a, cost_b));
}

void VFHTreeVisualization::updateDataIntern(vfh_star::Tree const& value)
{
    p->data = value;
}

void VFHTreeVisualization::updateDataIntern(Segment const& segment)
{
    p->segment = segment;
    p->hasSegment = true;
}

VizkitQtPlugin(VFHTreeVisualization);


