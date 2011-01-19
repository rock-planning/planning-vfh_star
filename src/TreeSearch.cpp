#include "TreeSearch.h"
#include <Eigen/Core>
#include <map>
#include <stdexcept>


using namespace vfh_star;

TreeSearchConf::TreeSearchConf()
    : maxTreeSize(0)
    , stepDistance(0.5)
    , angularSamplingMin(2 * M_PI / 50)
    , angularSamplingMax(2 * M_PI / 20)
    , angularSamplingNominalCount(4)
    , discountFactor(1.0)
    , obstacleSafetyDistance(0.1)
    , robotWidth(0.5) {}

TreeSearch::TreeSearch()
{
}

void TreeSearch::setSearchConf(const TreeSearchConf& conf)
{
    this->search_conf = conf;
}

const TreeSearchConf& TreeSearch::getSearchConf() const
{
    return search_conf;
}

TreeSearch::Angles TreeSearch::getDirectionsFromIntervals(const TreeSearch::AngleIntervals& intervals)
{
    std::vector< double > ret;

    double minStep = search_conf.angularSamplingMin;
    double maxStep = search_conf.angularSamplingMax;
    int minNodes = search_conf.angularSamplingNominalCount;
    
    // double size = intervals.size();
    // std::cout << "Interval vector size " << size << std::endl;
    
    for (AngleIntervals::const_iterator it = intervals.begin(); it != intervals.end(); it++) 
    {
	double start = (it->first);
	double end  = (it->second);

        double intervalOpening;

	// Special case, wrapping interval
	if (start > end)
            intervalOpening = end + 2 * M_PI - start;
        else
            intervalOpening = end - start;

        double step = 0;
        if (intervalOpening / minNodes < minStep)
            step = minStep;
        else if (intervalOpening / minNodes > maxStep)
            step = maxStep;
        else
            step = intervalOpening / minNodes;

        int intervalSize = floor(intervalOpening / step);
        double delta = (intervalOpening - (intervalSize * step)) / 2;
        for (int i = 0; i < intervalSize; ++i)
        {
            double angle = start + delta + i * step;
            if (angle > 2 * M_PI)
                ret.push_back(angle - 2 * M_PI);
            else
                ret.push_back(angle);
        }
    }

    //std::cerr << "found " << ret.size() << " possible directions" << std::endl;
    
    return ret;
}

base::geometry::Spline<3> TreeSearch::waypointsToSpline(const std::vector<base::Waypoint>& waypoints)
{
    if (waypoints.empty())
        return base::geometry::Spline<3>();

    std::vector<base::Vector3d> as_points;
    for(std::vector<base::Waypoint>::const_iterator it = waypoints.begin();
            it != waypoints.end(); it++)
        as_points.push_back(it->position);

    base::geometry::Spline<3> spline;
    spline.interpolate(as_points);
    return spline;
}

base::geometry::Spline<3> TreeSearch::getTrajectory(const base::Pose& start)
{
    std::vector<base::Waypoint> waypoints =
        getWaypoints(start);

    return waypointsToSpline(waypoints);
}

TreeNode const* TreeSearch::compute(const base::Pose& start)
{
    std::multimap<double, TreeNode *> expandCandidates;

    tree.clear();
    kdtree.clear();
    TreeNode *curNode = tree.createRoot(start, start.getYaw());
    kdtree.insert(curNode);

    expandCandidates.insert(std::make_pair(0, curNode));
    
    int max_depth = search_conf.maxTreeSize;
    while(!expandCandidates.empty()) 
    {
        if (max_depth > 0 && tree.getSize() > max_depth)
            return 0;

        /*	std::cout << "Possible expandable nodes" << std::endl;
                for(std::multimap<double, TreeNode *>::iterator it = expandCandidates.begin(); it != expandCandidates.end(); it++)
                {
                std::cout << "Node: dir " << it->second->getDirection() << " cost " << it->second->getCost() << "," << it->first << " depth is " << it->second->getDepth() << " Pos " << it->second->getPose().position.transpose() << std::endl;	    
                }
                std::cout << std::endl;*/

        // std::cout << expandCandidates.size() << " candidates in queue\n";

            // get the cheapest node for expansion
        curNode = expandCandidates.begin()->second;
        expandCandidates.erase(expandCandidates.begin());

        if (!validateNode(*curNode))
            continue;

        // 	 std::cout << "Expanding node with dir " << curNode->getDirection() << " cost " << curNode->getCost() << " depth is " << curNode->getDepth() << " pos " << curNode->getPose().position.transpose() << " heading " << curNode->getDirection() << std::endl;


        base::Position p = curNode->getPose().position;
        //std::cout << "opened " << p.x() << ", " << p.y() << ", " << p.z() << " direction=" << curNode->getDirection() << "\n    depth=" << curNode->getDepth() << " cost=" << curNode->getCost() << " heuristic= " << curNode->getHeuristic() << "\n";
        if (isTerminalNode(*curNode))
        {
            tree.setFinalNode(curNode);
            break;
        }

        //get possible ways to go for node
        AngleIntervals driveIntervals =
            getNextPossibleDirections(*curNode,
                    search_conf.obstacleSafetyDistance, search_conf.robotWidth);

        if (driveIntervals.empty())
            continue;

        Angles driveDirection =
            getDirectionsFromIntervals(driveIntervals);
        if (driveDirection.empty())
            continue;

        double curDiscount = pow(search_conf.discountFactor, curNode->getDepth());

        //expand node
        for(Angles::const_iterator it = driveDirection.begin(); it != driveDirection.end(); it++)
        {
            const double curDirection(*it);

            //generate new node
            std::pair<base::Pose, bool> projected =
                getProjectedPose(*curNode, curDirection,
                        search_conf.stepDistance);
            if (!projected.second)
                continue;

            //compute cost for it
            double nodeCost = curDiscount * getCostForNode(projected.first, curDirection, *curNode);

            // Check that we are not doing the same work multiple times. Node
            // that we want to keep a tree structure, so we skip nodes only if
            // they do not provide a shorter path than what's already existing
            //
            // searchNode should be used only for the search !
            TreeNode searchNode(projected.first, curDirection);
            double identity_threshold = search_conf.stepDistance / 5;
            std::pair<NNSearch::const_iterator, double> this_nn =
                kdtree.find_nearest(&searchNode, identity_threshold);
            if (this_nn.first != kdtree.end())
            {
                TreeNode const* closest_node   = *(this_nn.first);
                if (closest_node->getCost() <= nodeCost)
                {
                    TreeNode const* closest_parent = closest_node->getParent();
                    double parent_d = (closest_parent->getPose().position - curNode->getPose().position).norm();
                    if (parent_d < identity_threshold)
                        continue;
                }
            }


            //add Node to tree
            TreeNode *newNode = tree.createChild(curNode, projected.first, curDirection);
            newNode->setCost(curNode->getCost() + nodeCost);
            newNode->setPositionTolerance(search_conf.obstacleSafetyDistance);
            newNode->setHeadingTolerance(std::numeric_limits< double >::signaling_NaN());
            newNode->setHeuristic(curDiscount * getHeuristic(*newNode));

            // Add it to the expand list
            expandCandidates.insert(std::make_pair(newNode->getHeuristicCost(), newNode));

            // And add it to the kdtree
            kdtree.insert(newNode);

            // 	    std::cout << "Adding node for direction " << *it << " cost " << newNode->getCost() << " depth is " << newNode->getDepth() << " pos " << curNode->getPose().position.transpose() << " heading " << curNode->getDirection() << std::endl;
        }
    }

    curNode = tree.getFinalNode();
    if (curNode)
    {
        tree.verifyHeuristicConsistency(curNode);
        return curNode;
    }
    return 0;
}

std::vector< base::Waypoint > TreeSearch::getWaypoints(const base::Pose& start)
{
    TreeNode const* curNode = compute(start);
    if (curNode)
        return tree.buildTrajectoryTo(curNode);
    else
        return std::vector<base::Waypoint>();
}

bool TreeSearch::validateNode(const TreeNode& node) const
{
    return true;
}

const Tree& TreeSearch::getTree() const
{
    return tree;
}

TreeSearch::~TreeSearch()
{

}

Tree::Tree()
    : size(0)
    , final_node(0)
{
}

Tree::~Tree()
{
}

Tree::Tree(Tree const& other)
{
    *this = other;
}


Tree& Tree::operator = (Tree const& other)
{
    if (this == &other)
        return *this;

    clear();

    std::map<TreeNode const*, TreeNode*> node_map;
    for (std::list<TreeNode>::const_iterator it = other.nodes.begin();
            it != other.nodes.end(); ++it)
    {
        TreeNode const* orig_node = &(*it);
        TreeNode* new_node;
        if (orig_node->isRoot())
            new_node = createRoot(orig_node->getPose(), orig_node->getDirection());
        else
            new_node = createChild(node_map[orig_node->getParent()], orig_node->getPose(), orig_node->getDirection());

        new_node->setCost(orig_node->getCost());
        new_node->setHeuristic(orig_node->getHeuristic());
        new_node->setHeadingTolerance(orig_node->getHeadingTolerance());
        new_node->setPositionTolerance(orig_node->getPositionTolerance());

        node_map.insert( std::make_pair(orig_node, new_node) );
    }
    if (other.getFinalNode())
        setFinalNode(node_map[other.getFinalNode()]);
    return *this;
}

std::vector<base::Waypoint> Tree::buildTrajectoryTo(TreeNode const* node) const
{
    int size = node->getDepth() + 1;
    std::vector< base::Waypoint > result; 
    result.resize(size);
    for (int i = 0; i < size; ++i)
    {
        base::Pose p = node->getPose();
        base::Waypoint& wp = result[size-1-i];
        wp.heading  = p.getYaw();
        wp.position = p.position;
        wp.tol_position = node->getPositionTolerance();
        wp.tol_heading  = node->getHeadingTolerance();

        if (node->isRoot() && i != size - 1)
            throw std::runtime_error("internal error in buildTrajectoryTo: found a root node even though the trajectory is not finished");
        node = node->getParent();
    }

    // Small sanity check. The last node should be the root node
    if (!node->isRoot())
        throw std::runtime_error("internal error in buildTrajectoryTo: final node is not root");

    return result;
}

TreeNode* Tree::getRootNode()
{
    return &(nodes.front());
}

TreeNode* Tree::getFinalNode() const
{
    return final_node;
}

void Tree::setFinalNode(TreeNode* node)
{
    final_node = node;
}

void Tree::reserve(int size)
{
    int diff = free_nodes.size() - size;
    for (int i = 0; i < diff; ++i)
        free_nodes.push_back(TreeNode());
}

TreeNode* Tree::createNode(base::Pose const& pose, double dir)
{
    if (!free_nodes.empty())
    {
        nodes.splice(nodes.end(), free_nodes, free_nodes.begin());
        nodes.back() = TreeNode();
    }
    else
        nodes.push_back(TreeNode());

    TreeNode* n = &nodes.back();
    n->pose = pose;
    n->direction = dir;
    n->parent = n;
    n->index  = size;
    ++size;
    return n;
}

TreeNode* Tree::createRoot(base::Pose const& pose, double dir)
{
    if (!nodes.empty())
        throw std::runtime_error("trying to create a root node of an non-empty tree");

    return createNode(pose, dir);
}

TreeNode* Tree::createChild(TreeNode* parent, base::Pose const& pose, double dir)
{
    TreeNode* child = createNode(pose, dir);
    child->parent = parent;
    child->depth  = parent->depth + 1;
    parent->is_leaf = false;
    return child;
}

TreeNode *Tree::getParent(TreeNode* child)
{
    return child->parent;
}

int Tree::getSize() const
{
    return size;
}

void Tree::clear()
{
    free_nodes.splice(free_nodes.end(), nodes);
    final_node = 0;
    size = 0;
}

std::list<TreeNode> const& Tree::getNodes() const
{
    return nodes;
}

void Tree::verifyHeuristicConsistency(const TreeNode* from) const
{
    bool alright = true;

    base::Position leaf_p = from->getPose().position;

    const TreeNode* node = from;
    double actual_cost = node->getCost();
    while (!node->isRoot())
    {
        if (node->getHeuristicCost() > actual_cost)
        {
            base::Position p = node->getPose().position;
            std::cerr << "found invalid heuristic cost\n"
                << "  node at " << p.x() << " " << p.y() << " " << p.z() << " has c=" << node->getCost() << " h=" << node->getHeuristic() << " hc=" << node->getHeuristicCost() << "\n"
                << "  the corresponding leaf at " << leaf_p.x() << " " << leaf_p.y() << " " << leaf_p.z() << " has cost " << actual_cost << std::endl;
            alright = false;
        }
        node = node->getParent();
    }

    if (!alright)
        throw std::runtime_error("the chosen heuristic is not a minorant");
}

TreeNode::TreeNode(): parent(this), direction(0), cost(0), heuristic(0), depth(0), index(0)
{

}

TreeNode::TreeNode(const base::Pose& pose, double dir)
    : parent(this)
    , is_leaf(true)
    , pose(pose)
    , direction(dir)
    , cost(0)
    , heuristic(0)
    , depth(0)
    , index(0)
    , positionTolerance(0)
    , headingTolerance(0)
{

}


double TreeNode::getHeuristic() const
{
    return heuristic;
}

void TreeNode::setHeuristic(double value)
{
    heuristic = value;
}

double TreeNode::getHeuristicCost() const
{
    return cost + heuristic;
}

double TreeNode::getCost() const
{
    return cost;
}
void TreeNode::setCost(double value)
{
    cost = value;
}

const TreeNode* TreeNode::getParent() const
{
    return parent;
}

const base::Pose& TreeNode::getPose() const
{
    return pose;
}

int TreeNode::getDepth() const
{
    return depth;
}

bool TreeNode::isRoot() const
{
    return parent == this;
}

bool TreeNode::isLeaf() const
{
    return is_leaf;
}

double TreeNode::getDirection() const
{
    return direction;
}

int TreeNode::getIndex() const
{
    return index;
}

double TreeNode::getPositionTolerance() const
{
    return positionTolerance;
}
void TreeNode::setPositionTolerance(double tol)
{
    positionTolerance = tol;
}
double TreeNode::getHeadingTolerance() const
{
    return headingTolerance;
}
void TreeNode::setHeadingTolerance(double tol)
{
    headingTolerance = tol;
}

