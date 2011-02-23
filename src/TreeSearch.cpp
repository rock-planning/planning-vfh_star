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
    , robotWidth(0.5)
    , identityThreshold(-1)
    {}

TreeSearch::TreeSearch()
{
}

void TreeSearch::setSearchConf(const TreeSearchConf& conf)
{
    this->search_conf = conf;
    if(search_conf.identityThreshold < 0)
    {
	search_conf.identityThreshold = search_conf.stepDistance / 5.0;
    }
}

const TreeSearchConf& TreeSearch::getSearchConf() const
{
    return search_conf;
}

TreeSearch::Angles TreeSearch::getDirectionsFromIntervals(double curDir, const TreeSearch::AngleIntervals& intervals)
{
    std::vector< double > ret;

    double minStep = search_conf.angularSamplingMin;
    double maxStep = search_conf.angularSamplingMax;
    int minNodes = search_conf.angularSamplingNominalCount;
    
    // double size = intervals.size();
    // std::cout << "Interval vector size " << size << std::endl;
    bool straight = false;
    
    for (AngleIntervals::const_iterator it = intervals.begin(); it != intervals.end(); it++) 
    {
	double start = (it->first);
	double end  = (it->second);

        double intervalOpening;

	// Special case, wrapping interval
	if (start > end)
        {
            if ((curDir > start && curDir < end - 2 * M_PI) || (curDir < end && curDir + 2 * M_PI > start))
                straight = true;

            intervalOpening = end + 2 * M_PI - start;
        }
        else
        {
            if (curDir > start && curDir < end)
                straight = true;

            intervalOpening = end - start;
        }

        double step = intervalOpening / minNodes;
        if (step < minStep)
            step = minStep;
        else if (step > maxStep)
            step = maxStep;

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

    if (straight)
        ret.push_back(curDir);
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

    curNode->candidate_it = expandCandidates.insert(std::make_pair(0, curNode));
    
    int max_depth = search_conf.maxTreeSize;
    base::Time startTime = base::Time::now();
    
    while(!expandCandidates.empty()) 
    {
        curNode = expandCandidates.begin()->second;
        expandCandidates.erase(expandCandidates.begin());
        curNode->candidate_it = expandCandidates.end();

	if(!search_conf.maxSeekTime.isNull() && base::Time::now() - startTime > search_conf.maxSeekTime)
	    break;
	
        if (!validateNode(*curNode))
        {
            curNode->heuristic = -1;
            continue;
        }

        bool terminal = isTerminalNode(*curNode);
        if (!curNode->updated_cost && updateCost(*curNode, terminal))
        {
            curNode->updated_cost = true;

            double hcost = curNode->getHeuristicCost();
            if (hcost > expandCandidates.begin()->first)
            {
                // reinsert in the candidate queue and start again
                curNode->candidate_it = expandCandidates.insert(std::make_pair(hcost, curNode));
                continue;
            }
        }

        if (terminal)
        {
            tree.setFinalNode(curNode);
            break;
        }

        base::Position p = curNode->getPose().position;
        if (max_depth > 0 && tree.getSize() > max_depth)
            continue;

        // Get possible ways to go out of this node
        AngleIntervals driveIntervals =
            getNextPossibleDirections(*curNode,
                    search_conf.obstacleSafetyDistance, search_conf.robotWidth);

        if (driveIntervals.empty())
            continue;

        Angles driveDirections =
            getDirectionsFromIntervals(curNode->getDirection(), driveIntervals);
        if (driveDirections.empty())
            continue;

        double curDiscount = pow(search_conf.discountFactor, curNode->getDepth());

        // Expand the node: add children in the directions returned by
        // driveDirections
        for (Angles::const_iterator it = driveDirections.begin(); it != driveDirections.end(); it++)
        {
            if (max_depth > 0 && tree.getSize() >= max_depth)
                break;

            const double curDirection(*it);

            //generate new node
            std::pair<base::Pose, bool> projected =
                getProjectedPose(*curNode, curDirection,
                        search_conf.stepDistance);
            if (!projected.second)
                continue;

            //compute cost for it
            double nodeCost = curDiscount * getCostForNode(projected.first, curDirection, *curNode);

            // Check that we are not doing the same work multiple times.
            //
            // searchNode should be used only here !
            TreeNode searchNode(projected.first, curDirection);
            double identity_threshold = search_conf.identityThreshold;
            std::pair<NNSearch::const_iterator, double> this_nn =
                kdtree.find_nearest(&searchNode, identity_threshold);
            if (this_nn.first != kdtree.end())
            {
                TreeNode const* closest_node   = *(this_nn.first);
                if (closest_node->getCost() <= nodeCost)
                {
                    // The existing node is better than this one from a cost
                    // point of view. Check that the direction is also the same
                    TreeNode const* closest_parent = closest_node->getParent();
                    double parent_d = (closest_parent->getPose().position - curNode->getPose().position).norm();
                    if (parent_d < identity_threshold)
                        continue;
                }
                else if (closest_node->candidate_it != expandCandidates.end())
                {
                    TreeNode const* closest_parent = closest_node->getParent();
                    double parent_d = (closest_parent->getPose().position - curNode->getPose().position).norm();
                    if (parent_d < identity_threshold)
                    {
                        // The existing node is worse than this one, but we are
                        // lucky: the node has not been expanded yet. Just remove it
                        // from expandCandidates
                        closest_node->candidate_it->second->setHeuristic(-2.0);
                        expandCandidates.erase(closest_node->candidate_it);
                        closest_node->candidate_it = expandCandidates.end();
                        kdtree.erase(this_nn.first);
                    }
                }
            }

            // Finally, create the new node and add it in the tree
            TreeNode *newNode = tree.createChild(curNode, projected.first, curDirection);
            newNode->setCost(curNode->getCost() + nodeCost);
            newNode->setPositionTolerance(search_conf.obstacleSafetyDistance);
            newNode->setHeadingTolerance(std::numeric_limits< double >::signaling_NaN());
            newNode->setHeuristic(curDiscount * getHeuristic(*newNode));

            // Add it to the expand list
            newNode->candidate_it = expandCandidates.insert(std::make_pair(newNode->getHeuristicCost(), newNode));

            // And add it to the kdtree
            kdtree.insert(newNode);
        }
    }

    curNode = tree.getFinalNode();
    if (curNode)
    {
        std::cerr << "TreeSearch: found solution at c=" << curNode->getCost() << std::endl;
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

bool TreeSearch::updateCost(TreeNode& node, bool is_terminal) const
{
    return false;
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
    child->updated_cost = false;
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
        std::cerr << "WARN: the chosen heuristic does not seem to be a minorant" << std::endl;
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
    , updated_cost(false)
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

