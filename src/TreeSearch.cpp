#include "TreeSearch.h"
#include <Eigen/Core>
#include <map>
#include <stdexcept>

using namespace vfh_star;

TreeSearchConfiguration::TreeSearchConfiguration()
    : stepDistance(0.5)
    , searchDepth(5)
    , angularSampling(20)
    , discountFactor(0.8)
    , obstacleSafetyDistance(0.1)
    , robotWidth(0.5) {}

TreeSearch::TreeSearch()
{
}

void TreeSearch::setSearchConfiguration(const TreeSearchConfiguration& conf)
{
    this->search_conf = conf;
}

const TreeSearchConfiguration& TreeSearch::getSearchConfiguration() const
{
    return search_conf;
}

TreeSearch::Angles TreeSearch::getDirectionsFromIntervals(const TreeSearch::AngleIntervals& intervals)
{
    std::vector< double > ret;

    double angularSampling = 2 * M_PI / search_conf.angularSampling;
    
    // double size = intervals.size();
    // std::cout << "Interval vector size " << size << std::endl;
    
    for (AngleIntervals::const_iterator it = intervals.begin(); it != intervals.end(); it++) 
    {
	double start = (it->first);
	double end  = (it->second);

	//std::cout << "  sampling interval: start " << start << " end " << end << " with step=" << angularSampling << std::endl;

	// Special case, wrapping interval
	if (start > end)
	{
            double intervalOpening = end + 2 * M_PI - start;
            double intervalSize = floor(intervalOpening / angularSampling);
            double delta = (intervalOpening - (intervalSize * angularSampling)) / 2;
            for (int i = 0; i < intervalSize; ++i)
            {
                double angle = start + delta + i * angularSampling;
                if (angle > 2 * M_PI)
                    ret.push_back(angle - 2 * M_PI);
                else
                    ret.push_back(angle);
            }
	}
        else
        {
            double intervalOpening = end - start;
            double intervalSize = floor(intervalOpening / angularSampling);
            double delta = (intervalOpening - (intervalSize * angularSampling)) / 2;
            for (int i = 0; i < intervalSize; ++i)
                ret.push_back(start + delta + i * angularSampling);
        }
    }

    //std::cerr << "found " << ret.size() << " possible directions" << std::endl;
    
    return ret;
}

std::vector< base::Waypoint > TreeSearch::getTrajectory(const base::Pose& start)
{
    std::multimap<double, TreeNode *> expandCandidates;

    tree.clear();
    TreeNode *curNode = new TreeNode(start, TreeSearch::getHeading(start.orientation));
    tree.setRootNode(curNode);

    expandCandidates.insert(std::make_pair(0, curNode));
    
    while(!expandCandidates.empty()) 
    {
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
        // 	 std::cout << "Expanding node with dir " << curNode->getDirection() << " cost " << curNode->getCost() << " depth is " << curNode->getDepth() << " pos " << curNode->getPose().position.transpose() << " heading " << curNode->getDirection() << std::endl;


        base::Position p = curNode->getPose().position;
        //std::cout << "opened " << p.x() << ", " << p.y() << ", " << p.z() << " direction=" << curNode->getDirection() << "\n    depth=" << curNode->getDepth() << " cost=" << curNode->getCost() << " heuristic= " << curNode->getHeuristic() << "\n";
        if (isTerminalNode(*curNode))
            break;

        //get possible ways to go for node
        AngleIntervals driveIntervals =
            getNextPossibleDirections(curNode->getPose(),
                    search_conf.obstacleSafetyDistance, search_conf.robotWidth);

        Angles driveDirection =
            getDirectionsFromIntervals(driveIntervals);

        double curDiscount = pow(search_conf.discountFactor, curNode->getDepth());

        //expand node
        for(Angles::const_iterator it = driveDirection.begin(); it != driveDirection.end(); it++)
        {
            const double curDirection(*it);

            //generate new node
            base::Pose newPose = getProjectedPose(curNode->getPose(), curDirection, search_conf.stepDistance);
            TreeNode *newNode = new TreeNode(newPose, curDirection);

            //add Node to tree
            tree.addChild(curNode, newNode);

            //compute cost for it
            double nodeCost = getCostForNode(*newNode);

            nodeCost = curDiscount * nodeCost;
            newNode->setCost(curNode->getCost() + nodeCost);
            newNode->setPositionTolerance(search_conf.obstacleSafetyDistance);
            newNode->setHeadingTolerance(std::numeric_limits< double >::signaling_NaN());

            //heuristic
            newNode->setHeuristic(curDiscount * getHeuristic(*newNode));
            expandCandidates.insert(std::make_pair(newNode->getHeuristicCost(), newNode));

            // 	    std::cout << "Adding node for direction " << *it << " cost " << newNode->getCost() << " depth is " << newNode->getDepth() << " pos " << curNode->getPose().position.transpose() << " heading " << curNode->getDirection() << std::endl;
        }
    }

    tree.verifyHeuristicConsistency(curNode);
    return tree.buildTrajectoryTo(curNode);
}

const Tree& TreeSearch::getTree() const
{
    return tree;
}

double TreeSearch::getHeading(const Eigen::Quaterniond &orientation)
{
    return orientation.toRotationMatrix().eulerAngles(2,1,0)[0];
}

TreeSearch::~TreeSearch()
{

}

Tree::Tree()
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

    nodes.clear();

    std::map<TreeNode const*, TreeNode*> node_map;
    for (std::list<TreeNode*>::const_iterator it = other.nodes.begin();
            it != other.nodes.end(); ++it)
    {
        TreeNode* orig_node = *it;
        TreeNode* new_node =
            new TreeNode(orig_node->getPose(), orig_node->getDirection());
        new_node->setCost(orig_node->getCost());
        new_node->setHeuristic(orig_node->getHeuristic());
        new_node->setHeadingTolerance(orig_node->getHeadingTolerance());
        new_node->setPositionTolerance(orig_node->getPositionTolerance());

        node_map.insert( std::make_pair(orig_node, new_node) );
        addChild(node_map[orig_node->getParent()], new_node);
    }
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
        wp.heading  = TreeSearch::getHeading(p.orientation);
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
    return nodes.front();
}

void Tree::setRootNode(TreeNode* root)
{
    if (!nodes.empty())
        throw std::runtime_error("trying to change the root node of an non-empty tree");

    nodes.push_front(root);
}
void Tree::addChild(TreeNode* parent, TreeNode* child)
{
    child->parent = parent;
    child->depth  = parent->depth + 1;
    nodes.push_back(child);
}

const std::vector<TreeNode *>& Tree::getChildren(TreeNode* parent)
{
    return parent->children;
}

TreeNode *Tree::getParent(TreeNode* child)
{
    return child->parent;
}

int Tree::getSize() const
{
    return nodes.size();
}

void Tree::clear()
{
    for (std::list<TreeNode*>::iterator it = nodes.begin(); it != nodes.end(); ++it)
        delete *it;
    nodes.clear();
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

TreeNode::TreeNode(): parent(this), direction(0), cost(0), heuristic(0), depth(0)
{

}

TreeNode::TreeNode(const base::Pose& pose, double dir): parent(this), pose(pose), direction(dir), cost(0), heuristic(0), depth(0), positionTolerance(0), headingTolerance(0)
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
    return children.empty();
}

double TreeNode::getDirection() const
{
    return direction;
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

