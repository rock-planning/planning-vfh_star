#include "TreeSearch.h"
#include <Eigen/Core>
#include <map>
#include <stdexcept>
#include <iostream>
#include <base/angle.h>
#include <base/float.h>

using namespace vfh_star;

bool printDebug = false;;
            
void TreeSearchConf::computePosAndYawThreshold()
{
    if(identityPositionThreshold < 0)
    {
        identityPositionThreshold = stepDistance / 5.0;
    }
    
    if(identityYawThreshold < 0)
    {
        identityYawThreshold = 3.0 * 180.0 / M_PI;
    }

}
    
    
TreeSearch::TreeSearch(): nnLookup(NULL)
{
    search_conf.computePosAndYawThreshold();
}

TreeSearch::~TreeSearch()
{
    delete nnLookup;
}

void TreeSearch::setSearchConf(const TreeSearchConf& conf)
{
    this->search_conf = conf;
    search_conf.computePosAndYawThreshold();

    //trigger update of nearest neighbour lookup 
    //will be reconstructed on next search
    delete nnLookup;
    nnLookup = 0;
}

const TreeSearchConf& TreeSearch::getSearchConf() const
{
    return search_conf;
}

void TreeSearch::addDirections(TreeSearch::Angles &directions, const base::Angle &start, const base::Angle &end, const double minStep, const double maxStep, const int minNodes) const
{
    const double intervalOpening = (end - start).getRad();
    
    double step = intervalOpening / minNodes;
    if (step < minStep)
        step = minStep;
    else if (step > maxStep)
        step = maxStep;

    int intervalSize = floor(intervalOpening / step);
    base::Angle delta = base::Angle::fromRad(step);
    base::Angle result = start;
    for (int i = 0; i < intervalSize + 1; ++i)
    {
        directions.push_back(result);
        result += delta;
    }
}


TreeSearch::Angles TreeSearch::getDirectionsFromIntervals(const base::Angle &curDir, const TreeSearch::AngleIntervals& intervals)
{
    TreeSearch::Angles ret;

    const double minStep = search_conf.directionSampleConf.angularSamplingMin;
    const double maxStep = search_conf.directionSampleConf.angularSamplingMax;
    const int minNodes = search_conf.directionSampleConf.angularSamplingNominalCount;
    
    // double size = intervals.size();
    // std::cout << "Interval vector size " << size << std::endl;
    bool straight = false;

    const base::Angle frontIntervalHalf = base::Angle::fromRad(search_conf.oversamplingWidth / 2.0);
    
    base::Angle frontIntervalStart = base::Angle::fromRad(-search_conf.oversamplingWidth / 2.0);
    base::Angle frontIntervalEnd = frontIntervalHalf;
    
//     std::cout << std::endl;
//     std::cout << std::endl;
//     std::cout << "CurDir " << curDir<< std::endl;
//     
    base::Angle frontDirection;
    
    if(printDebug)
    {
        for (AngleIntervals::const_iterator it = intervals.begin(); it != intervals.end(); it++) 
        {
            std::cout << "Interval start " << it->startRad << " end " << it->endRad << std::endl; 
        }
    }
    
    for (AngleIntervals::const_iterator it = intervals.begin(); it != intervals.end(); it++) 
    {
        const base::AngleSegment &interval(*it);

        if(interval.isInside(frontDirection))
            straight = true;
        
        bool foundFront = false;
        
        if(interval.isInside(frontIntervalStart))
        {
            foundFront = true;

            if(interval.isInside(frontIntervalEnd))
            {
                if(printDebug)
                    std::cout << "Whole Front " << frontIntervalStart << " " << frontIntervalEnd << " is in range " << start << " " << end << std::endl;
                //whole front interval is inside the current interval
                addDirections(ret, frontIntervalStart, frontIntervalEnd, 
                                                        search_conf.directionOversampleConf.angularSamplingMin, 
                                                        search_conf.directionOversampleConf.angularSamplingMin,
                                                        search_conf.directionOversampleConf.angularSamplingNominalCount);
            }
            else
            {
                if(printDebug)
                    std::cout << "Start of Front " << frontIntervalStart << " " << frontIntervalEnd << " is in range " << start << " " << end << std::endl;
                //frontIntervalStart to end is the search interval
                addDirections(ret, frontIntervalStart, end, search_conf.directionOversampleConf.angularSamplingMin, 
                                                            search_conf.directionOversampleConf.angularSamplingMin,
                                                            search_conf.directionOversampleConf.angularSamplingNominalCount);
            }
        }
        else
        {
            if(interval.isInside(frontIntervalEnd))
            {
                if(printDebug)
                    std::cout << "End of Front " << frontIntervalStart << " " << frontIntervalEnd << " is in range " << start << " " << end << std::endl;

                //only end of front in interval
                addDirections(ret, start, frontIntervalEnd, search_conf.directionOversampleConf.angularSamplingMin, 
                                                            search_conf.directionOversampleConf.angularSamplingMin,
                                                            search_conf.directionOversampleConf.angularSamplingNominalCount);
                foundFront = true;
            }
            else
            {
                if(start.isInRange(frontIntervalStart, frontIntervalEnd) && end.isInRange(frontIntervalStart, frontIntervalEnd))
                {
                    if(printDebug)
                        std::cout << "Intervall is bigger " << frontIntervalStart << " " << frontIntervalEnd << " is in range " << start << " " << end << std::endl;

                    //whole interval is smaller than front interval
                    addDirections(ret, start, end, search_conf.directionOversampleConf.angularSamplingMin, 
                                                search_conf.directionOversampleConf.angularSamplingMin,
                                                search_conf.directionOversampleConf.angularSamplingNominalCount);

                    continue;
                }
            }
        }

        
        double intervalOpening = end.getRad() - start.getRad();
        if(intervalOpening < 0)
            intervalOpening += 2*M_PI;
        
        //FIXME, this is dangerous
        if(end.getRad() == 0 && start.getRad() == 0)
            intervalOpening = 2 * M_PI;
        
        double step = intervalOpening / minNodes;
        if (step < minStep)
            step = minStep;
        else if (step > maxStep)
            step = maxStep;
                
        int intervalSize = floor(intervalOpening / step);
        base::Angle delta = base::Angle::fromRad(step);
//         std::cout << "Delta is " << delta << std::endl;
                if(printDebug)
        {
            std::cout << "intervalOpening " << intervalOpening << " step " << step << std::endl;
            std::cout << "intervalSize " << intervalSize << " delta " << delta <<std::endl; 
        }

        base::Angle result = start;
        for (int i = 0; i < intervalSize + 1; ++i)
        {
            //skip angles in the oversampling area
            if(foundFront && result.isInRange(frontIntervalStart, frontIntervalEnd))
            {
                if(printDebug)
                    std::cout << "Skipping " << result << std::endl;
            }
            else
            {
                if(printDebug)
                    std::cout << "Adding dir " << result << std::endl;
                ret.push_back(result);
            }            
            result +=delta;
        }
    }

    if (straight)
        ret.push_back(frontDirection);
    
    if(printDebug)
    {

        std::cerr << "found " << ret.size() << " possible directions" << std::endl;
        for(Angles::iterator it = ret.begin(); it != ret.end(); it++)        
            std::cout << *it << std::endl;
    }
    
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
    tree.clear();
    if(!nnLookup)
	nnLookup = new NNLookup(1.0, search_conf.identityPositionThreshold / 2.0 , search_conf.identityYawThreshold / 2.0);
    
    nnLookup->clear();
    TreeNode *curNode = tree.createRoot(start, base::Angle::fromRad(start.getYaw()));
    curNode->setHeuristic(getHeuristic(*curNode));
    curNode->setCost(0.0);
    nnLookup->setNode(curNode);

    curNode->candidate_it = expandCandidates.insert(std::make_pair(curNode->getHeuristicCost(), curNode));
    
    int max_depth = search_conf.maxTreeSize;
    
    base::Time startTime = base::Time::now();
    
    while(!expandCandidates.empty()) 
    {
        curNode = expandCandidates.begin()->second;
	if(curNode->getHeuristicCost() != expandCandidates.begin()->first)
	{
	    std::cout << "Warning map is mixed up " << curNode->getHeuristicCost() << " " << expandCandidates.begin()->first << std::endl;
	    std::cout << curNode->getPosition().transpose() << " Ori " << curNode->getYaw() << std::endl;
	}

// 	std::cout << "Expanding " << curNode->getPose().position.transpose() << " " << " val in queue " << expandCandidates.begin()->first << " Cost " << curNode->getCost() << " HC " << curNode->getHeuristic() << std::endl; 
        expandCandidates.erase(expandCandidates.begin());
        curNode->candidate_it = expandCandidates.end();


	
	if(!search_conf.maxSeekTime.isNull() && base::Time::now() - startTime > search_conf.maxSeekTime)
	    break;
	
        if (!validateNode(*curNode))
        {
            curNode->heuristic = -1;
	    nnLookup->clearIfSame(curNode);
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

        if (max_depth > 0 && tree.getSize() > max_depth)
            continue;

        printDebug = false;
        if(curNode->getPosition().x() > 1.0 && curNode->getPosition().x() < 2.0)
            ; //printDebug = true;
        
        // Get possible ways to go out of this node
        AngleIntervals driveIntervals =
            getNextPossibleDirections(*curNode);

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

            const base::Angle &curDirection(*it);

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
	    
	    const double searchNodeCost = nodeCost + curNode->getCost();
	    TreeNode *closest_node = nnLookup->getNodeWithinBounds(searchNode);
	    if(closest_node)
	    {
		if(closest_node->getCost() <= searchNodeCost)
		{
		    //Existing node is better than current node
		    //discard the current node
		    continue;
		} 
		else
		{
                    //remove from parent
                    closest_node->parent->removeChild(closest_node);
                    
                    //remove closest node and subnodes
                    removeSubtreeFromSearch(closest_node);                    
		}
	    }
	    
            // Finally, create the new node and add it in the tree
            TreeNode *newNode = tree.createChild(curNode, projected.first, curDirection);
            newNode->setCost(curNode->getCost() + nodeCost);
	    newNode->setCostFromParent(nodeCost);
            newNode->setPositionTolerance(std::numeric_limits< double >::signaling_NaN());
            newNode->setHeadingTolerance(std::numeric_limits< double >::signaling_NaN());
            newNode->setHeuristic(curDiscount * getHeuristic(*newNode));

            // Add it to the expand list
            newNode->candidate_it = expandCandidates.insert(std::make_pair(newNode->getHeuristicCost(), newNode));

//             std::cout << "Expanded Node " << newNode->getPosition().transpose() << " cost " << newNode->getCost() << " heuristic " << newNode->getHeuristic() << std::endl; 

	    //add new node to nearest neighbour lookup
	    nnLookup->setNode(newNode);
        }
    }

    std::cout << "Created " << tree.getSize() << " Nodes " << " cur usage " << tree.nodes.size() << std::endl;
    expandCandidates.clear();
    
    curNode = tree.getFinalNode();
       
    if (curNode)
    {
        std::cerr << "TreeSearch: found solution at c=" << curNode->getCost() << std::endl;
        tree.verifyHeuristicConsistency(curNode);
        return curNode;
    }
    return 0;
}

void TreeSearch::updateNodeCosts(TreeNode* node)
{
    const std::vector<TreeNode *> &childs(node->getChildren());
    
    for(std::vector<TreeNode *>::const_iterator it = childs.begin(); it != childs.end();it++)
    {
	const double costToNode = (*it)->getCostFromParent();
	(*it)->setCost(node->getCost() + costToNode);
	//if this node is in the expand list remove and reenter it
	//so that the position in the queue gets updated
	if((*it)->candidate_it != expandCandidates.end())
	{
	    expandCandidates.erase((*it)->candidate_it);
	    (*it)->candidate_it = expandCandidates.insert(std::make_pair((*it)->getHeuristicCost(), (*it)));
	};
	
	updateNodeCosts(*it);
    }
}

void TreeSearch::removeSubtreeFromSearch(TreeNode* node)
{
    if(node->candidate_it != expandCandidates.end())
    {
        expandCandidates.erase(node->candidate_it);
        node->candidate_it = expandCandidates.end();
    };
    
    nnLookup->clearIfSame(node);
    
    const std::vector<TreeNode *> &childs(node->getChildren());
    
    for(std::vector<TreeNode *>::const_iterator it = childs.begin(); it != childs.end();it++)
    {
	removeSubtreeFromSearch(*it);
    }
    //all children are invalid now we can just clear them
    node->childs.clear();
    
    tree.removeNode(node);
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

Tree::Tree()
    : size(0)
    , final_node(0)
    , root_node(0)    
{
}

Tree::~Tree()
{
}

Tree::Tree(Tree const& other)
{
    *this = other;
}

void Tree::copyNodeChilds(const TreeNode* otherNode, TreeNode *ownNode, Tree const& other)
{
    for(std::vector<TreeNode *>::const_iterator it = otherNode->getChildren().begin();
	it != otherNode->getChildren().end(); it++)
	{
	    TreeNode const* orig_node = *it;
	    TreeNode* new_node = createChild(ownNode, orig_node->getPose(), orig_node->getDirection());
	    new_node->setCost(orig_node->getCost());
	    new_node->setHeuristic(orig_node->getHeuristic());
	    new_node->setHeadingTolerance(orig_node->getHeadingTolerance());
	    new_node->setPositionTolerance(orig_node->getPositionTolerance());

	    if(other.getFinalNode() == orig_node)
		setFinalNode(new_node);

	    copyNodeChilds(*it, new_node, other);
	}
}

Tree& Tree::operator = (Tree const& other)
{
    if (this == &other)
        return *this;

    clear();

    if(other.root_node)
    {
        root_node = createRoot(other.root_node->getPose(), other.root_node->getDirection());    
        copyNodeChilds(other.root_node, root_node, other);
    }
    
//     std::cout << "Copied " << getSize() << " Nodes " << std::endl;
    
    return *this;
}

std::vector< base::Trajectory > Tree::buildTrajectoriesTo(const vfh_star::TreeNode* node, const Eigen::Affine3d& body2Trajectory) const
{
    std::vector<const vfh_star::TreeNode *> nodes;
    const vfh_star::TreeNode* nodeTmp = node;
    int size = node->getDepth() + 1;
    for (int i = 0; i < size; ++i)
    {
	nodes.insert(nodes.begin(), nodeTmp);
	if (nodeTmp->isRoot() && i != size - 1)
            throw std::runtime_error("internal error in buildTrajectoryTo: found a root node even though the trajectory is not finished");
	nodeTmp = nodeTmp->getParent();
    }    

    return buildTrajectoriesTo(nodes, body2Trajectory);
}

std::vector< base::Trajectory > Tree::buildTrajectoriesTo(std::vector<const vfh_star::TreeNode *> nodes, const Eigen::Affine3d &body2Trajectory) const
{    
    std::vector<base::Trajectory> result;
	
    if(nodes.empty())
	return result;
    
    std::vector<const vfh_star::TreeNode *>::const_iterator it = nodes.begin();
    std::vector<base::Vector3d> as_points;
    
    base::Angle posDir;
    base::Angle nodeDir;
    bool lastNodeIsForward = true;
    if(!nodes.empty())
    {
	posDir = (*it)->getYaw();
	nodeDir = (*it)->getDirection();
	lastNodeIsForward = fabs((posDir - nodeDir).rad) < 4.0/5.0 * M_PI;

	const Eigen::Affine3d body2Planner((*it)->getPose().toTransform());
	const Eigen::Affine3d trajectory2Planner(body2Planner * body2Trajectory.inverse());
	as_points.push_back((trajectory2Planner * Eigen::Vector3d(0,0,0)));
	it++;

    }
 
    //steering angles that are backwards means we are trying to drive backwards
    base::Angle backIntervalStart = base::Angle::fromRad( - M_PI/2.0);
    base::Angle backIntervalEnd = base::Angle::fromRad( M_PI/2.0);;
 
    for(;it != nodes.end(); it++)
    {
	const vfh_star::TreeNode *curNode = *it;
	bool curNodeIsForward = curNode->getDirection().isInRange(backIntervalStart, backIntervalEnd);
        
	//check if direction changed
	if(lastNodeIsForward != curNodeIsForward)
	{
	    base::Trajectory tr;
	    if(lastNodeIsForward)
		//forward
		tr.speed = 1.0;
	    else
		tr.speed = -1.0;
		
	    tr.spline.interpolate(as_points);
	    
	    result.push_back(tr);
	}

	const Eigen::Affine3d body2Planner((*it)->getPose().toTransform());
	const Eigen::Affine3d trajectory2Planner(body2Planner * body2Trajectory.inverse());
	as_points.push_back((trajectory2Planner * Eigen::Vector3d(0,0,0)));

	lastNodeIsForward = curNodeIsForward;
    }

    base::Trajectory tr;
    if(lastNodeIsForward)
	//forward
	tr.speed = 1.0;
    else
	//backwards
	tr.speed = -1.0;
	
    tr.spline.interpolate(as_points);
    result.push_back(tr);

    return result;
}

std::vector<base::Waypoint> Tree::buildTrajectoryTo(TreeNode const* node) const
{
    std::vector< base::Waypoint > result; 
    
    if(!node)
        return result;
    
    int size = node->getDepth() + 1;
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

const TreeNode* Tree::getRootNode() const
{
    return root_node;
}

TreeNode* Tree::getRootNode()
{
    return root_node;
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
    int diff = nodes.size() - size;
    for (int i = 0; i < diff; ++i)
        nodes.push_back(TreeNode());
}

TreeNode* Tree::createNode(base::Pose const& pose, const base::Angle &dir)
{
    TreeNode* n;
    if (!free_nodes.empty())
    {
        n = free_nodes.front();
        free_nodes.pop_front();
    }
    else
    {
        nodes.push_back(TreeNode());
        n = &nodes.back();
    }

    n->clear();
    n->pose = pose;
    n->yaw = base::Angle::fromRad(pose.getYaw());
    n->direction = dir;
    n->index  = size;
    ++size;
    return n;
}

TreeNode* Tree::createRoot(base::Pose const& pose, const base::Angle &dir)
{
    if (root_node)
        throw std::runtime_error("trying to create a root node of an non-empty tree");

    root_node = createNode(pose, dir);
    
    return root_node;
}

TreeNode* Tree::createChild(TreeNode* parent, base::Pose const& pose, const base::Angle &dir)
{
    TreeNode* child = createNode(pose, dir);
    child->depth  = parent->depth + 1;
    parent->addChild(child);
    return child;
}

void Tree::removeNode(TreeNode* node)
{
    node->childs.clear();
    free_nodes.push_back(node);
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
    free_nodes.clear();
    for(std::list<TreeNode>::iterator it = nodes.begin(); it != nodes.end(); it++)
    {
        free_nodes.push_back(&(*it));
    }
    final_node = 0;
    root_node = 0;
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
        if (node->getHeuristicCost() > actual_cost && fabs(node->getHeuristicCost() - actual_cost) > 0.0001)
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

NNLookup::NNLookup(double boxSize, double boxResolutionXY, double boxResolutionTheta): curSize(0), curSizeHalf(0), boxSize(boxSize), boxResolutionXY(boxResolutionXY), boxResolutionTheta(boxResolutionTheta)
{
}

NNLookup::~NNLookup()
{
    for(std::list<NNLookupBox *>::const_iterator it = freeBoxes.begin(); it != freeBoxes.end(); it++)
    {
        delete *it;
    }

    for(std::list<NNLookupBox *>::const_iterator it = usedBoxes.begin(); it != usedBoxes.end(); it++)
    {
        delete *it;
    }
}

void NNLookup::clear()
{
    freeBoxes.splice(freeBoxes.begin(), usedBoxes);
    for(std::vector<std::vector<NNLookupBox *> >::iterator it = globalGrid.begin(); it != globalGrid.end(); it++)
    {
	for(std::vector<NNLookupBox *>::iterator it2 = it->begin(); it2 != it->end(); it2++)
	    *it2 = NULL;
    }
    usedBoxes.clear();
}

bool NNLookup::getIndex(const vfh_star::TreeNode& node, int& x, int& y)
{
    //calculate position in global grid
    x = floor(node.getPosition().x() / boxSize) + curSizeHalf;
    y = floor(node.getPosition().y() / boxSize) + curSizeHalf;

    if((x < 0) || (x >= curSize) || (y < 0) || (y >= curSize))
	return false;

    return true;
}

TreeNode* NNLookup::getNodeWithinBounds(const vfh_star::TreeNode& node)
{
    int x,y;
    if(!getIndex(node, x, y))
	return NULL;
    
    NNLookupBox *box = globalGrid[x][y];
    if(box == NULL)
	return NULL;
    
    return box->getNearestNode(node);
}


void NNLookup::clearIfSame(const vfh_star::TreeNode* node)
{
    int x,y;
    if(!getIndex(*node, x, y))
	return;
    
    NNLookupBox *box = globalGrid[x][y];
    if(box == NULL)
	return;

    box->clearIfSame(node);
}

void NNLookup::extendGlobalGrid(int requestedSize)
{
    const int oldSize = curSize;
    int newSize = requestedSize * 2;
    curSize = newSize;
    curSizeHalf = requestedSize;
    globalGrid.resize(newSize);
    
    std::vector<std::vector<NNLookupBox *> >newGrid;
    newGrid.resize(newSize);    
    for(std::vector<std::vector<NNLookupBox *> >:: iterator it = newGrid.begin(); it != newGrid.end(); it++)
    {
	(*it).resize(newSize, NULL);
    }
    
    const int diffHalf = (newSize - oldSize) / 2;
    
    for(int x = 0; x < oldSize; x++)
    {
        const int newX = x + diffHalf;
        for(int y = 0; y < oldSize; y++)
        {
            const int newY = y + diffHalf;
            newGrid[newX][newY] = globalGrid[x][y];
        }
    }
    
    globalGrid.swap(newGrid);
    
}


void NNLookup::setNode(TreeNode* node)
{
    int x,y;
    if(!getIndex(*node, x, y))
    {
	extendGlobalGrid(std::max(abs(x),abs(y)) + 1);
	assert(getIndex(*node, x, y));
    }

    NNLookupBox *box = globalGrid[x][y];
    if(box == NULL)
    {
	const Eigen::Vector3d boxPos = Eigen::Vector3d(floor(node->getPosition().x()),
						 floor(node->getPosition().y()),
						0) + Eigen::Vector3d(boxSize / 2.0, boxSize / 2.0, 0);
                                                
	if(!freeBoxes.empty())
	{
	    box = (freeBoxes.front());
	    freeBoxes.pop_front();
	    box->clear();
	    box->setNewPosition(boxPos);
	}
	else
	{
	    box = new NNLookupBox(boxResolutionXY, boxResolutionTheta, boxSize, boxPos);
	}
	usedBoxes.push_back(box);
	globalGrid[x][y] = box;
    }
    
    box->setNode(node);
}


NNLookupBox::NNLookupBox(double resolutionXY, double angularReosultion, double size, const Eigen::Vector3d& centerPos) : resolutionXY(resolutionXY), angularResolution(angularReosultion), size(size)
{
    xCells = size / resolutionXY + 1;
    yCells = xCells;
    aCells = M_PI / angularReosultion * 2 + 1;

    hashMap.resize(xCells);
    for(int x = 0; x < xCells; x++)
    {
	hashMap[x].resize(yCells);
	for(int y = 0; y < yCells; y++)
	{
	    hashMap[x][y].resize(aCells, NULL);
	}
    }
    
    toWorld = centerPos - Eigen::Vector3d(size/2.0, size/2.0,0);
}

void NNLookupBox::setNewPosition(const Eigen::Vector3d& centerPos)
{
    toWorld = centerPos - Eigen::Vector3d(size/2.0, size/2.0,0);
}

void NNLookupBox::clear()
{
    for(int x = 0; x < xCells; x++)
    {
	for(int y = 0; y < yCells; y++)
	{
	    for(int a = 0; a < aCells; a++)
	    {
		hashMap[x][y][a] = NULL;
	    }
	}
    }
}

bool NNLookupBox::getIndixes(const vfh_star::TreeNode &node, int& x, int& y, int& a) const
{
    const Eigen::Vector3d mapPos = node.getPosition() - toWorld;
//     std::cout << "NodePos " << node.getPosition().transpose() << " mapPos " << mapPos.transpose() << std::endl;
    x = mapPos.x() / resolutionXY;
    y = mapPos.y() / resolutionXY;
    a = node.getYaw().getRad() / angularResolution;
    if(a < 0)
	a+= M_PI/angularResolution * 2.0;

//     std::cout << "X " << x << " Y " << y << " A " << a << std::endl;
    
    assert((x >= 0) && (x < xCells) &&
	    (y >= 0) && (y < yCells) &&
	    (a >= 0) && (a < aCells));
    
    return true;
}

void NNLookupBox::clearIfSame(const vfh_star::TreeNode* node)
{
    int x, y, a;
    bool valid = getIndixes(*node, x, y, a);
    if(!valid)
	throw std::runtime_error("NNLookup::clearIfSame:Error, accessed node outside of lookup box");

    if(hashMap[x][y][a] == node)
	hashMap[x][y][a] = NULL;
}

TreeNode* NNLookupBox::getNearestNode(const vfh_star::TreeNode& node)
{
    int x, y, a;
    bool valid = getIndixes(node, x, y, a);
    if(!valid)
	throw std::runtime_error("NNLookup::getNearestNode:Error, accessed node outside of lookup box");
    
    TreeNode *ret = hashMap[x][y][a];
    return ret;
}

void NNLookupBox::setNode(TreeNode* node)
{
    int x, y, a;
    bool valid = getIndixes(*node, x, y, a);
    if(!valid)
	throw std::runtime_error("NNLookup::setNode:Error, accessed node outside of lookup box");
    
    hashMap[x][y][a] = node;
}


TreeNode::TreeNode()
{
    clear();
}

TreeNode::TreeNode(const base::Pose& pose, base::Angle dir)
{
    clear();
    direction = dir;
    this->pose = pose;
    yaw = base::Angle::fromRad(pose.getYaw());
}

void TreeNode::clear()
{
    parent = this;
    pose = base::Pose();
    yaw = base::Angle();
    is_leaf = true;
    cost = 0;
    heuristic = 0;
    depth = 0;
    index = 0;
    updated_cost = false;
    positionTolerance = 0;
    headingTolerance = 0;
    direction = base::Angle();
    childs.clear();
}

const base::Vector3d &TreeNode::getPosition() const
{
    return pose.position;
}

const base::Angle &TreeNode::getYaw() const
{
    return yaw;
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

void TreeNode::setCostFromParent(double value)
{
    costFromParent = value;
}

double TreeNode::getCostFromParent() const
{
    return costFromParent;
}

void TreeNode::addChild(TreeNode* child)
{
    is_leaf = false;
    child->parent = this;
    childs.push_back(child);
}

void TreeNode::removeChild(TreeNode* child)
{
    std::vector<TreeNode *>::iterator it = std::find(childs.begin(), childs.end(), child);
    if(it != childs.end())
    {
	childs.erase(it);
    }
    if(childs.empty())
        is_leaf = true;
}

const std::vector< TreeNode* >& TreeNode::getChildren() const
{
    return childs;
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

const base::Angle &TreeNode::getDirection() const
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

