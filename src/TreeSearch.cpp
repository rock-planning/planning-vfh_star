#include "TreeSearch.h"
#include <Eigen/Core>
#include <map>
#include <stdexcept>
#include <iostream>
#include <base/Angle.hpp>
#include <base/Float.hpp>

using namespace vfh_star;

bool printDebug = false;
            
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
    
TreeSearch::TreeSearch(): tree2World(Eigen::Affine3d::Identity()), nnLookup(NULL)
{
    search_conf.computePosAndYawThreshold();
}

void TreeSearch::setTreeToWorld(Eigen::Affine3d tree2World)
{
    this->tree2World = tree2World;
    tree.setTreeToWorld(tree2World);
}

const Eigen::Affine3d& TreeSearch::getTreeToWorld() const
{
    return tree2World;
}

TreeSearch::~TreeSearch()
{
    delete nnLookup;
}

void TreeSearch::configChanged()
{
   //trigger update of nearest neighbour lookup 
    //will be reconstructed on next search
    delete nnLookup;
    nnLookup = 0;
}

void TreeSearch::setSearchConf(const TreeSearchConf& conf)
{
    this->search_conf = conf;
    search_conf.computePosAndYawThreshold();

    configChanged();
 }


const TreeSearchConf& TreeSearch::getSearchConf() const
{
    return search_conf;
}

void TreeSearch::addDirections(TreeSearch::Angles& directions, const base::AngleSegment& segment, const double minStep, const double maxStep, const int minNodes) const
{
    const double intervalOpening = segment.getWidth();
    
    double step = intervalOpening / minNodes;
    if (step < minStep)
        step = minStep;
    else if (step > maxStep)
        step = maxStep;

    int intervalSize = ceil(intervalOpening / step);
    base::Angle delta = base::Angle::fromRad(intervalOpening / intervalSize);
    base::Angle result = segment.getStart();
    for (int i = 0; i < intervalSize + 1; ++i)
    {
        directions.push_back(result);
        result += delta;
    }
}


TreeSearch::Angles TreeSearch::getDirectionsFromIntervals(const base::Angle &curDir, const TreeSearch::AngleIntervals& intervals)
{
    TreeSearch::Angles ret;
    
    if(printDebug)
    {
        for (AngleIntervals::const_iterator it = intervals.begin(); it != intervals.end(); it++) 
        {
            std::cout << "Drivable Interval start " << it->startRad << " end " << it->endRad << std::endl; 
        }
    }

    //sample the giben intervals with the different sampling policies
    for(std::vector<AngleSampleConf>::const_iterator it = search_conf.sampleAreas.begin(); it != search_conf.sampleAreas.end(); it++)
    {
        //create a sample interval aligned to the robot direction
        base::AngleSegment sampleInterval(base::Angle::fromRad(it->intervalStart) + curDir, it->intervalWidth);
        
        if(printDebug)
        {
            std::cout << "Sample interval is " << sampleInterval << std::endl;
        }        
        for (AngleIntervals::const_iterator it2 = intervals.begin(); it2 != intervals.end(); it2++) 
        {
            const base::AngleSegment &interval(*it2);

            std::vector<base::AngleSegment> intersections = sampleInterval.getIntersections(interval);
            for(std::vector<base::AngleSegment>::const_iterator it3 = intersections.begin(); it3 != intersections.end(); it3++)
            {
                addDirections(ret, *it3, it->angularSamplingMin, it->angularSamplingMax, it->angularSamplingNominalCount);
                
                if(it3->isInside(curDir))
                    ret.push_back(curDir);
            }
        }
    }

    if(printDebug)
    {
        std::cerr << "found " << ret.size() << " possible directions" << std::endl;
        for(Angles::iterator it = ret.begin(); it != ret.end(); it++)        
            std::cout << *it << std::endl;
    }
    
    return ret;
}

void TreeSearch::addDriveMode(DriveMode& driveMode)
{
    driveModes.push_back(&driveMode);
}

void TreeSearch::clearDriveModes()
{
    driveModes.clear();
}

double TreeSearch::getCostForNode(const ProjectedPose& projection, const base::Angle& direction, const TreeNode& parentNode)
{
    return driveModes[projection.driveMode]->getCostForNode(projection, direction, parentNode);
}

std::vector< ProjectedPose > TreeSearch::getProjectedPoses(const TreeNode& curNode, const base::Angle& heading, double distance)
{
    int i = 0;
    std::vector< ProjectedPose > ret;
    for(std::vector<DriveMode *>::const_iterator it = driveModes.begin(); it != driveModes.end(); it++)
    {
        ProjectedPose newPose;
        if((*it)->projectPose(newPose, curNode, heading, distance))
        {
            newPose.driveMode = i;
            ret.push_back(newPose);
        }
        i++;
    }
    return ret;
}

TreeNode const* TreeSearch::compute(const base::Pose& start_world)
{
    if(!driveModes.size())
    {
        throw std::runtime_error("TreeSearch:: Error, no drive mode was registered");
    }

    if(tree.debugTree)
    {
        tree.debugTree->nodes.clear();
        tree.debugTree->nodes.reserve(search_conf.maxTreeSize);
    }
    
    base::Pose start(tree2World.inverse() * start_world.toTransform());
    tree.clear();
    if(!nnLookup)
	nnLookup = new NNLookup(1.0, search_conf.identityPositionThreshold / 2.0 , search_conf.identityYawThreshold / 2.0, driveModes.size());
    
    nnLookup->clear();
    TreeNode *curNode = tree.createRoot(start, base::Angle::fromRad(start.getYaw()));
    curNode->setHeuristic(getHeuristic(*curNode));
    curNode->setCost(0.0);
    nnLookup->setNode(curNode);

    curNode->candidate_it = expandCandidates.insert(std::make_pair(curNode->getHeuristicCost(), curNode));
    
    int max_depth = search_conf.maxTreeSize;
    
    if(tree.debugTree)
    {
        tree.debugTree->finalNode = -1;
        tree.debugTree->startNode = curNode->getIndex();
        tree.debugTree->treePos = tree2World;
    }
    
    int candidateNr = 0;
    
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

        if(tree.debugTree)
        {
            tree.debugTree->nodes[curNode->getIndex()].expansionOrder = candidateNr;
        }
        candidateNr ++;
	
	if(!search_conf.maxSeekTime.isNull() && base::Time::now() - startTime > search_conf.maxSeekTime)
	    break;
	
        if (!validateNode(*curNode))
        {
            if(tree.debugTree)
            {
                tree.debugTree->nodes[curNode->getIndex()].isValid = false;
            }
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

//         printDebug = false;
//         if(curNode->getPosition().x() > 1.0 && curNode->getPosition().x() < 2.0)
//             ; //printDebug = true;
        
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
            std::vector<ProjectedPose> projectedPoses =
                getProjectedPoses(*curNode, curDirection,
                        search_conf.stepDistance);

            for(std::vector<ProjectedPose>::const_iterator projected = projectedPoses.begin(); projected != projectedPoses.end();projected++ )
            {

                if(!projected->nextPoseExists)
                    continue;

                //compute cost for it
                double nodeCost = curDiscount * getCostForNode(*projected, curDirection, *curNode);

                
                // Check that we are not doing the same work multiple times.
                //
                // searchNode should be used only here !
                TreeNode searchNode(projected->pose, curDirection, projected->driveMode);
                
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
                TreeNode *newNode = tree.createChild(curNode, projected->pose, curDirection);
                newNode->setDriveMode(projected->driveMode);
                newNode->setCost(curNode->getCost() + nodeCost);
                newNode->setCostFromParent(nodeCost);
                newNode->setPositionTolerance(std::numeric_limits< double >::signaling_NaN());
                newNode->setHeadingTolerance(std::numeric_limits< double >::signaling_NaN());
                newNode->setHeuristic(curDiscount * getHeuristic(*newNode));

                // Add it to the expand list
                newNode->candidate_it = expandCandidates.insert(std::make_pair(newNode->getHeuristicCost(), newNode));

                if(tree.debugTree)
                {
                    DebugNode &dbg(tree.debugTree->nodes[newNode->getIndex()]);
                    dbg.cost = newNode->getCost();
                }
                
                //add new node to nearest neighbour lookup
                nnLookup->setNode(newNode);
            }
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
    , tree2World(Eigen::Affine3d::Identity())
    , debugTree(0)
{
    clear();
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
    
    this->tree2World = other.tree2World;
//     std::cout << "Copied " << getSize() << " Nodes " << std::endl;
    
    return *this;
}

std::vector< base::Trajectory > TreeSearch::buildTrajectoriesTo(const vfh_star::TreeNode* node, const Eigen::Affine3d& world2Trajectory) const
{
    if(!node)
        return std::vector< base::Trajectory >();
    
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

    return buildTrajectoriesTo(nodes, world2Trajectory);
}

std::vector< base::Trajectory > TreeSearch::buildTrajectoriesTo(std::vector<const vfh_star::TreeNode *> nodes, const Eigen::Affine3d &world2Trajectory) const
{    
    std::vector<base::Trajectory> result;
	
    if(nodes.empty())
	return result;
    
    std::vector<const vfh_star::TreeNode *>::const_iterator it = nodes.begin();
    std::vector<base::Vector3d> as_points;
    
    base::Angle posDir;
    base::Angle nodeDir;
    //HACK  we don't actuall know the current drive mode of the robot.
    //we set it to the drivemode of the first node of the generated
    //trajectory
    int lastDriveMode = 0;
    if(!nodes.empty())
    {
	const Eigen::Affine3d body2Tree((*it)->getPose().toTransform());
        const Eigen::Affine3d tree2Trajectory(world2Trajectory * tree2World);
	const Eigen::Affine3d body2Trajectory(tree2Trajectory * body2Tree);
	as_points.push_back((body2Trajectory * Eigen::Vector3d(0,0,0)));
	it++;
        //HACK we don't actuall know the direction of the root node
        //we set it to the direction of the first node of the generated
        //trajectory
        lastDriveMode = (*it)->getDriveMode();
    }
  
    for(;it != nodes.end(); it++)
    {
        int curDriveMode = (*it)->getDriveMode();
                
	//check if direction changed
	if(lastDriveMode != curDriveMode)
	{
	    base::Trajectory tr;
            driveModes[lastDriveMode]->setTrajectoryParameters(tr);
	    tr.spline.interpolate(as_points);
	    result.push_back(tr);
	}

        const Eigen::Affine3d body2Tree((*it)->getPose().toTransform());
        const Eigen::Affine3d tree2Trajectory(world2Trajectory * tree2World);
        const Eigen::Affine3d body2Trajectory(tree2Trajectory * body2Tree);
        as_points.push_back((body2Trajectory * Eigen::Vector3d(0,0,0)));

	lastDriveMode = curDriveMode;
    }

    base::Trajectory tr;
    driveModes[lastDriveMode]->setTrajectoryParameters(tr);
	
    tr.spline.interpolate(as_points);
    result.push_back(tr);

    return result;
}

const DebugTree* TreeSearch::getDebugTree() const
{
    return tree.debugTree;
}

void TreeSearch::activateDebug()
{
    if(tree.debugTree)
        return;
    
    tree.debugTree = new DebugTree();
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
    if(debugTree)
    {
        debugTree->finalNode = final_node->index;
    }
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
        //use free node as long as possible
        if(nodesLeftInStorage)
        {
            n = &(*nextNodePos);
            nodesLeftInStorage--;
            nextNodePos++;
        }
        else
        {
            nodes.push_back(TreeNode());
            n = &nodes.back();
        }
    }

    n->clear();
    n->pose = pose;
    n->yaw = base::Angle::fromRad(pose.getYaw());
    n->direction = dir;
    n->index  = size;
    if(debugTree)
    {
        debugTree->nodes.push_back(DebugNode(size, pose));
    }
    
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
    
    if(debugTree)
    {
        DebugNode &dbgParent(debugTree->nodes[parent->index]);
        DebugNode &dbgChild(debugTree->nodes[child->index]);
        dbgChild.parent = parent->index;
        dbgParent.childs.push_back(child->index);
    }    
    return child;
}

void Tree::removeNode(TreeNode* node)
{
    if(debugTree)
    {
        debugTree->nodes[node->index].wasRemoved = true;
    }

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
    nodesLeftInStorage = nodes.size();
    nextNodePos = nodes.begin();
    final_node = 0;
    root_node = 0;
    size = 0;
}

const Eigen::Affine3d& Tree::getTreeToWorld() const
{
    return tree2World;
}

void Tree::setTreeToWorld(Eigen::Affine3d tree2World)
{
    this->tree2World = tree2World;
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

NNLookup::NNLookup(double boxSize, double boxResolutionXY, double boxResolutionTheta, uint8_t maxDriveModes): 
        curSize(0), curSizeHalf(0), boxSize(boxSize), boxResolutionXY(boxResolutionXY), 
        boxResolutionTheta(boxResolutionTheta), maxDriveModes(maxDriveModes)
{
    std::cout << "NNLookup created with boxSize " << boxSize << " boxResolutionXY " << boxResolutionXY << " boxResolutionTheta " << boxResolutionTheta << " maxDriveModes " << ((int) maxDriveModes) << std::endl;
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
        //ugly fast way to clear the vector
        memset(it->data(), 0, sizeof(NNLookupBox *) * it->size());
// 	for(std::vector<NNLookupBox *>::iterator it2 = it->begin(); it2 != it->end(); it2++)
// 	    *it2 = NULL;
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
	int newSizeHalf = std::max(abs(floor(node->getPosition().x() / boxSize)),
				abs(floor(node->getPosition().y() / boxSize))) + 1;
    	extendGlobalGrid(newSizeHalf);
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
	    box = new NNLookupBox(boxResolutionXY, boxResolutionTheta, boxSize, boxPos, maxDriveModes);
	}
	usedBoxes.push_back(box);
	globalGrid[x][y] = box;
    }
    
    box->setNode(node);
}


NNLookupBox::NNLookupBox(double resolutionXY, double angularResoultion, double size, const Eigen::Vector3d& centerPos, uint8_t maxDriveModes) : resolutionXY(resolutionXY), angularResolution(angularResoultion), size(size), maxDriveModes(maxDriveModes)
{
    xCells = size / resolutionXY + 1;
    yCells = xCells;
    aCells = M_PI / angularResoultion * 2 + 1;

    hashMap.resize(xCells);
    for(int x = 0; x < xCells; x++)
    {
	hashMap[x].resize(yCells);
	for(int y = 0; y < yCells; y++)
	{
	    hashMap[x][y].resize(aCells);
            for(int a = 0; a < aCells; a++)
            {
                hashMap[x][y][a].resize(maxDriveModes, NULL);
            }
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
                for(int dm= 0; dm < maxDriveModes; dm++)
                    hashMap[x][y][a][dm] = NULL;
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

    assert(node->getDriveMode() < maxDriveModes);
    if(hashMap[x][y][a][node->getDriveMode()] == node)
	hashMap[x][y][a][node->getDriveMode()] = NULL;
}

TreeNode* NNLookupBox::getNearestNode(const vfh_star::TreeNode& node)
{
    int x, y, a;
    bool valid = getIndixes(node, x, y, a);
    if(!valid)
	throw std::runtime_error("NNLookup::getNearestNode:Error, accessed node outside of lookup box");
    
    assert(node.getDriveMode() < maxDriveModes);
    TreeNode *ret = hashMap[x][y][a][node.getDriveMode()];
    return ret;
}

void NNLookupBox::setNode(TreeNode* node)
{
    int x, y, a;
    bool valid = getIndixes(*node, x, y, a);
    if(!valid)
	throw std::runtime_error("NNLookup::setNode:Error, accessed node outside of lookup box");
    
    assert(node->getDriveMode() < maxDriveModes);
    hashMap[x][y][a][node->getDriveMode()] = node;
}


TreeNode::TreeNode()
{
    clear();
}

TreeNode::TreeNode(const base::Pose& pose, const base::Angle &dir, uint8_t driveMode)
    : parent(this)
    , is_leaf(true)
    , pose(pose)
    , yaw(base::Angle::fromRad(pose.getYaw()))
    , direction(dir)
    , cost(0)
    , heuristic(0)
    , costFromParent(0)
    , driveMode(driveMode)
    , depth(0)
    , index(0)
    , updated_cost(false)
    , positionTolerance(0)
    , headingTolerance(0)
{
    clear();
    direction = dir;
    this->pose = pose;
//     yaw = pose.getYaw();
    this->driveMode=driveMode;
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
    costFromParent = 0;
    driveMode = 0;
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

uint8_t TreeNode::getDriveMode() const
{
    return driveMode;
}

void TreeNode::setDriveMode(uint8_t mode)
{
    driveMode = mode;
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

