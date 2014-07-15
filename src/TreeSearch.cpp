#include "TreeSearch.h"
#include <Eigen/Core>
#include <map>
#include <stdexcept>
#include <iostream>
#include <base/Angle.hpp>
#include <base/Float.hpp>

namespace vfh_star {

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
            std::cout << "Drivable Interval start " << *it << std::endl; 
        }
    }

    //sample the given intervals with the different sampling policies
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
            if(printDebug)
            {
                for(std::vector<base::AngleSegment>::const_iterator pit = intersections.begin(); pit != intersections.end(); pit++)
                {
                    std::cout << "Intersection is " << *pit << std::endl;
                }
            }
            
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
    return projection.driveMode->getCostForNode(projection, direction, parentNode);
}

std::vector< ProjectedPose > TreeSearch::getProjectedPoses(const TreeNode& curNode, const base::Angle& heading, double distance)
{
    int i = 0;
    std::vector< ProjectedPose > ret;
    for(std::vector<DriveMode *>::const_iterator it = driveModes.begin(); it != driveModes.end(); it++)
    {
        ProjectedPose newPose;
        if((*it)->projectPose(newPose, curNode, heading - curNode.getYaw() , distance))
        {
            newPose.driveMode = *it;
            newPose.driveModeNr = i;

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
    
    //FIXME what is the current drive mode ?
    curNode->setDriveModeNr(0);
    curNode->setDriveMode(driveModes.at(0));
    
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
        {
            std::cout << "Quitting planning as max search time was reached" << std::endl;
	    break;
        }
        
        if (!validateNode(*curNode))
        {
            if(tree.debugTree)
            {
                tree.debugTree->nodes[curNode->getIndex()].isValid = false;
            }
	    nnLookup->clearIfSame(curNode);
//             std::cout << "Node is invalid" << std::endl;
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
                TreeNode searchNode(projected->pose, curDirection, projected->driveMode, projected->driveModeNr);
                
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
                newNode->setDriveModeNr(projected->driveModeNr);
                newNode->setCost(curNode->getCost() + nodeCost);
                newNode->setCostFromParent(nodeCost);
                newNode->setPositionTolerance(std::numeric_limits< double >::signaling_NaN());
                newNode->setHeadingTolerance(std::numeric_limits< double >::signaling_NaN());
                newNode->setHeuristic(curDiscount * getHeuristic(*newNode));

                // Add it to the expand list
                newNode->candidate_it = expandCandidates.insert(std::make_pair(newNode->getHeuristicCost(), newNode));

//                 std::cout << "Added new node " << newNode->getPose().position.transpose() << " Yaw " << newNode->getYaw() << " Cost " << newNode->getCost() << " Heuristic " << newNode->getHeuristicCost() << std::endl;
                
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

std::vector< base::Trajectory > TreeSearch::buildTrajectoriesTo(const TreeNode* node, const Eigen::Affine3d& world2Trajectory) const
{
    if(!node)
        return std::vector< base::Trajectory >();
    
    std::vector<const TreeNode *> nodes;
    const TreeNode* nodeTmp = node;
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

std::vector< base::Trajectory > TreeSearch::buildTrajectoriesTo(std::vector<const TreeNode *> nodes, const Eigen::Affine3d &world2Trajectory) const
{    
    std::vector<base::Trajectory> result;
	
    if(nodes.empty())
	return result;
    
    std::vector<const TreeNode *>::const_iterator it = nodes.begin();
    std::vector<base::Vector3d> as_points;
    
    base::Angle posDir;
    base::Angle nodeDir;
    //HACK  we don't actuall know the current drive mode of the robot.
    //we set it to the drivemode of the first node of the generated
    //trajectory
    DriveMode const *lastDriveMode = 0;
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
        DriveMode const *curDriveMode = (*it)->getDriveMode();
                
	//check if direction changed
	if(lastDriveMode != curDriveMode)
	{
	    base::Trajectory tr;
            lastDriveMode->setTrajectoryParameters(tr);
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
    lastDriveMode->setTrajectoryParameters(tr);
	
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

}

