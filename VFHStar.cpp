#include "VFHStar.h"
#include <Eigen/Core>
#include <map>

VFHStar::VFHStar()
{
    stepDistance = 0.1;
    maxTreeDepth = 5;
}

std::vector< double > VFHStar::getDirectionsFromIntervals(const std::vector< std::pair< double, double > >& intervals, double heading)
{
    std::vector< double > ret;
    double start;
    double end;
    
    double size = intervals.size();
    
    if(heading < 0)
	heading += 2*M_PI;
    
    std::cout << "Interval vector size " << size << std::endl;
    
    std::cout << "Heading is " << heading << std::endl;
    
    for(std::vector< std::pair<double, double> >::const_iterator it = intervals.begin(); it != intervals.end(); it++) 
    {
	start = (it->first);
	end = (it->second);

	std::cout << "Testing for interval: start " << start << " end " << end << std::endl;

	//special case, every direction is free
	if(start == 0 && end == 2*M_PI)
	{
	    std::cout << "All free took heading" << std::endl;
	    ret.push_back(heading);
	    continue;
	}
	//Special case, wrapping interval
	if(start > end)
	{
	    std::cout << "Wrapping interval" << std::endl;

	    if(heading >0 && heading < end)
	    {
		std::cout << "Heading within zero and end took heading" << std::endl;
		ret.push_back(heading);
		continue;
	    }
	    
	    if(start < heading && heading < 2*M_PI)
	    {
		std::cout << "Heading within start and 2*PI took heading" << std::endl;
		ret.push_back(heading);
		continue;
	    }
	}
	
	//special case, our main direction of travel lies 
	//within the interval
	if(start < heading && heading < end)
	{
	    ret.push_back(heading);
	    std::cout << "Heading within start and end took heading" << std::endl;
	    continue;
	}
	
	//special case narrow opening
	if(start == end)
	{
	    ret.push_back(start);
	    continue;
	}
	
	//both ends of the interval are candidates for travel
	ret.push_back(start);
	ret.push_back(end);
	std::cout << "Added start " << start << " and end " << end << std::endl;
    }
    
    return ret;
}

std::vector< base::Waypoint > VFHStar::getTrajectory(const base::Pose& start, double heading, double lastDrivenDirection)
{
    std::vector< base::Waypoint > finalTrajectory; 
    Tree tree;
    double lastDirection = lastDrivenDirection;
    
    double discountFactor = 0.8;
    
    double headingWeight = 5;
    double orientationWeight = 2;
    double directionWeight = 2;
    
    bool primaryNode = true;
    
    std::multimap<double, TreeNode *> expandCandidates;

    TreeNode *curNode = new TreeNode(start, lastDirection);
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
	
	//get the cheapest node for expansion
	 curNode = expandCandidates.begin()->second;
	 
	 expandCandidates.erase(expandCandidates.begin());
// 	 std::cout << "Expanding node with dir " << curNode->getDirection() << " cost " << curNode->getCost() << " depth is " << curNode->getDepth() << " pos " << curNode->getPose().position.transpose() << " heading " << curNode->getDirection() << std::endl;


	if(curNode->getDepth() >= maxTreeDepth)
	    break;
	
	//get possible ways to go for node
	std::vector< std::pair<double, double> > driveIntervals = getNextPossibleDirections(curNode->getPose());

	std::vector<double> driveDirection = getDirectionsFromIntervals(driveIntervals, heading);
	
	double curDiscount = pow(discountFactor, curNode->getDepth() + 1);
	
	//expand node
	for(std::vector<double>::const_iterator it = driveDirection.begin(); it != driveDirection.end(); it++)
	{
	    const double &curDirection(*it);
	    
	    //generate new node
	    TreeNode *newNode = new TreeNode(getProjectedPose(curNode->getPose(), curDirection, stepDistance), curDirection);
	    
	    //compute cost for it
	    double nodeCost = getCostForNode(*newNode, heading, primaryNode, headingWeight, orientationWeight, directionWeight);
	    
	    if(!primaryNode)
		nodeCost = curDiscount * nodeCost;
	    
	    newNode->getCost() = curNode->getCost() + nodeCost;
	    
	    //add Node to tree
	    tree.addChild(curNode, newNode);
	    
	    //heurestic
	    newNode->getHeurestic() = curNode->getHeurestic() + curDiscount * getHeurestic(*newNode, heading, headingWeight, orientationWeight, directionWeight);
	    
	    expandCandidates.insert(std::make_pair(newNode->getHeurestic(), newNode));
	    
// 	    std::cout << "Adding node for direction " << *it << " cost " << newNode->getCost() << " depth is " << newNode->getDepth() << " pos " << curNode->getPose().position.transpose() << " heading " << curNode->getDirection() << std::endl;
	}
	
	primaryNode = false;
    }
    
    const TreeNode *pNode = curNode;
    
    std::vector<base::Pose> tmp;
    
    //build trajectory for goal node
    while(1) 
    {
	//suboptimal but ok
	tmp.push_back(pNode->getPose());
	
	//check if we reached the root node
	if(pNode->getParent() == pNode)
	    break;
	
	pNode = pNode->getParent();
    }
    
    //FIXME this is a bad place for this parameter
    double obstacleSafetyDist = 0.05;
    base::Waypoint wp;
    for(std::vector<base::Pose>::reverse_iterator it = tmp.rbegin(); it != tmp.rend(); it++) 
    {
	wp.heading = it->orientation.toRotationMatrix().eulerAngles(2,1,0)[0];
	wp.position = it->position;
	wp.tol_position = obstacleSafetyDist;
	//TODO make a parameter
	wp.tol_heading = 0.1;
	finalTrajectory.push_back(wp);
    }
    
    return finalTrajectory;
}

double VFHStar::getHeading(const Eigen::Quaterniond &orientation) const
{
    return orientation.toRotationMatrix().eulerAngles(2,1,0)[0];
}

double VFHStar::getMotionDirection(const Eigen::Vector3d &start, const Eigen::Vector3d &end) const
{
    //note, we define y as our zero heading
    return atan2(end.y() - start.y(), end.x() - end.y()) + M_PI / 2.0;
}

double VFHStar::angleDiff(const double &a1, const double &a2) const
{
    double d = a1-a2;
    return std::min(std::min(fabs(d), fabs(d - M_PI*2)),fabs(d + M_PI*2));
}

double VFHStar::getHeurestic(const TreeNode &node, double headingDirection, double headingWeight, double orientationWeight, double directionWeight) const
{
    return orientationWeight * angleDiff(headingDirection,  getHeading(node.getParent()->getPose().orientation)) + directionWeight * angleDiff(headingDirection, node.getParent()->getDirection());
};

double VFHStar::getCostForNode(const TreeNode& curNode, double headingDirection, bool primary, double headingWeight, double orientationWeight, double directionWeight) const
{
    /**
    * cost is build from three factors:
    * a * difference of direction and heading direction
    * b * difference of direction and current Robot orientation
    * c * difference of direction and previouse motion orientation
    *
    * direction means the direction that was selected, in the previouse step, thus leading to this node
    */

    const double a = headingWeight;
    const double b = orientationWeight;
    const double c = directionWeight;
    
    const double direction = curNode.getDirection();
    
    double aPart = angleDiff(direction, headingDirection);
    
    if(!primary) 
    {
	aPart = std::max(aPart, angleDiff(headingDirection, getMotionDirection(curNode.getParent()->getPose().position, curNode.getPose().position)));
    }
    
    const double bPart = angleDiff(direction, getHeading(curNode.getParent()->getPose().orientation));
    const double cPart = angleDiff(direction, curNode.getParent()->getDirection());
    
    const double cost = a * aPart + b * bPart + c * cPart;
    
    return cost;
}

VFHStar::~VFHStar()
{

}

Tree::Tree()
{

}

Tree::~Tree()
{

}

TreeNode::TreeNode(): parent(this), direction(0), cost(0), heuristic(0), leafDepth(0)
{

}

TreeNode::TreeNode(const base::Pose& pose, const double& dir): parent(this), pose(pose), direction(dir), cost(0), heuristic(0), leafDepth(0)
{

}

double TreeNode::getHeurestic() const
{
    return heuristic;
}

double& TreeNode::getHeurestic()
{
    return heuristic;
}

double& TreeNode::getCost()
{
    return cost;
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
    return leafDepth;
}

TreeNode* Tree::getRootNode()
{
    return root;
}

void Tree::setRootNode(TreeNode* root)
{
    this->root = root;
}

double TreeNode::getDirection() const
{
    return direction;
}

void Tree::addChild(TreeNode* parent, TreeNode* child)
{
    parent->childs.push_back(child);
    child->parent = parent;
    child->leafDepth = parent->leafDepth + 1;
}

void Tree::removeChild(TreeNode* parent, TreeNode* child)
{
    for(std::vector<TreeNode *>::iterator it = parent->childs.begin(); it != parent->childs.end(); it++) {
	if(*it == child)
	{
	    parent->childs.erase(it);
	    break;
	}
    }
}

const std::vector<TreeNode *>& Tree::getChilds(TreeNode* parent)
{
    return parent->childs;
}

TreeNode *Tree::getParent(TreeNode* child)
{
    return child->parent;
}
