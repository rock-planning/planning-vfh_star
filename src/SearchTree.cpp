#include <vfh_star/SearchTree.h>

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

double TreeNode::getHeuristic() const
{
    return heuristic;
}

double& TreeNode::getHeuristic()
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
    parent->children.push_back(child);
    child->parent = parent;
    child->leafDepth = parent->leafDepth + 1;
}

void Tree::removeChild(TreeNode* parent, TreeNode* child)
{
    for(std::vector<TreeNode *>::iterator it = parent->children.begin(); it != parent->children.end(); it++) {
	if(*it == child)
	{
	    parent->children.erase(it);
	    break;
	}
    }
}

const std::vector<TreeNode *>& Tree::getChildren(TreeNode* parent)
{
    return parent->children;
}

TreeNode *Tree::getParent(TreeNode* child)
{
    return child->parent;
}

