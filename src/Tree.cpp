#include "Tree.hpp"

namespace vfh_star {
    
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

TreeNode *Tree::getParent(TreeNode* child)
{
    return child->parent;
}

}