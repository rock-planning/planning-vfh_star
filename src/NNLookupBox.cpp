#include "NNLookupBox.hpp"

namespace vfh_star {


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

bool NNLookupBox::getIndixes(const TreeNode &node, int& x, int& y, int& a) const
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

void NNLookupBox::clearIfSame(const TreeNode* node)
{
    int x, y, a;
    bool valid = getIndixes(*node, x, y, a);
    if(!valid)
        throw std::runtime_error("NNLookup::clearIfSame:Error, accessed node outside of lookup box");

    assert(node->getDriveModeNr() < maxDriveModes);
    if(hashMap[x][y][a][node->getDriveModeNr()] == node)
        hashMap[x][y][a][node->getDriveModeNr()] = NULL;
}

TreeNode* NNLookupBox::getNearestNode(const TreeNode& node)
{
    int x, y, a;
    bool valid = getIndixes(node, x, y, a);
    if(!valid)
        throw std::runtime_error("NNLookup::getNearestNode:Error, accessed node outside of lookup box");
    
    assert(node.getDriveModeNr() < maxDriveModes);
    TreeNode *ret = hashMap[x][y][a][node.getDriveModeNr()];
    return ret;
}

void NNLookupBox::setNode(TreeNode* node)
{
    int x, y, a;
    bool valid = getIndixes(*node, x, y, a);
    if(!valid)
        throw std::runtime_error("NNLookup::setNode:Error, accessed node outside of lookup box");
    
    assert(node->getDriveModeNr() < maxDriveModes);
    hashMap[x][y][a][node->getDriveModeNr()] = node;
}

}