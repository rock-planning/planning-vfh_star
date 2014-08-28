#include "NNLookup.hpp"

namespace vfh_star {
    
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
//      for(std::vector<NNLookupBox *>::iterator it2 = it->begin(); it2 != it->end(); it2++)
//          *it2 = NULL;
    }
    usedBoxes.clear();
}

bool NNLookup::getIndex(const TreeNode& node, int& x, int& y)
{
    //calculate position in global grid
    x = floor(node.getPosition().x() / boxSize) + curSizeHalf;
    y = floor(node.getPosition().y() / boxSize) + curSizeHalf;

    if((x < 0) || (x >= curSize) || (y < 0) || (y >= curSize))
        return false;

    return true;
}

TreeNode* NNLookup::getNodeWithinBounds(const TreeNode& node)
{
    int x,y;
    if(!getIndex(node, x, y))
        return NULL;
    
    NNLookupBox *box = globalGrid[x][y];
    if(box == NULL)
        return NULL;
    
    return box->getNearestNode(node);
}


void NNLookup::clearIfSame(const TreeNode* node)
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
        if(!getIndex(*node, x, y))
        {
            throw std::runtime_error("Internal Error, max was increased but node is still out of map. This should never happen");
        }
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

}