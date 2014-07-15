#ifndef NNLOOKUP_H
#define NNLOOKUP_H

#include <stdint.h>
#include <list>
#include "TreeNode.hpp"
#include "NNLookupBox.hpp"

namespace vfh_star {
    
class NNLookup
{
public:
    NNLookup(double boxSize, double boxResolutionXY, double boxResolutionTheta, uint8_t maxDriveModes = 1);
    ~NNLookup();
    TreeNode *getNodeWithinBounds(const TreeNode &node);
    void clearIfSame(const TreeNode *node);
    void setNode(TreeNode *node);
    void clear();
    
private:
    bool getIndex(const TreeNode &node,  int& x, int& y);
    void extendGlobalGrid(int newSize);
    int curSize;
    int curSizeHalf;

    double boxSize;
    double boxResolutionXY;
    double boxResolutionTheta;
    uint8_t maxDriveModes;
    
    std::vector<std::vector<NNLookupBox *> > globalGrid;
    std::list<NNLookupBox *> usedBoxes;
    std::list<NNLookupBox *> freeBoxes;
};

}
#endif // NNLOOKUP_H
