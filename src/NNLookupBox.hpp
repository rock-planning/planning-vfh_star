#ifndef NNLOOKUPBOX_H
#define NNLOOKUPBOX_H

#include "TreeNode.hpp"

namespace vfh_star {
    
class NNLookupBox
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    NNLookupBox(double resolutionXY, double angularResoultion, double size, const Eigen::Vector3d &centerPos, uint8_t maxDriveModes);
    TreeNode *getNearestNode(const TreeNode &node);
    void clearIfSame(const TreeNode *node);
    void setNode(TreeNode *node);
    void clear();
    void setNewPosition(const Eigen::Vector3d &centerPos);
    
private:
    bool getIndixes(const TreeNode &node, int& x, int& y, int& a) const;
    int xCells;
    int yCells;
    int aCells;
    Eigen::Vector3d toWorld;
    double resolutionXY;
    double angularResolution;
    double size;
    uint8_t maxDriveModes;
    std::vector<std::vector<std::vector<std::vector<TreeNode *> > > > hashMap;
};
    

}

#endif // NNLOOKUPBOX_H
