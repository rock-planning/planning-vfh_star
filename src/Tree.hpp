#ifndef TREE_H
#define TREE_H

#include <Eigen/Core>
#include <list>
#include "TreeNode.hpp"
#include "Types.h"

namespace vfh_star {

class Tree
{
        friend class TreeSearch;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Tree();
        ~Tree();

        Tree(Tree const& other);
        Tree& operator = (Tree const& other);
        
        TreeNode* createRoot(const base::Pose& pose, const base::Angle& dir);
        TreeNode* createNode(const base::Pose& pose, const base::Angle& dir);
        TreeNode* createChild(TreeNode* parent, const base::Pose& pose, const base::Angle& dir);

        TreeNode *getParent(TreeNode *child);
        const TreeNode *getRootNode() const;
        TreeNode *getRootNode();

        int getSize() const;
        void clear();
        void verifyHeuristicConsistency(const TreeNode* from) const;
        std::list<TreeNode> const& getNodes() const;
        void setFinalNode(TreeNode* node);
        TreeNode* getFinalNode() const;

        void reserve(int size);
        
        /**
         * Removes the node from the tree.
         * Internally it will be added to the free list for reuse.
         * 
         * The caller is responsible for making sure that the node
         * is not referenced somewere in the tree. 
         * */
        void removeNode(TreeNode *node);

        void setTreeToWorld(Eigen::Affine3d tree2World);
        
        const Eigen::Affine3d &getTreeToWorld() const;

    private:
        void copyNodeChilds(const TreeNode* otherNode, TreeNode* ownNode, const Tree& other);

        /** A tree might get quite big, in which case calling nodes.size() is
         * really not efficient. Since it is trivial to keep the size uptodate,
         * just do it
         */
        int size; 

        /**
         * Node Storage
         * */
        std::list<TreeNode> nodes;

        /**
         * Iterator pointing to the next node in there
         * storage that is 'free'
         * */
        std::list<TreeNode>::iterator nextNodePos;

        /**
         * Number of unused nodes in the storage.
         * */
        size_t nodesLeftInStorage;

        /**
         * Temporary storage for nodes that were
         * removed from the tree.
         * */
        std::list<TreeNode *> free_nodes;

        /** The final node (0 if none has been found) */
        TreeNode* final_node;
        
        ///Root node of the tree
        TreeNode *root_node;
        
        ///Pose of tree in world frame, only needed for displaying
        Eigen::Affine3d tree2World;

        DebugTree *debugTree;
};

    
}

#endif // TREE_H
