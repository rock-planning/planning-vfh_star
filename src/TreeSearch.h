#ifndef VFHSTAR_TREESEARCH_H
#define VFHSTAR_TREESEARCH_H

#include <base/Pose.hpp>
#include <base/Waypoint.hpp>
#include <base/geometry/Spline.hpp>
#include <vector>
#include <list>
#include <base/Eigen.hpp>
#include <map>

#include <vfh_star/Types.h>
#include <base/Trajectory.hpp>
#include <base/Angle.hpp>
#include "Types.h"

namespace vfh_star {

class TreeNode;
class NNLookupBox;

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
    bool getIndixes(const vfh_star::TreeNode &node, int& x, int& y, int& a) const;
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
    
class TreeNode
{
    friend class Tree;
    friend class TreeSearch;
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	TreeNode();
	TreeNode(const base::Pose &pose, const base::Angle &dir, uint8_t driveMode);
        
        void clear();

        bool isRoot() const;
        bool isLeaf() const;
	
	const base::Pose &getPose() const;
        const TreeNode *getParent() const;
	
	void addChild(TreeNode *child);
	void removeChild(TreeNode *child);
	const std::vector<TreeNode *> &getChildren() const;
	
	const base::Angle &getDirection() const;
	int getDepth() const;

	const base::Angle &getYaw() const;
	const base::Vector3d &getPosition() const;
	
        int getIndex() const;

	double getCost() const;
	void setCost(double value);
	double getHeuristic() const;
	void setHeuristic(double value);
        double getHeuristicCost() const;
        
        void setDriveMode(uint8_t mode);
        uint8_t getDriveMode() const;

	void setCostFromParent(double value);
	double getCostFromParent() const;
	
        double getPositionTolerance() const;
        void setPositionTolerance(double tol);
        double getHeadingTolerance() const;
        void setHeadingTolerance(double tol);
	
    private:
	TreeNode *parent;
        bool is_leaf;
	
	///pose of the node, note the orientation of the node and the direction may differ, 
	///because of kinematic constrains of the robot
	base::Pose pose;

        ///yaw of the pose
	base::Angle yaw;
	
	///direction, that was choosen, that lead to this node
	base::Angle direction;
	
	///cost from start to this node
	double cost;
	
	///heuristic from node to goal
	double heuristic;
	
	///cost from parent to this node
	double costFromParent;
	
        ///the drive mode that was used to get to the current position
        uint8_t driveMode;
        
	int depth;
        int index;
        bool updated_cost;

        float positionTolerance;
        float headingTolerance;

	std::vector<TreeNode *> childs;
	
        // Used by TreeSearch only
        mutable std::multimap<double, TreeNode *>::iterator candidate_it;
};

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
	TreeNode* createChild(vfh_star::TreeNode* parent, const base::Pose& pose, const base::Angle& dir);

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
	void copyNodeChilds(const vfh_star::TreeNode* otherNode, vfh_star::TreeNode* ownNode, const vfh_star::Tree& other);

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

};

class ProjectedPose 
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        ProjectedPose(): nextPoseExists(true), driveMode(0), angleTurned(0) {};
        base::Pose pose;
        bool nextPoseExists;
        uint8_t driveMode;
        double angleTurned;
};

class DriveMode
{
public:
    /**
     * Sets the parameters of this drive mode on the trajectory
     * */
    virtual void setTrajectoryParameters(base::Trajectory &tr) const = 0;
    
    /**
     * Returns the pose in which the robot will be after traveling from the curNode
     * over a distance of 'distance' towards the moveDirection, using this drive mode.
     * 
     * returns false if pose could not be projected using this drive mode
     * */
    virtual bool projectPose(ProjectedPose &result, const TreeNode& curNode, const base::Angle& moveDirection, double distance) const = 0;
    
    /**
     * Returns the cost of driving from the parentNode to the projected position using this drive mode;
     * */
    virtual double getCostForNode(const ProjectedPose& projection,const base::Angle &direction, const TreeNode& parentNode) const = 0;
};

/** The basic search algorithm used for planning */
class TreeSearch
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef std::vector<base::Angle> Angles;
        typedef std::vector<base::AngleSegment> AngleIntervals;

	TreeSearch();
        virtual ~TreeSearch();

        std::vector<base::Trajectory> buildTrajectoriesTo(TreeNode const* leaf, const Eigen::Affine3d &world2Trajectory) const;
        std::vector<base::Trajectory> buildTrajectoriesTo(std::vector<const vfh_star::TreeNode *> nodes, const Eigen::Affine3d &world2Trajectory) const;


        void setSearchConf(const TreeSearchConf& conf);
        const TreeSearchConf& getSearchConf() const;
        Tree const& getTree() const;

        /**
         * This method is supposed to be called every time the 
         * config changed. It will drop all cached nodes etc.
         * to make shure that the new config is used later on.
         * */
        void configChanged();
        
        /**
         * Transformation from the node tree to the world
         * coordinate system. 
         * */
        void setTreeToWorld(Eigen::Affine3d tree2World);
        
        const Eigen::Affine3d &getTreeToWorld() const;
        
    protected:
        /** Generates a search tree that reaches the desired goal, and returns
         * the goal node
         * Warning, the returned treeNode is in tree cooridinates,
         * not in world coordinates
         */
        TreeNode const* compute(const base::Pose& start_world);

        
	Angles getDirectionsFromIntervals(const base::Angle &curDir, const AngleIntervals& intervals);

        // The tree generated at the last call to getTrajectory
        Tree tree;
        TreeSearchConf search_conf;
	
        /** Returns true if the given node is a terminal node, i.e. if it
         * reached the goal
         */
        virtual bool isTerminalNode(const TreeNode& node) const = 0;

        /** Returns the estimated cost from the given node to the optimal node
         * reachable from that node. Note that this estimate must be a minorant,
         * i.e. must be smaller or equal than the actual value
         */
	virtual double getHeuristic(const TreeNode &node) const = 0;

        /** Returns the cost of travelling from \c node's parent to \c node
         * itself. It might include a cost of "being at" \c node as well
         */
	double getCostForNode(const ProjectedPose &projection,
                const base::Angle &direction, const TreeNode& parentNode);

	/**
	* This method returns possible directions where 
	* the robot can drive to, from the given position.
        * The returned directions are given as intervals
        * of drivable angles. The intervals must be in map frame. 
        *
        * The returned vector is a list of angle intervals
	**/
	virtual AngleIntervals getNextPossibleDirections(
                const TreeNode& curNode) const = 0;

                

        /**
        * Project the curNode to new node candidat using all registered drive modes. 
        */
	std::vector<ProjectedPose> getProjectedPoses(const TreeNode& curNode,
                const base::Angle &heading,
                double distance);

        /**
         * This function is called to validate a node that has previously been
         * projected with getProjectedPose. It will get called only on nodes
         * that get developped, so it called a lot less than getProjectedPose.
         * Overload it only if it makes sense that some expensive tests in
         * getProjectedPose get moved there.
         *
         * The default implementation returns true (valid).
         */
        virtual bool validateNode(const TreeNode& node) const;

        /** This function is called after validateNode to check if the node cost
         * should be changed. The only valid change is upwards
         *
         * The method should return true if the cost has been changed, and false
         * otherwise. The is_final flag is true if the node is a terminal node
         * and false otherwise. It might be used to add some heuristic cost on
         * the final states.
         *
         * The default implementation does nothing and returns false
         */
        virtual bool updateCost(TreeNode& node, bool is_terminal) const;

        /**
         * Adds a drive mode to the planner
         * */
        void addDriveMode(vfh_star::DriveMode &driveMode);
        
        void clearDriveModes();
    private:
        void addDirections(vfh_star::TreeSearch::Angles& directions, const base::AngleSegment &segement, const double minStep, const double maxStep, const int minNodes) const;
	void updateNodeCosts(TreeNode *node);
	void removeSubtreeFromSearch(TreeNode *node);
        
	Eigen::Affine3d tree2World;
	
	std::multimap<double, TreeNode *> expandCandidates;
        std::vector<DriveMode *> driveModes;
	NNLookup *nnLookup;
};
} // vfh_star namespace

#endif // VFHSTAR_H

