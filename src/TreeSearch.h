#ifndef VFHSTAR_TREESEARCH_H
#define VFHSTAR_TREESEARCH_H

#include <base/pose.h>
#include <base/waypoint.h>
#include <base/geometry/spline.h>
#include <vector>
#include <list>
#include <base/eigen.h>
#include <kdtree++/kdtree.hpp>
#include <map>

#include <vfh_star/Types.h>
#include <base/trajectory.h>

namespace vfh_star {

class TreeNode;
class NNLookupBox;

class NNLookup
{
public:
    NNLookup(double boxSize, double boxResolutionXY, double boxResolutionTheta);
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

    std::vector<std::vector<NNLookupBox *> > globalGrid;
    std::list<NNLookupBox *> usedBoxes;
    std::list<NNLookupBox *> freeBoxes;
};

class NNLookupBox
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    NNLookupBox(double resolutionXY, double angularResoultion, double size, const Eigen::Vector3d &centerPos);
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
    std::vector<std::vector<std::vector<TreeNode *> > > hashMap;
};
    
class TreeNode
{
    friend class Tree;
    friend class TreeSearch;
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	TreeNode();
	TreeNode(const base::Pose &pose, double dir);
        
        void clear();

        bool isRoot() const;
        bool isLeaf() const;
	
	const base::Pose &getPose() const;
        const TreeNode *getParent() const;
	
	void addChild(TreeNode *child);
	void removeChild(TreeNode *child);
	const std::vector<TreeNode *> &getChildren() const;
	
	double getDirection() const;
	int getDepth() const;

	double getYaw() const;
	const base::Vector3d getPosition() const;
	
        int getIndex() const;

	double getCost() const;
	void setCost(double value);
	double getHeuristic() const;
	void setHeuristic(double value);
        double getHeuristicCost() const;

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

	double yaw;
	
	///direction, that was choosen, that lead to this node
	double direction;
	
	///cost from start to this node
	double cost;
	
	///heuristic from node to goal
	double heuristic;
	
	///cost from parent to this node
	double costFromParent;
	
	int depth;
        int index;
        bool updated_cost;

        double positionTolerance;
        double headingTolerance;

	std::vector<TreeNode *> childs;
	
        // Used by TreeSearch only
        mutable std::multimap<double, TreeNode *>::iterator candidate_it;
};

class Tree
{
        friend class TreeSearch;

    public:
	Tree();
	~Tree();

        Tree(Tree const& other);
        Tree& operator = (Tree const& other);
	
        TreeNode* createRoot(base::Pose const& pose, double dir);
        TreeNode* createNode(base::Pose const& pose, double dir);
	TreeNode* createChild(TreeNode *parent, base::Pose const& pose, double dir);

	TreeNode *getParent(TreeNode *child);
        const TreeNode *getRootNode() const;
        TreeNode *getRootNode();

        std::vector<base::Waypoint> buildTrajectoryTo(TreeNode const* leaf) const;
	std::vector<base::Trajectory> buildTrajectoriesTo(TreeNode const* leaf, const Eigen::Affine3d &body2Trajectory) const;
        std::vector<base::Trajectory> buildTrajectoriesTo(std::vector<const vfh_star::TreeNode *> nodes, const Eigen::Affine3d &body2Trajectory) const;

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

    private:
	void copyNodeChilds(const vfh_star::TreeNode* otherNode, vfh_star::TreeNode* ownNode, const vfh_star::Tree& other);

        /** A tree might get quite big, in which case calling nodes.size() is
         * really not efficient. Since it is trivial to keep the size uptodate,
         * just do it
         */
        int size; 

        /** The set of tree nodes */
        std::list<TreeNode> nodes;

        /** The set of tree nodes */
        std::list<TreeNode *> free_nodes;

        /** The final node (0 if none has been found) */
        TreeNode* final_node;
	
	///Root node of the tree
	TreeNode *root_node;
};

/** The basic search algorithm used for planning */
class TreeSearch
{
    public:
        typedef std::vector<double> Angles;
        typedef std::vector< std::pair<double, double> > AngleIntervals;

	TreeSearch();
        virtual ~TreeSearch();

        static base::geometry::Spline<3> waypointsToSpline(const std::vector<base::Waypoint>& waypoints);

        /** This is a version of getWaypoints where the returned set of
         * waypoints is converted into a spline
         */
        base::geometry::Spline<3> getTrajectory(const base::Pose& start);

        /** Computes the optimal path as a set of waypoints
         */
        std::vector<base::Waypoint> getWaypoints(const base::Pose& start);

        /** Generates a search tree that reaches the desired goal, and returns
         * the goal node
         */
        TreeNode const* compute(const base::Pose& start);

        void setSearchConf(const TreeSearchConf& conf);
        const TreeSearchConf& getSearchConf() const;
        Tree const& getTree() const;

    protected:
	Angles getDirectionsFromIntervals(double curDir, const AngleIntervals& intervals);

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
	virtual double getCostForNode(const base::Pose& p,
                double direction, const TreeNode& parentNode) const = 0;

	/**
	* This method returns possible directions where 
	* the robot can drive to, from the given position
        *
        * The returned vector is a list of angle intervals
	**/
	virtual AngleIntervals getNextPossibleDirections(
                const TreeNode& curNode,
                double obstacleSafetyDist,
                double robotWidth) const = 0;

	/**
	* This function returns a pose in which the robot would
	* be if he would have driven towards the given direction.
	* This method should take the robot driving constrains
	* into account. 
        *
        * The boolean in the pair is true if a next pose existed given the
        * provided parameters, and false otherwise. The distance between the
        * returned pose and the curNode's pose must be UP TO \c distance, but
        * can be lower in case of e.g. obstacles.
	*/
	virtual std::pair<base::Pose, bool> getProjectedPose(const TreeNode& curNode,
                double heading,
                double distance) const = 0;

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

    private:
	
	void updateNodeCosts(TreeNode *node);
	void removeSubtreeFromSearch(TreeNode *node);
	
	std::multimap<double, TreeNode *> expandCandidates;
	NNLookup *nnLookup;
};
} // vfh_star namespace

#endif // VFHSTAR_H

