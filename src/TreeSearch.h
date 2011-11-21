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

namespace vfh_star {
class TreeNode
{
    friend class Tree;
    friend class TreeSearch;
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	TreeNode();
	TreeNode(const base::Pose &pose, double dir);

        bool isRoot() const;
        bool isLeaf() const;
	
	const base::Pose &getPose() const;
	const TreeNode *getParent() const;
	double getDirection() const;
	int getDepth() const;

        int getIndex() const;

	double getCost() const;
	void setCost(double value);
	double getHeuristic() const;
	void setHeuristic(double value);
        double getHeuristicCost() const;

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

	///direction, that was choosen, that lead to this node
	double direction;
	double cost;
	double heuristic;
	int depth;
        int index;
        bool updated_cost;

        double positionTolerance;
        double headingTolerance;

        // Used by TreeSearch only
        mutable std::multimap<double, TreeNode *>::iterator candidate_it;
};

class Tree
{
    public:
	Tree();
	~Tree();

        Tree(Tree const& other);
        Tree& operator = (Tree const& other);

        TreeNode* createRoot(base::Pose const& pose, double dir);
	TreeNode* createChild(TreeNode *parent, base::Pose const& pose, double dir);

	TreeNode *getParent(TreeNode *child);
	TreeNode *getRootNode();

        std::vector<base::Waypoint> buildTrajectoryTo(TreeNode const* leaf) const;

        int getSize() const;
        void clear();
        void reserve();
        void verifyHeuristicConsistency(const TreeNode* from) const;
        std::list<TreeNode> const& getNodes() const;
        void setFinalNode(TreeNode* node);
        TreeNode* getFinalNode() const;

        void reserve(int size);

    private:
        TreeNode* createNode(base::Pose const& pose, double dir);

        /** A tree might get quite big, in which case calling nodes.size() is
         * really not efficient. Since it is trivial to keep the size uptodate,
         * just do it
         */
        int size; 

        /** The set of tree nodes */
        std::list<TreeNode> nodes;

        /** The set of tree nodes */
        std::list<TreeNode> free_nodes;

        /** The final node (0 if none has been found) */
        TreeNode* final_node;
};

/** The basic search algorithm used for planning */
class TreeSearch
{
    public:
        typedef std::vector<double> Angles;
        typedef std::vector< std::pair<double, double> > AngleIntervals;

	TreeSearch();

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
	
	virtual ~TreeSearch();

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
        struct TreeNodePositionAccessor
        {
            typedef double result_type;
            double operator ()(TreeNode const* node, int i) const
            { return node->getPose().position[i]; }
        };
        typedef KDTree::KDTree<3, TreeNode const*, TreeNodePositionAccessor> NNSearch;
        NNSearch kdtree;
};
} // vfh_star namespace

#endif // VFHSTAR_H

