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
#include "DriveMode.hpp"
#include "Tree.hpp"
#include "NNLookup.hpp"

namespace vfh_star {

class TreeNode;
class NNLookupBox;

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
        std::vector<base::Trajectory> buildTrajectoriesTo(std::vector<const TreeNode *> nodes, const Eigen::Affine3d &world2Trajectory) const;


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
        
        const DebugTree *getDebugTree() const;
        
        void activateDebug(); 
        
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
        void addDriveMode(DriveMode &driveMode);
        
        void clearDriveModes();
    private:
        void addDirections(TreeSearch::Angles& directions, const base::AngleSegment &segement, const double minStep, const double maxStep, const int minNodes) const;
	void updateNodeCosts(TreeNode *node);
	void removeSubtreeFromSearch(TreeNode *node);
        
	Eigen::Affine3d tree2World;
	
	std::multimap<double, TreeNode *> expandCandidates;
        std::vector<DriveMode *> driveModes;
	NNLookup *nnLookup;
};
} // vfh_star namespace

#endif // VFHSTAR_H

