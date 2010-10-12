#ifndef VFHSTAR_H
#define VFHSTAR_H

#include <base/pose.h>
#include <base/waypoint.h>
#include <Eigen/StdVector>
#include <vector>

class TreeNode
{
    friend class Tree;
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	TreeNode();
	TreeNode(const base::Pose &pose, const double &dir);
	
	const base::Pose &getPose() const;
	double &getCost();
	const TreeNode *getParent() const;
	double getDirection() const;
	int getDepth() const;
	double getHeurestic() const;
	double &getHeurestic();
	
    private:
	std::vector<TreeNode *> childs;
	TreeNode *parent;
	
	///pose of the node, note the orientation of the node and the direction may differ, 
	///because of kinematic constrains of the robot
	base::Pose pose;

	///direction, that was choosen, that lead to this node
	double direction;
	double cost;
	double heuristic;
	
	int leafDepth;
};

class Tree
{
    public:
	Tree();
	~Tree();
	void addChild(TreeNode *parent, TreeNode *child);
	void removeChild(TreeNode *parent, TreeNode *child);
	
	const std::vector<TreeNode *> &getChilds(TreeNode *parent);
	TreeNode *getParent(TreeNode *child);
	TreeNode *getRootNode();
	void setRootNode(TreeNode *root);
	
    private:
	TreeNode *root;
};

class VFHStar
{
    public:
	VFHStar();

	std::vector<base::Waypoint> getTrajectory(const base::Pose& start, double heading, double lastDrivenDirection);

	virtual ~VFHStar();
    private:
	double getHeading(const Eigen::Quaterniond &orientation) const;
	std::vector< double > getDirectionsFromIntervals(const std::vector< std::pair<double, double> > &intervals, double heading);
	double getMotionDirection(const Eigen::Vector3d &start, const Eigen::Vector3d &end) const;
	double angleDiff(const double &a1, const double &a2) const;
	double getHeurestic(const TreeNode &node, double headingDirection, double headingWeight, double orientationWeight, double directionWeight) const;
	double getCostForNode(const TreeNode& curNode, double headingDirection, bool primary, double headingWeight, double orientationWeight, double directionWeight) const;

	/**
	* This method returns possible directions where 
	* the robot can drive to, from the given position
	**/
	virtual std::vector< std::pair<double, double> > getNextPossibleDirections(const base::Pose &curPose) const = 0;

	/**
	* This function returns a pose in which the robot would
	* be if he would have driven towards the given direction.
	* This method should take the robot driving constrains
	* into account. 
	*/
	virtual base::Pose getProjectedPose(const base::Pose &curPose, const double &heading,  const double &distance) const = 0;
		
	///step size of forward projection
	double stepDistance;
	
	///maximum depth of search tree
	int maxTreeDepth;
    
};

#endif // VFHSTAR_H
