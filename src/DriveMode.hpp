#ifndef DRIVEMODE_H
#define DRIVEMODE_H

#include <base/Trajectory.hpp>
#include <base/Angle.hpp>
#include <base/Pose.hpp>
#include <stdint.h>
#include <string>

namespace vfh_star
{

class TreeNode;
class DriveMode;

class ProjectedPose 
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        ProjectedPose(): nextPoseExists(true), driveMode(0), driveModeNr(0), angleTurned(0) {};
        base::Pose pose;
        bool nextPoseExists;
        DriveMode const *driveMode;
        uint8_t driveModeNr;
        double angleTurned;
};

class DriveMode
{
private:
    std::string identifier;
public:
    DriveMode(const std::string &identifier);
    
    /**
     * Sets the parameters of this drive mode on the trajectory
     * */
    virtual void setTrajectoryParameters(base::Trajectory &tr) const = 0;

//     /**
//      * Sets the parameters of this drive mode on the trajectory
//      * */
//     virtual void setTrajectoryParameters(base::TrajectoryWithDriveMode &tr) const = 0;
//     
    /**
     * Returns the pose in which the robot will be after traveling from the curNode
     * over a distance of 'distance' towards the moveDirection, using this drive mode.
     * 
     * @param resultInWorldFrame Next projected pose in world frame.
     * @param curNode Current node. Position in here is also in world frame
     * @param moveDirectionInRobotFrame is in Robot Coordinates, meaning an angle of zero is forward.
     * @param distance the distance the robot should travel
     * 
     * returns false if pose could not be projected using this drive mode
     * */
    virtual bool projectPose(ProjectedPose &resultInWorldFrame, const TreeNode& curNode, const base::Angle& moveDirectionInRobotFrame, double distance) const = 0;
    
    /**
     * Returns the cost of driving from the parentNode to the projected position using this drive mode;
     * */
    virtual double getCostForNode(const ProjectedPose& projection,const base::Angle &direction, const TreeNode& parentNode) const = 0;
    
    
    /**
     * Returns a unique identifier for this drive mode
     * */
    const std::string &getIdentifier() const
    {
        return identifier;
    };
};

}
#endif // DRIVEMODE_H
