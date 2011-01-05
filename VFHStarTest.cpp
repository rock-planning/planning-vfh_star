#include "VFHStar.h"

using namespace Eigen;

class StarTest: public vfh_star::VFHStar
{
    public:
	StarTest();
	virtual std::vector< std::pair< double, double > > getNextPossibleDirections(const base::Pose& curPose, double obstacleSafetyDist, double robotWidth) const;
	virtual base::Pose getProjectedPose(const base::Pose& curPose, double heading, double distance) const;
};

StarTest::StarTest()
{

}



std::vector< std::pair< double, double > > StarTest::getNextPossibleDirections(const base::Pose& curPose, double obstacleSafetyDist, double robotWidth) const
{
    std::vector< std::pair< double, double > > fakeDirs;
//     fakeDirs.push_back(0);
    fakeDirs.push_back(std::make_pair(-2*M_PI/10.0, -M_PI/10.0) );
    fakeDirs.push_back(std::make_pair(-2*M_PI/4.0, -M_PI/4.0));
    fakeDirs.push_back(std::make_pair(-M_PI/4.0, M_PI/4.0));
    
    return fakeDirs;
}

base::Pose StarTest::getProjectedPose(const base::Pose& curPose, double heading, double distance) const
{
    //super omnidirectional robot
    Vector3d p(0, distance, 0);
    
    base::Pose ret;
    ret.orientation = AngleAxisd(heading, Vector3d::UnitZ());
    ret.position = curPose.position + ret.orientation * p;

    return ret;
}


int main()
{
    StarTest t;
    
    base::Pose start;
    start.orientation = Eigen::Quaterniond::Identity();
    double mainHeading = 0;

    std::vector<base::Waypoint> trajectory = t.getTrajectory(start, mainHeading);
    std::cout << "Starting from " << start.position.transpose() << " with heading " << vfh_star::VFHStar::getHeading(start.orientation) << " in direction of " << mainHeading << std::endl;
    
    std::cout << "Resulting tree is " << t.getTree().getSize() << std::endl;

    std::cout << "Result: " << std::endl;
    for(std::vector<base::Waypoint>::const_iterator it = trajectory.begin(); it != trajectory.end(); it++) 
    {
	std::cout << "  " << it->position.transpose() << std::endl;
    }
    
    return 0;
}
