#include "VFHStar.h"

using namespace Eigen;

class StarTest: public vfh_star::VFHStar
{
    public:
	StarTest();
	virtual std::vector< std::pair< double, double > > getNextPossibleDirections(const vfh_star::TreeNode& curNode, double obstacleSafetyDist, double robotWidth) const;
	virtual std::pair<base::Pose, bool> getProjectedPose(const vfh_star::TreeNode& curNode, double heading, double distance) const;
};

StarTest::StarTest()
{

}



std::vector< std::pair< double, double > > StarTest::getNextPossibleDirections(const vfh_star::TreeNode& curNode, double obstacleSafetyDist, double robotWidth) const
{
    std::vector< std::pair< double, double > > fakeDirs;
//     fakeDirs.push_back(0);
    fakeDirs.push_back(std::make_pair(-2*M_PI/10.0, -M_PI/10.0) );
    fakeDirs.push_back(std::make_pair(-2*M_PI/4.0, -M_PI/4.0));
    fakeDirs.push_back(std::make_pair(M_PI/4.0, 2 * M_PI/4.0));
    
    return fakeDirs;
}

std::pair<base::Pose, bool> StarTest::getProjectedPose(const vfh_star::TreeNode& curNode, double heading, double distance) const
{
    //super omnidirectional robot
    Vector3d p(0, distance, 0);
    
    base::Pose ret;
    ret.orientation = AngleAxisd(heading, Vector3d::UnitZ());
    ret.position = curNode.getPose().position + ret.orientation * p;

    return std::make_pair(ret, true);
}


int main()
{
    StarTest t;
    
    base::Pose start;
    start.orientation = Eigen::Quaterniond::Identity();
    double mainHeading = 0;

    std::vector<base::Waypoint> trajectory = t.getWaypoints(start, mainHeading, 5);
    std::cout << "Starting from " << start.position.transpose() << " with heading " << start.getYaw() << " in direction of " << mainHeading << std::endl;
    
    std::cout << "Resulting tree is " << t.getTree().getSize() << std::endl;

    std::cout << "Result: " << std::endl;
    for(std::vector<base::Waypoint>::const_iterator it = trajectory.begin(); it != trajectory.end(); it++) 
    {
	std::cout << "  " << it->position.transpose() << std::endl;
    }

    for (int i = 0; i < 100; ++i)
    {
        std::cerr << i << std::endl;
        std::vector<base::Waypoint> trajectory = t.getWaypoints(start, mainHeading, 5);
    }
    
    return 0;
}

