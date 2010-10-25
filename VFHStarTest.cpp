#include "VFHStar.h"

using namespace Eigen;

class StarTest: public VFHStar
{
    public:
	StarTest();
	virtual std::vector< std::pair< double, double > > getNextPossibleDirections(const base::Pose& curPose, const double& obstacleSafetyDist, const double& robotWidth) const;
	virtual base::Pose getProjectedPose(const base::Pose& curPose, const double& heading, const double& distance) const;
};

StarTest::StarTest()
{

}



std::vector< std::pair< double, double > > StarTest::getNextPossibleDirections(const base::Pose& curPose, const double &obstacleSafetyDist, const double &robotWidth) const
{
    std::vector< std::pair< double, double > > fakeDirs;
//     fakeDirs.push_back(0);
    fakeDirs.push_back(std::make_pair(-M_PI/10.0, -M_PI/10.0) );
    fakeDirs.push_back(std::make_pair(-M_PI/4.0, -M_PI/4.0));
    fakeDirs.push_back(std::make_pair(M_PI/4.0, M_PI/4.0));
    
    return fakeDirs;
}

base::Pose StarTest::getProjectedPose(const base::Pose& curPose, const double& heading, const double& distance) const
{
    //super omnidirectional robot
    Vector3d p(0, distance, 0);
    
    base::Pose ret;
    ret.orientation = curPose.orientation * AngleAxisd(heading, Vector3d::UnitZ());
    ret.position = curPose.position + ret.orientation * p;
    
    return ret;
}


int main()
{
    StarTest t;
    
    base::Pose start;
    double heading = 0;
    std::vector<base::Waypoint> trajectory = t.getTrajectory(start, heading, 0);
    std::cout << " Starting from " << start.position.transpose() << " with heading " << heading << std::endl;
    
    for(std::vector<base::Waypoint>::const_iterator it = trajectory.begin(); it != trajectory.end(); it++) 
    {
	std::cout << "Position is " << it->position.transpose() << std::endl;
    }
    
    return 0;
}