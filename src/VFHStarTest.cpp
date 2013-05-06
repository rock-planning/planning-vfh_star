#include "VFHStar.h"
#include <iostream>
#include <vizkit/QtThreadedWidget.hpp>
#include <vizkit/Vizkit3DWidget.hpp>
#include <vizkit/VFHTreeVisualization.hpp>
#include <vizkit/EnvironmentItemVisualizer.hpp>
#include <vizkit/EnvireVisualization.hpp>

using namespace Eigen;

class StarTest: public vfh_star::VFHStar
{
    public:
	StarTest();
        virtual AngleIntervals getNextPossibleDirections(const vfh_star::TreeNode& curNode) const;
        virtual std::pair< base::Pose, bool > getProjectedPose(const vfh_star::TreeNode& curNode, const base::Angle& heading, double distance) const;
        
        base::Angle minSteerAngle;
        base::Angle maxSteerAngle;
};

StarTest::StarTest()
{
    minSteerAngle = base::Angle::fromDeg(-30);
    maxSteerAngle = base::Angle::fromDeg(30);
}

vfh_star::TreeSearch::AngleIntervals StarTest::getNextPossibleDirections(const vfh_star::TreeNode& curNode) const
{
    vfh_star::TreeSearch::AngleIntervals ret = vfh_star::VFHStar::getNextPossibleDirections(curNode);
    
//     std::cout << "VFHStar returned these directions :" << std::endl;
//     
//     for(vfh_star::TreeSearch::AngleIntervals::iterator it = ret.begin(); it != ret.end(); it++)
//     {
//         std::cout << "From " << it->first << " to " << it->second << std::endl;
//     }
    
    return ret;
}


// vfh_star::VFHStar::AngleIntervals StarTest::getNextPossibleDirections(const vfh_star::TreeNode& curNode) const
// {
//     vfh_star::VFHStar::AngleIntervals fakeDirs;
// //     fakeDirs.push_back(0);
//     fakeDirs.push_back(std::make_pair( minSteerAngle, maxSteerAngle ));
//     fakeDirs.push_back(std::make_pair( minSteerAngle.inverse(), maxSteerAngle.inverse() ));
//     
//     return fakeDirs;
// }

std::pair<base::Pose, bool> StarTest::getProjectedPose(const vfh_star::TreeNode& curNode, const base::Angle& direction, double distance) const
{
    //super omnidirectional robot
    Vector3d p(distance, 0, 0);
    
    base::Pose ret;

    base::Angle headingChange = direction;
    
    //compute inveser direction if driving backwards
    if(!direction.isInRange(minSteerAngle, maxSteerAngle))
    {
        headingChange.invert();
    }
    
    ret.orientation = curNode.getPose().orientation * AngleAxisd(direction.getRad(), base::Vector3d::UnitZ());
    ret.position = curNode.getPose().position + ret.orientation * p;

    return std::make_pair(ret, true);
}


int main()
{
    StarTest t;
     
//     t.setSearchConf(t.getSearchConf());
    
    envire::Environment env;
    envire::TraversabilityGrid trGrid(400, 400, 0.05, 0.05, -10.0, -10.0);
    envire::FrameNode gridFrame();
    env.attachItem(&trGrid);

    envire::TraversabilityGrid::ArrayType &trData(trGrid.getGridData(envire::TraversabilityGrid::TRAVERSABILITY));
    
    for(size_t y = 0; y < trGrid.getCellSizeY(); y++)
    {
        for(size_t x = 0; x < trGrid.getCellSizeX(); x++)
        {
            trData[y][x] = vfh_star::TRAVERSABLE;
        }
    }
    
    
    Vector3d obstaclePos(1.5,0,0);
    double obstWidth = 1.0;
    double obstHeight = 0.2;
    size_t obstX, obstY;
    assert(trGrid.toGrid(obstaclePos, obstX, obstY));
    
    std::cout << "Scale " << trGrid.getScaleY() << " result " << -(obstHeight / trGrid.getScaleY() / 2.0) << std::endl;
    
    //add obstacle in front of robot
    for(int y = -(obstWidth / trGrid.getScaleY() / 2.0) ; y < (obstWidth / trGrid.getScaleY() / 2.0); y++)
    {
        for(int x = -(obstHeight / trGrid.getScaleX() / 2.0); x < (obstHeight / trGrid.getScaleX() / 2.0); x++)
        {
            assert((obstX + x > 0) && (obstX + x < trGrid.getCellSizeX()));
            assert((obstY + y > 0) && (obstY + y < trGrid.getCellSizeY()));
            trData[obstY + y][obstX + x] = vfh_star::OBSTACLE;
        }
    }
    
//     exit(0);
/*    
    for(size_t y = 0; y < trGrid.getCellSizeY(); y++)
    {
        for(size_t x = 0; x < trGrid.getCellSizeX(); x++)
        {
            int foo = trData[y][x];
            std::cout << foo << " ";
        }
        std::cout << std::endl;
    }
    */
    t.setNewTraversabilityGrid(&trGrid);
    
    vfh_star::TreeSearchConf conf = t.getSearchConf();
    conf.maxTreeSize = 1000000;
    conf.stepDistance = 0.1;
    //0.5° min steps
    conf.directionSampleConf.angularSamplingMin = 5 * M_PI / 360.0;
    //1° max between steps
    conf.directionSampleConf.angularSamplingMax = 10 * M_PI / 180.0;
    conf.directionSampleConf.angularSamplingNominalCount = 5;
    conf.identityPositionThreshold = 0.06;
    conf.identityYawThreshold = 3*M_PI/180.0;
    t.setSearchConf(conf);
    
    
    vfh_star::VFHStarConf starConf = t.getCostConf();
    starConf.vfhConf.obstacleSafetyDistance = 0.1;
    starConf.vfhConf.robotWidth = 0.5;
    starConf.vfhConf.obstacleSenseRadius = 1.0;
    
    starConf.mainHeadingWeight = 0;
    starConf.turningWeight = 2;
    
    t.setCostConf(starConf);
    
    
    base::Pose start;
    start.orientation = Eigen::Quaterniond::Identity();
    base::Angle mainHeading;
    
    base::Time startTime = base::Time::now();
    
    std::vector<base::Waypoint> trajectory = t.getWaypoints(start, mainHeading, 5);
    
    base::Time endTime = base::Time::now();

    std::cout << "Starting from " << start.position.transpose() << " with heading " << start.getYaw() << " in direction of " << mainHeading << std::endl;    
    
    std::cout << "Resulting tree is " << t.getTree().getSize() << " took " << endTime-startTime << std::endl;

    std::cout << "Result: " << std::endl;
    for(std::vector<base::Waypoint>::const_iterator it = trajectory.begin(); it != trajectory.end(); it++) 
    {
	std::cout << "  " << it->position.transpose() << std::endl;
    }

//     for (int i = 0; i < 2; ++i)
//     {
//         std::cerr << i << std::endl;
//         std::vector<base::Waypoint> trajectory = t.getWaypoints(start, mainHeading, 5);
//     }
    
    QtThreadedWidget<vizkit::Vizkit3DWidget> app;
    vizkit::VFHTreeVisualization treeViz;
    envire::EnvireVisualization envViz;

    app.start();
    app.getWidget()->addDataHandler(&treeViz);
    app.getWidget()->addDataHandler(&envViz);
    
    treeViz.updateData(t.getTree());
    treeViz.removeLeaves(false);
    
    envViz.updateData(&env);
    
    std::cout << "Tree has " << t.getTree().getSize() << " Nodes " << std::endl;
    
    while(app.isRunning())
        usleep(10000);
    
    
    return 0;
}

