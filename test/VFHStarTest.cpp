#include <vfh_star/VFHStar.h>
#include <iostream>
#include <vizkit3d/QtThreadedWidget.hpp>
#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d/VFHTreeVisualization.hpp>
#include <vizkit3d/EnvironmentItemVisualizer.hpp>
#include <vizkit3d/EnvireVisualization.hpp>
#include <vizkit3d/GridVisualization.hpp>

using namespace Eigen;
using namespace vfh_star;

class StarTestDriveMode : public vfh_star::DriveMode
{
public:
    StarTestDriveMode() : DriveMode("TestMode")
    {
        
    }
    virtual double getCostForNode(const ProjectedPose& projection, const base::Angle& direction, const TreeNode& parentNode) const
    {
        return (projection.pose.position - parentNode.getPosition()).norm();
    }
    
    virtual bool projectPose(ProjectedPose &result, const TreeNode& curNode, const base::Angle& moveDirection, double distance) const
    {
        //super omnidirectional robot
        base::Vector3d p(distance, 0, 0);
        base::Angle curHeading = curNode.getYaw();

        // Compute rate of turn for front to wanted heading
        double angleDiffForward = fabs((moveDirection - curHeading).getRad());

        //forward case
        result.pose.orientation = Eigen::AngleAxisd(moveDirection.getRad(), base::Vector3d::UnitZ());
        result.pose.position = curNode.getPose().position + result.pose.orientation * p;
        result.angleTurned = angleDiffForward;
        result.nextPoseExists = true;
        
        return true;
    }
    
    virtual void setTrajectoryParameters(base::Trajectory& tr) const
    {
        tr.speed = 1.0;
    };
    
    // virtual void setTrajectoryParameters(base::TrajectoryWithDriveMode& tr) const
    // {
    //     ;
    // }
};

class StarTest: public vfh_star::VFHStar
{
    public:
        StarTest();
        virtual double getHeuristic(const TreeNode& node) const;
        virtual AngleIntervals getNextPossibleDirections(const TreeNode& curNode) const;
        virtual bool validateNode(const TreeNode& node) const;
        base::Angle minSteerAngle;
        base::Angle maxSteerAngle;
        StarTestDriveMode driveMode;
};

StarTest::StarTest()
{
    addDriveMode(driveMode);
    minSteerAngle = base::Angle::fromDeg(-30);
    maxSteerAngle = base::Angle::fromDeg(30);
}

bool StarTest::validateNode(const TreeNode& node) const
{
    if(node.getCost() == std::numeric_limits<double>::infinity())
        return false;
    
    if(!vfh.validPosition(node.getPose())) {
        return false;
    }
    return true;
}


TreeSearch::AngleIntervals StarTest::getNextPossibleDirections(const TreeNode& curNode) const
{
    return vfh.getNextPossibleDirections(curNode.getPose());
}

double StarTest::getHeuristic(const TreeNode& node) const
{
    return HorizonPlanner::getHeuristic(node);
}



int main()
{
    StarTest t;
     
//     t.setSearchConf(t.getSearchConf());
    const int UNKNOWN = 0;
    const int OBSTACLE = 1;
    const int TRAVERSABLE = 2;
    
    envire::Environment env;

    envire::TraversabilityClass unknown;
    envire::TraversabilityClass drivable(1.0);
    envire::TraversabilityClass obstacle(0.0);

    envire::TraversabilityGrid trGrid(400, 400, 0.05, 0.05, -10.0, -10.0);
    trGrid.setTraversabilityClass(UNKNOWN, unknown);
    trGrid.setTraversabilityClass(OBSTACLE, obstacle);
    trGrid.setTraversabilityClass(TRAVERSABLE, drivable);
    
    envire::FrameNode gridFrame(Eigen::Affine3d::Identity());
    env.attachItem(&trGrid);
    env.setFrameNode(&trGrid, &gridFrame);
    env.getRootNode()->addChild(&gridFrame);
    
    envire::TraversabilityGrid::ArrayType &trData(trGrid.getGridData(envire::TraversabilityGrid::TRAVERSABILITY));
    std::fill(trData.data(), trData.data() + trData.num_elements(), TRAVERSABLE);
    
    Vector3d obstaclePos(1.5,0,0);
    double obstWidth = 1.0;
    double obstHeight = 0.2;
    size_t obstX = 0, obstY = 0;
    assert(trGrid.toGrid(obstaclePos, obstX, obstY));
    
    std::cout << "Scale " << trGrid.getScaleY() << " result " << -(obstHeight / trGrid.getScaleY() / 2.0) << std::endl;
    
    
    //add obstacle in front of robot
    for(int y = -(obstWidth / trGrid.getScaleY() / 2.0) ; y < (obstWidth / trGrid.getScaleY() / 2.0); y++)
    {
        for(int x = -(obstHeight / trGrid.getScaleX() / 2.0); x < (obstHeight / trGrid.getScaleX() / 2.0); x++)
        {
            assert((obstX + x > 0) && (obstX + x < trGrid.getCellSizeX()));
            assert((obstY + y > 0) && (obstY + y < trGrid.getCellSizeY()));
            trData[obstY + y][obstX + x] = OBSTACLE;
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
    conf.maxTreeSize = 100000;
    conf.stepDistance = 0.1;
    
    AngleSampleConf global;
    //5° min steps
    global.angularSamplingMin = 5 * M_PI / 360.0;
    //10° max between steps
    global.angularSamplingMax = 10 * M_PI / 180.0;
    global.angularSamplingNominalCount = 5;
    global.intervalStart = 0;
    global.intervalWidth = 2* M_PI;
    
    conf.sampleAreas.clear();
    conf.sampleAreas.push_back(global);
    
    conf.identityPositionThreshold = 0.06;
    conf.identityYawThreshold = 3*M_PI/180.0;
    t.setSearchConf(conf);
    
    
    vfh_star::VFHStarConf starConf = t.getCostConf();
    starConf.vfhConf.obstacleSafetyDistance = 0.1;
    starConf.vfhConf.robotWidth = 0.5;
    starConf.vfhConf.obstacleSenseRadius = 1.0;
    starConf.vfhConf.histogramSize = 180;
    starConf.vfhConf.lowThreshold = 2000.0;
    
    starConf.mainHeadingWeight = 0;
    starConf.turningWeight = 2;
    
    t.setCostConf(starConf);
    t.activateDebug();
    
    base::Pose start;
    start.orientation = Eigen::Quaterniond::Identity();
    base::Angle mainHeading = base::Angle::fromDeg(0.0);
    
    base::Time startTime = base::Time::now();
    
    std::vector<base::Trajectory> trajectory = t.getTrajectories(start, mainHeading, 5.0, Eigen::Affine3d::Identity());
    
    base::Time endTime = base::Time::now();

    std::cout << "Starting from " << start.position.transpose() << " with heading " << start.getYaw() << " in direction of " << mainHeading << std::endl;    
    
    std::cout << "Resulting tree is " << t.getTree().getSize() << " took " << endTime-startTime << std::endl;

    std::cout << "Result: " << std::endl;
//     for(std::vector<base::Waypoint>::const_iterator it = trajectory.begin(); it != trajectory.end(); it++) 
//     {
// 	std::cout << "  " << it->position.transpose() << std::endl;
//     }

//     for (int i = 0; i < 2; ++i)
//     {
//         std::cerr << i << std::endl;
//         std::vector<base::Waypoint> trajectory = t.getWaypoints(start, mainHeading, 5);
//     }
    
    QtThreadedWidget<vizkit3d::Vizkit3DWidget> app;
    
    vizkit3d::GridVisualization gridViz;
    vizkit3d::VFHTreeVisualization treeViz;
    envire::EnvireVisualization envViz;

    app.start();
    app.getWidget()->addPlugin(&treeViz);
    app.getWidget()->addPlugin(&envViz);
    app.getWidget()->addPlugin(&gridViz);
    
    if(t.getDebugTree())
    {
        treeViz.updateData(*(t.getDebugTree()));
    }
    treeViz.removeLeaves(false);
    
    envViz.updateData(&env);
    
    std::cout << "Tree has " << t.getTree().getSize() << " Nodes " << std::endl;
    
    while(app.isRunning())
        usleep(10000);
    
    
    return 0;
}

