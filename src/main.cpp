#include "ros/ros.h"
#include <iostream>
#include <ros/console.h>
#include "swarmsim/Trajectory.h"
#include "swarmsim/Visualize.h"
#include "swarmsim/utils.h"
#include <geometry_msgs/Point.h>

using namespace std;

typedef std::vector<geometry_msgs::Point> Grid;

int main(int argc, char **argv) {
    float frequency = 100;
    ros::init(argc, argv, "prm_vis");
    static ros::NodeHandle n("~");
    string obstacleConfigFileName;
    string dataDir;
    int nDrones;

    n.getParam("/prm_vis/obstacleFileName", obstacleConfigFileName);
    n.getParam("/prm_vis/data_dir", dataDir);
    n.getParam("/prm_vis/nDrones", nDrones);

    ROS_DEBUG_STREAM("obstacleConfigFileName: " << obstacleConfigFileName);
    ROS_DEBUG_STREAM("dataDir: " <<dataDir);
    vector<Trajectory> trajectories = simutils::loadTrajectoriesFromFile(nDrones, n, dataDir);

    stringstream ss;
    ss << dataDir << obstacleConfigFileName;
    string obstacleConfigPath = ss.str();
    Visualize vis(n, "map", nDrones, obstacleConfigPath);
    vis.addToPaths(trajectories);


    while(ros::ok()) {
        vis.draw();
    }

    return 0;
}

Grid getGridSampling() {}

Grid getSampling(Grid pathNodes) {}

