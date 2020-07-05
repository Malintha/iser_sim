#include "ros/ros.h"
#include <iostream>
#include <ros/console.h>
#include "Trajectory.h"
#include "Visualize.h"


using namespace std;

int main(int argc, char **argv) {
    float frequency = 100;
    ros::init(argc, argv, "prim_vis");
    static ros::NodeHandle n("~");
    string obstacleConfigFileName;
    string trajDir;

    n.getParam("/prim_vis/obstacleFileName", obstacleConfigFileName);
    n.getParam("/prim_vis/traj_dir", trajDir);
    


    ROS_DEBUG_STREAM("obstacleConfigFileName: " << obstacleConfigFileName);
    ROS_DEBUG_STREAM("obstacleConfigFileName: " << obstacleConfigFileName);


    return 0;
}