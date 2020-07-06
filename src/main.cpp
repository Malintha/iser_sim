#include "ros/ros.h"
#include <iostream>
#include <ros/console.h>
#include "swarmsim/Trajectory.h"
#include "swarmsim/Visualize.h"
#include "swarmsim/utils.h"
#include <geometry_msgs/Point.h>
#include <fstream>

using namespace std;

typedef std::vector<geometry_msgs::Point> Grid;
struct visProp {
    double length;
    double width;
    double height;
    double incre;
};

Grid getGridSampling() {
    double len = 40;
    double wid = 40;
    double hei = 5;
    double incre = 5;
    double h_ = 4;

    Grid grid;
    for(double l=0;l<=len;l+=incre) {
        for(double w=0;w<=wid;w+=incre) {
            for(double h=h_;h<=hei;h+=incre) {
                geometry_msgs::Point p;
                p.x = l;
                p.y = w;
                p.z = h;
                grid.emplace_back(p);
            }
        }
    }
    return grid;
}

Grid getStartGoalPositions(std::string startsConfigPath) {
    std::ifstream infile(startsConfigPath);
    int x, y, z;
    Grid positions;
    while(infile >> x >> y >> z) {
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        p.z = z;
        positions.push_back(p);
    }
    ROS_ERROR_STREAM("Start positions: "<<positions.size());
return positions;
}

std::vector<Grid> getAgentSchedules(int nAgents, std::string dataDir) {
    stringstream ss;
    std::vector<Grid> schedule;
    for(int i=1; i<=nAgents; i++) {
        ss << dataDir << "agent_" << std::to_string(i)<<".txt";
        std::ifstream infile(ss.str());
        int x, y, z;
        Grid path;
        while(infile >> x >> y >> z) {
            geometry_msgs::Point p;
            p.x = x;
            p.y = y;
            p.z = z;
            path.push_back(p);
            std::cout<<"point: "<<x<<" "<<y<<" "<<z<<std::endl;
        }
    schedule.push_back(path);
    }
    return schedule;
}

int main(int argc, char **argv) {
    float frequency = 100;
    ros::init(argc, argv, "prm_vis");
    static ros::NodeHandle n("~");
    string obstacleConfigFileName;
    string dataDir;
    int nAgents, nDrones;
    string startsConfigName;
    nAgents = 2;
    n.getParam("/prm_vis/obstacleFileName", obstacleConfigFileName);
    n.getParam("/prm_vis/data_dir", dataDir);
    n.getParam("/prm_vis/nDrones", nDrones);
    n.getParam("/prm_vis/startsConfigName", startsConfigName);

    ROS_DEBUG_STREAM("obstacleConfigFileName: " << obstacleConfigFileName);
    ROS_DEBUG_STREAM("dataDir: " <<dataDir);
    stringstream ss_;
    ss_ << dataDir << startsConfigName;
    string startsFilePath = ss_.str();
    Grid positions = getStartGoalPositions(startsFilePath);
    std::vector<Grid> schedule = getAgentSchedules(nAgents, dataDir);
    ROS_ERROR_STREAM("schedule length: "<<schedule.size());

    vector<Trajectory> trajectories = simutils::loadTrajectoriesFromFile(nDrones, n, dataDir);

    stringstream ss;
    ss << dataDir << obstacleConfigFileName;
    string obstacleConfigPath = ss.str();
    Visualize vis(n, "map", nDrones, obstacleConfigPath);
    vis.addToPaths(trajectories);

    // get grid sampling
    Grid grid = getGridSampling();
    vis.addToGrid(grid);
    vis.addStartGoal(positions);

    while(ros::ok()) {
        vis.draw();
    }

    return 0;
}


Grid getSampling(Grid pathNodes) {

}

