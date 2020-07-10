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
    std::vector<Grid> schedule;
    for(int i=1; i<=nAgents; i++) {
        stringstream ss;
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
        }
    schedule.push_back(path);
    }
    return schedule;
}

Grid getTopology(std::string dataDir) {
    Grid topology;
    stringstream ss;
    ss << dataDir << "topology.txt";
    std::ifstream infile(ss.str());
    int x, y, z, cls;
    while(infile >> x >> y >> z >> cls) {
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        p.z = z;
        topology.push_back(p);
    }
    return topology;
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
    stringstream ss, ss0, ss1;
    ss0 << dataDir << startsConfigName;
    string startsFilePath = ss0.str();
    ss1 << dataDir << "trajectories/";
    Grid positions = getStartGoalPositions(startsFilePath);
    std::vector<Grid> schedule = getAgentSchedules(nAgents, dataDir);
    ROS_ERROR_STREAM("schedule length: "<<schedule.size());
    
    vector<Trajectory> trajectories = simutils::loadTrajectoriesFromFile(nDrones, n, ss1.str());
    std::cout<<trajectories[0].pos.size()<<std::endl;
    ss << dataDir << obstacleConfigFileName;
    string obstacleConfigPath = ss.str();
    Visualize vis(n, "world", nDrones, obstacleConfigPath);
    vis.addToPaths(trajectories);
    Grid topology = getTopology(dataDir);
    std::cout<<"here"<<std::endl;

    vis.addToTopo(topology);
    vis.addAgentPaths(schedule);

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

