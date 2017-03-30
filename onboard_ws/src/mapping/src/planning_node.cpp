//basic ROS include
#include <ros/ros.h>

//grid map stuff
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

using namespace std;
using namespace grid_map;

class Planner{

public:
  //class methods
  Planner(ros::NodeHandle nh);
  void runtime();
  void map_cb(const grid_map_msgs::GridMap::ConstPtr& map_in);

private:
  //class members
  ros::Subscriber map_sub_;                   //subscribes to local map
  ros::Publisher wp_pub_;                     //publishes waypoints
  GridMap map_;                               //actually stores the map

};

/*
Constructor for Planner. Accepts nodehandle and initializes subscriber, publisher
*/
Planner::Planner(ros::NodeHandle nh){
  map_sub_ = nh.subscribe("/map_node/local_map", 5, &Planner::map_cb, this);
  wp_pub_ = nh.advertise<TODO I DON'T KNOW WHAT>("/waypoints",1);

}

/*
Method that just runs and waits for laser scans
*/
void runtime(){

}

/*
Callback for map topic. Accepts map, checks if the current parth is good, and, if
necessary, runs A*
*/
void map_cb(const grid_map_msgs::GridMap::ConstPtr& map_in){
  //get map
  //make a temporary map
  //inflate obstacles
  //find uhat to next waypoints
  //check if the line is safe
  //find closest node to next waypoints
  //A*
  //simplify path
})

int main(int argc, char** argv){
	//initialize node and publisher
	ros::init(argc, argv, "plan");
	ros::NodeHandle nh("planning_node");

	//create planner object for local path planning
	Planner p(nh);
}
