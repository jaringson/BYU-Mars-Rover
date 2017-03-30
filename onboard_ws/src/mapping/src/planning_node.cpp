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
  const float THRESHOLD_ = 0.15;              //threshold for what the rover can't drive over
  const float ROVER_WIDTH_ = 1.25;            //width of rover in meters (measured)

};

/*
Constructor for Planner. Accepts nodehandle and initializes subscriber, publisher
*/
Planner::Planner(ros::NodeHandle nh){
  map_sub_ = nh.subscribe("/map_node/local_map", 5, &Planner::map_cb, this);
  wp_pub_ = nh.advertise<grid_map_msgs::GridMap>("/waypoints",1);     //THIS IS WRONG BUT I'M NOT SURE WHAT IT WILL BE

}

/*
Method that just runs and waits for laser scans
*/
void Planner::runtime(){

}

/*
Callback for map topic. Accepts map, checks if the current parth is good, and, if
necessary, runs A*
*/
void Planner::map_cb(const grid_map_msgs::GridMap::ConstPtr& map_in){
  //take map from message to actual map
  GridMapRosConverter::fromMessage(*map_in, map_);

  //make a temporary map and set its elevation layer to be equal to the elevation layer of the actual map
  GridMap temp_map({"elevation"});
  temp_map.get({"elevation"}) = map_.get({"elevation"});

  //------------------Obstacle Inflation----------------

  //inflate obstacles
  for (GridMapIterator it(temp_map); !it.isPastEnd(); ++it){
    //for use in inflating area around impassable spots
    Position center;
    double radius = ROVER_WIDTH_/2.0;

    //if the elevation somewhere is a real value and above the threshold
    if(!isnan(temp_map.at("elevation", *it)) && temp_map.at("elevation", *it) >= THRESHOLD_){
      //get the position (will serve as the center of the circle)
      temp_map.getPosition(*it, center);

      for (grid_map::CircleIterator iterator(map_, center, radius); !iterator.isPastEnd(); ++iterator){
        //set each spot in the circle to a non-clearable height
        temp_map.at("elevator", *iterator) = 2*THRESHOLD_;
      }
    }
  }

  //For testing, could publish right here and display in rviz

  /*
  at this point, temp_map should have a number that could be 0 of (possibly intersecting)
  circles of impassable points. all others should be nan or values below THRESHOLD_.
  Now, figure out the line between our current position and next waypoint. If it's clear,
  just head in that direction. If not, then we need to replan our local path.
  */

  //-------------------Check the path ahead---------------

  //find unit vector to next waypoint


  //check if the line is safe
  //find closest node to next waypoint
  //A*
  //simplify path
}

int main(int argc, char** argv){
	//initialize node and publisher
	ros::init(argc, argv, "plan");
	ros::NodeHandle nh("planning_node");

	//create planner object for local path planning
	Planner p(nh);
}
