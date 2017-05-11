//basic ROS include
#include <ros/ros.h>

//custom message type for a bunch of floats
#include <mapping/FloatList.h>
#include <std_msgs/Float64MultiArray.h>

//grid map stuff
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <eigen_conversions/eigen_msg.h>

#include <Eigen/Core>
#include <math.h>
#include "AstarPlanner.h"
#include <typeinfo>

using namespace std;
using namespace grid_map;

class Planner{

public:
  //class methods
  Planner(ros::NodeHandle nh);
  virtual ~Planner();
  void runtime();
  void map_cb(const grid_map_msgs::GridMap::ConstPtr& map_in);
  MatrixXf smooth_path(const Eigen::MatrixXf& path);
  // void wp_cb(Eigen::MatrixXf path_in);

private:
  //class members
  ros::Subscriber map_sub_;                   //subscribes to local map
  ros::Subscriber global_wp_sub_;             //subscribes to global waypoint path
  ros::Publisher wp_pub_;                     //publishes waypoints
  ros::Publisher test_pub_;                   //used for testing generated maps
  GridMap map_;                               //actually stores the map
  Eigen::MatrixXf waypoints_;                 //nx3 matrix where n is number of waypoints
  int current_wp_;                            //current waypoint
  Eigen::Vector2f current_pos_;               //current position of rover, update frequently
  std_msgs::Float64MultiArray wp_msg_;        //message for waypoints
  bool received_path_;                        //if true, path is already received, do not let incoming topics update it
  bool clear_path_;                           //if true, no obstacles between current position and next waypoint
  const float THRESHOLD_ = 0.075;//0.15              //threshold for what the rover can't drive over
  const float ROVER_WIDTH_ = 0.5;//1.25;            //width of rover in meters (measured as 1.07, safety at 1.25)

};

/*
Constructor for Planner. Accepts nodehandle and initializes subscriber, publisher
*/
Planner::Planner(ros::NodeHandle nh){
  map_sub_ = nh.subscribe("/grid_map_tutorial_demo/grid_map", 5, &Planner::map_cb, this);
  // global_wp_sub_ = nh.subscribe("/global_path", 1, &Planner::wp_cb, this);
  wp_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/waypoint_path",1);
  test_pub_ = nh.advertise<grid_map_msgs::GridMap>("/test_map",1);     //THIS IS WRONG BUT I'M NOT SURE WHAT IT WILL BE

  //set this as false because we haven't got it yet
  received_path_ = false;

  //set this as true because we don't know
  clear_path_ = true;

  // initizilize waypoints to a dummy 0 matrix
  Eigen::MatrixXf another_one = Eigen::MatrixXf::Zero(8,3);
  waypoints_ = another_one;

  //temporary (for testing checking along line)
  waypoints_(1,0) = -2.5;
  waypoints_(1,1) = 2;

  //initialize current position to (0,0)
  current_pos_ << 0,
                  0;

  //initialize current waypoint to 1 (first waypoint should be (0,0))
  current_wp_ = 1;

  runtime();
}

/*
Destructor for Planner
*/
Planner::~Planner() {}

/*
Method that just runs and waits for maps
*/
void Planner::runtime(){
  ros::spin();
}

// Eigen::MatrixXf smooth_path(const Eigen::MatrixXf& path){
//   int length = path.rows();
//   int current_point = 1;
//
//   Eigen::MatrixXf dummy = path;
//   return dummy;
// }

/*
Callback for map topic. Accepts map, checks if the current parth is good, and, if
necessary, runs A*
*/
void Planner::map_cb(const grid_map_msgs::GridMap::ConstPtr& map_in){
  //take map from message to actual map
  GridMapRosConverter::fromMessage(*map_in, map_);

  //make a temporary map and set its elevation layer to be equal to the elevation layer of the actual map
  GridMap temp_map;
  temp_map.setFrameId("map");
  temp_map.setGeometry(Length(map_.getLength().x(),map_.getLength().y()),map_.getResolution(),Position(map_.getPosition()));
  temp_map.add("elevation", map_.get("elevation"));

  //------------------Obstacle Inflation----------------

  Position center;

  //inflate obstacles
  for (GridMapIterator it(map_); !it.isPastEnd(); ++it){
    //for use in inflating area around impassable spots
    double radius = ROVER_WIDTH_/2.0;

    //if the elevation somewhere is a real value and above the threshold
    if(!isnan(map_.at("elevation", *it)) && map_.at("elevation", *it) >= THRESHOLD_){
      //get the position (will serve as the center of the circle)
      map_.getPosition(*it, center);

      for (grid_map::CircleIterator iterator(map_, center, radius); !iterator.isPastEnd(); ++iterator){
        //set each spot in the circle to a non-clearable height
        // ROS_INFO("temp_map spot was to: %f",temp_map.at("elevation", *iterator));
        temp_map.at("elevation", *iterator) = 2*THRESHOLD_;
        // ROS_INFO("temp_map spot set to: %f",temp_map.at("elevation", *iterator));
      }
    }
  }

  /*
  at this point, temp_map should have a number that could be 0 of (possibly intersecting)
  groups of impassable points. all others should be nan or values below THRESHOLD_.
  Now, figure out the line between our current position and next waypoint. If it's clear,
  just head in that direction. If not, then we need to replan our local path.
  */

  //-------------------Check the path ahead---------------

  //TODO add a transform listener that gets the current position in inertial frame
  //and sets curent_pos_ to it

  //vector for going as far along direction to next waypoint from current position
  //if next waypoint is outside map bounds, will be an edge
  //otherwise set to be next waypoint
  Eigen::Vector2f edge_pt(2);

  if(temp_map.isInside(Position(waypoints_(current_wp_,0),waypoints_(current_wp_,1)))){
    edge_pt << waypoints_(current_wp_,0),
               waypoints_(current_wp_,1);
  } else {
    //find unit vector to next waypoint
    Eigen::Vector2f uhat(2);
    uhat << waypoints_(current_wp_,0) - current_pos_[0],
            waypoints_(current_wp_,1) - current_pos_[1];

    uhat = uhat / uhat.norm();

    float PI = 3.14159265359;
    float h_over_2 = map_.getLength().x()/2.0-map_.getResolution();
    float w_over_2 = map_.getLength().y()/2.0-map_.getResolution();

    float theta = atan2(waypoints_(current_wp_,1) - current_pos_[1],waypoints_(current_wp_,0) - current_pos_[0]);
    theta = fmod(theta + 2.0*PI, 2.0*PI);
    ROS_INFO("theta: %f", theta);

    if(theta <= PI/4.0)
        edge_pt << current_pos_[0] + h_over_2,
                   current_pos_[1] + h_over_2*tan(theta);
    else if(theta <= 3.0*PI/4.0)
        edge_pt << current_pos_[0] - w_over_2*tan(theta-PI/2.0),
                   current_pos_[1] + w_over_2;
    else if(theta <= 5.0*PI/4.0)
        edge_pt << current_pos_[0] - h_over_2,
                   current_pos_[1] - h_over_2*tan(theta-PI);
    else if(theta <= 7.0*PI/4.0)
        edge_pt << current_pos_[0] + w_over_2*tan(theta-3.0*PI/2.0),
                   current_pos_[1] - w_over_2;
    else
        edge_pt << current_pos_[0] + h_over_2,
                   current_pos_[1] + h_over_2*tan(theta);
  }

  ROS_INFO("edge0: %f",edge_pt[0]);
  ROS_INFO("edge1: %f",edge_pt[1]);

  Position startPos(current_pos_[0],current_pos_[1]);
  Position endPos(edge_pt[0],edge_pt[1]);

  for (grid_map::LineIterator iterator(temp_map, startPos, endPos);
      !iterator.isPastEnd(); ++iterator) {
    if(temp_map.at("elevation",*iterator) >= THRESHOLD_){
      clear_path_ = false;
      ROS_INFO("elevation: %f",temp_map.at("elevation",*iterator));
      break;
    }

    //only executes if every spot along the line is below the threshold
    clear_path_ = true;
  }

  if(clear_path_)
    ROS_INFO("clear");
  else
    ROS_INFO("not clear");

  Eigen::MatrixXf *inflated_elevation = new Eigen::MatrixXf;
  *inflated_elevation = temp_map.get("elevation");
  // AstarPlanner astar(inflated_elevation);

  MatrixXf mapinit(10,10);
  mapinit << 1,1,1,1,1,1,1,1,1,1,
         1,1,1,1,1,1,1,1,1,1,
         1,9,1,1,1,1,1,1,1,1,
         9,9,9,9,9,9,1,1,1,1,
         1,1,1,1,1,1,1,1,1,1,
         1,1,1,1,1,1,1,1,1,1,
         1,1,1,1,1,1,1,1,1,1,
         1,1,1,1,1,1,1,1,1,1,
         1,1,1,1,1,1,1,1,1,1,
         1,1,1,1,1,1,1,1,1,1;


  Eigen::MatrixXf* map = new Eigen::MatrixXf(10,10);
  *map = mapinit;

  AstarPlanner astar(map);
  astar.SetGoal(4,0);
  Eigen::MatrixXf path = astar.GetPath();
  astar.PrintMap();

  GridMap newOne;
  newOne.setFrameId("whatever");
  newOne.setGeometry(Length(10,10),1,Position(0,0));
  newOne.add("elevation",mapinit);
  // cout << newOne.get("elevation");
  Position whatever;
  if(astar.pathfound){
    cout << "smoothing path" << endl;
    Eigen::MatrixXf smoothed_path(1,2);
    int base_index = 0;
    smoothed_path.row(0) = path.row(0);
    for(int i=1; i < path.rows(); i++){
      Position base_pos(path(base_index,0),path(base_index,1));
      Position check_pos(path(i,0),path(i,1));
      for (grid_map::LineIterator iterator(newOne, base_pos, check_pos);
          !iterator.isPastEnd(); ++iterator) {
        if(newOne.at("elevation",*iterator) >= 2){//THRESHOLD_){
          newOne.getPosition(*iterator, whatever);
          cout << "Offending position: " << whatever << endl;
          cout << "Height:" << newOne.at("elevation",*iterator) << endl;
          cout << "There is something too tall between " << base_index << " and " << i << endl;
          smoothed_path.conservativeResize(smoothed_path.rows()+1,NoChange);
          smoothed_path.row(smoothed_path.rows()-1) = path.row(i-1);
          base_index = i;
          cout << "I'm increasing the index to : " << base_index << endl;
          break;
        }
      }
    }
    smoothed_path.conservativeResize(smoothed_path.rows()+1,NoChange);
    smoothed_path.row(smoothed_path.rows()-1) = path.row(path.rows()-1);
    cout << "The smoothed path is: " << smoothed_path << endl;
  }else
    cout << "no path exists" << endl;

  //For testing, could publish right here and display in rviz
  ros::Time time = ros::Time::now();
  temp_map.setTimestamp(time.toNSec());
  grid_map_msgs::GridMap message;
  GridMapRosConverter::toMessage(temp_map, message);
  test_pub_.publish(message);
  ROS_INFO("Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

  //A* but if there is no safe path to where you're trying to go, need to
  //come up with something
  //simplify path
}

/*
Callback for global waypoint path (can we just have this publish continuously always?)
*/
// void Planner::wp_cb(Eigen::MatrixXf path_in){
//   if(!received_path_){
//     waypoints_ = path_in;
//     received_path_ = true;
//   }
// }

int main(int argc, char** argv){
	//initialize node and publisher
	ros::init(argc, argv, "plan");
	ros::NodeHandle nh("planning_node");

	//create planner object for local path planning
	Planner p(nh);
}

//------------------WP Pub test, this works. Move it wherever-----------------
//test the waypoint publisher
// tf::matrixEigenToMsg(waypoints_,wp_msg_);
// wp_pub_.publish(wp_msg_);
//----------------------------------------------------------------------------
