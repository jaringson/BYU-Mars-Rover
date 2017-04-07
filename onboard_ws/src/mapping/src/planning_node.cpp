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

using namespace std;
using namespace grid_map;

class Planner{

public:
  //class methods
  Planner(ros::NodeHandle nh);
  virtual ~Planner();
  void runtime();
  void map_cb(const grid_map_msgs::GridMap::ConstPtr& map_in);
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
  const float THRESHOLD_ = 0.075;//0.15              //threshold for what the rover can't drive over
  const float ROVER_WIDTH_ = 1.25;            //width of rover in meters (measured as 1.07, safety at 1.25)

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

  // initizilize waypoints to a dummy 0 matrix
  Eigen::MatrixXf another_one = Eigen::MatrixXf::Zero(8,3);
  waypoints_ = another_one;

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
Method that just runs and waits for laser scans
*/
void Planner::runtime(){
  ros::spin();
}

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

  //inflate obstacles
  for (GridMapIterator it(map_); !it.isPastEnd(); ++it){
    //for use in inflating area around impassable spots
    Position center;
    double radius = ROVER_WIDTH_/2.0;

    //if the elevation somewhere is a real value and above the threshold
    if(!isnan(map_.at("elevation", *it)) && map_.at("elevation", *it) >= THRESHOLD_){
      //get the position (will serve as the center of the circle)
      map_.getPosition(*it, center);

      for (grid_map::CircleIterator iterator(map_, center, radius); !iterator.isPastEnd(); ++iterator){
        //set each spot in the circle to a non-clearable height
        ROS_INFO("temp_map spot was to: %f",temp_map.at("elevation", *iterator));
        temp_map.at("elevation", *iterator) = 2*THRESHOLD_;
        ROS_INFO("temp_map spot set to: %f",temp_map.at("elevation", *iterator));
      }
    }
  }

  //For testing, could publish right here and display in rviz
  ros::Time time = ros::Time::now();
  temp_map.setTimestamp(time.toNSec());
  grid_map_msgs::GridMap message;
  GridMapRosConverter::toMessage(temp_map, message);
  test_pub_.publish(message);
  ROS_INFO("Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

  /*
  at this point, temp_map should have a number that could be 0 of (possibly intersecting)
  circles of impassable points. all others should be nan or values below THRESHOLD_.
  Now, figure out the line between our current position and next waypoint. If it's clear,
  just head in that direction. If not, then we need to replan our local path.
  */

  //-------------------Check the path ahead---------------

  //TODO add a transform listener that gets the current position in inertial frame
  //and sets cuurent_pos_ to it

  //find unit vector to next waypoint
  Eigen::Vector2d uhat(2);
  uhat << waypoints_(current_wp_,0) - current_pos_[0],
          waypoints_(current_wp_,1) - current_pos_[1];
  uhat = uhat / uhat.squaredNorm();

  //------------------WP Pub test, this works. Move it wherever-----------------
  //test the waypoint publisher
  // tf::matrixEigenToMsg(waypoints_,wp_msg_);
  // wp_pub_.publish(wp_msg_);
  //----------------------------------------------------------------------------

  // uhat = w_next-w_current

  //check if the line is safe
  //find closest node to next waypoint
  //A*
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
