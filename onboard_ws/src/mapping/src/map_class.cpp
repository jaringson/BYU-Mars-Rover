//basic ROS include
#include <ros/ros.h>

//message types
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <rover_msgs/NavState.h>
#include <tf/transform_listener.h>

//grid map stuff
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

//conversion stuff
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>


#include <cmath>

using namespace std;
using namespace grid_map;

class Map_Maker{

public:
	//class methods
	Map_Maker(ros::NodeHandle nh);
	void laser_cb(const sensor_msgs::LaserScan::ConstPtr& scan_in);
	void state_cb(const rover_msgs::NavState::ConstPtr& msg);
	void runtime();

private:
	//class members
	ros::Subscriber laser_sub_;											//subscribes to LaserScan
	ros::Subscriber state_sub_;											//subscribes to Estimate
	ros::Publisher map_pub_;												//publishes our map
	tf::TransformListener listener_;								//does tf stuff
	laser_geometry::LaserProjection projector_;			//transforms the LaserScan to PointCloud2
	GridMap map_;																		//is the map

	//important assumptions:
	//	1. using a NWU coordinate system
	//	2. state is coming in much faster than the scan (which is at about 2 Hz)
	float x_pos_;																		//inertial x-position
	float y_pos_;																		//inertial y-position
};

/*
Constructor for Map_Maker. Accepts nodehandle and initializes subscriber, publisher, and GridMap
*/
Map_Maker::Map_Maker(ros::NodeHandle nh){
	//initialize laser and state subscribers
	laser_sub_ = nh.subscribe("/scan", 20, &Map_Maker::laser_cb, this);
	state_sub_ = nh.subscribe("/estimate", 20, &Map_Maker::state_cb, this);

	//initialize map publisher
	map_pub_ = nh.advertise<grid_map_msgs::GridMap>("/local_map",1);

	//initialize GridMap
	GridMap map_temp({"elevation"});
	map_ = map_temp;
	map_.setFrameId("ins_ground");
	map_.setGeometry(Length(5.0,5.0),0.1);

	//check that it worked
	ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
		map_.getLength().x(), map_.getLength().y(),
		map_.getSize()(0), map_.getSize()(1));

	runtime();

}

/*
Callback for laser scan. Accepts scan and updates map
*/
void Map_Maker::laser_cb(const sensor_msgs::LaserScan::ConstPtr& scan_in){
	//check to make sure that the transform exists (wait up to 1 second for the scan)
	if(!listener_.waitForTransform(
		scan_in->header.frame_id,
		"/ins_ground",
		//timestamp on scan is first point so need to check there is a transform for last point
		scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
		ros::Duration(1.0))){

     return;
	}

	//put the scan into a pointcloud because point clouds are mroe intuitive to
	//add to a map (x,y, and z coordinate system)
	sensor_msgs::PointCloud2 cloud_msg;
	//you can specify the end frame of the transformation here and your cloud will be in that frame
	projector_.transformLaserScanToPointCloud("/ins_ground",*scan_in,cloud_msg,listener_);

	//now do stuff with point cloud
	//make a PCL pointcloud (as opposed to the ROS msg earlier)
	pcl::PointCloud<pcl::PointXYZ>* cloud_xyz = new pcl::PointCloud<pcl::PointXYZ>;
	pcl::fromROSMsg(cloud_msg,*cloud_xyz);

	//loop through cloud and update the map
	for(size_t i = 0; i < cloud_xyz->points.size(); i++){
		//get current [x,y,z]
		float x_temp = cloud_xyz->points[i].x;
		float y_temp = cloud_xyz->points[i].y;
		float z_temp = cloud_xyz->points[i].z;

		if z_temp <= 0
			continue;

		//add the current map position to x and y
		x_temp += map_.getPosition().x();
		y_temp += map_.getPosition().y();

		//make a position (from GridMap namespace)
		Position pos_pt(x_temp,y_temp);

		//TODO rotation to ground plane (if there is time)

		//if the position falls within the map, update its elevation
		if(map_.isInside(pos_pt)){
			map_.atPosition("elevation", pos_pt) = z_temp;
		}
	}

	//move map (avoids asynchronicity) (real talk: makes sure you move the map AFTER adding laser scan)
	Position map_center(x_pos_,y_pos_);
	map_.move(map_center);

}

/*
Method to update position
*/
void Map_Maker::state_cb(const rover_msgs::NavState::ConstPtr& msg){
	x_pos_ = msg->position[0];
	y_pos_ = msg->position[1];
}

/*
Method that just runs and waits for laser scans and estimates
*/
void Map_Maker::runtime(){
	ros::Rate r(10);
	ros::Time time = ros::Time::now();

	while(ros::ok()){
		map_.setTimestamp(time.toNSec());
		grid_map_msgs::GridMap msg;
		GridMapRosConverter::toMessage(map_, msg);
		map_pub_.publish(msg);
		ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", msg.info.header.stamp.toSec());
		// ros::spinOnce();
		r.sleep();
		ros::spin();
	}

}

int main(int argc, char** argv){
	//initialize node and publisher
	ros::init(argc, argv, "map");
	ros::NodeHandle nh("map_node");

	//create map_maker object for updating and publishing map
	Map_Maker mm(nh);
}
