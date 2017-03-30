//basic ROS include
#include <ros/ros.h>

//message types
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
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
	void laser_cb(const sensor_msgs::LaserScan::ConstPtr& scan);
	void runtime();

	ros::Subscriber laser_sub_;											//subscribes to LaserScan
	ros::Publisher map_pub_;												//publishes our map
	tf::TransformListener listener_;								//does tf stuff
	laser_geometry::LaserProjection projector_;			//transforms the LaserScan to PointCloud2
	GridMap map_;																		//is the map
};

/*
Constructor for Map_Maker. Accepts nodehandle and initializes subscriber, publisher, and GridMap
*/
Map_Maker::Map_Maker(ros::NodeHandle nh){
	//initialize subscriber and publisher
	laser_sub_ = nh.subscribe("laser_scan", 20, &Map_Maker::laser_cb, this);

	//initialize map publisher
	map_pub_ = nh.advertise<grid_map_msgs::GridMap>("local_map",1);

	//initialize GridMap
	GridMap map_temp({"elevation"});
	map_ = map_temp;
	map_.setFrameId("map");
	map_.setGeometry(Length(5.0,5.0),0.5);

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

	//check to make sure that the transform exists
	if(!listener_.waitForTransform(
		scan_in->header.frame_id,
		"/base_link",
		//timestamp on scan is first point so need to check there is a transform for last point
		scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
		ros::Duration(1.0))){
     return;
	}

	sensor_msgs::PointCloud2 cloud_msg;
	projector_.transformLaserScanToPointCloud("/base_link",*scan_in,cloud_msg,listener_);

	//now do stuff with point cloud (probably iterate through points and add to map)
	//make a PCL pointcloud
	pcl::PointCloud<pcl::PointXYZ>* cloud_xyz = new pcl::PointCloud<pcl::PointXYZ>;
	pcl::fromROSMsg(cloud_msg,*cloud_xyz);

	//loop through cloud and update the map
	for(size_t i = 0; i < cloud_xyz->points.size(); i++){
		//get current [x,y,z]
		float x_temp = cloud_xyz->points[i].x;
		float y_temp = cloud_xyz->points[i].y;
		float z_temp = cloud_xyz->points[i].z;

		//make a position (from GridMap namespace)
		Position pos_pt(x_temp,y_temp);

		//if the position falls within the map, update its elevation
		if(map_.isInside(pos_pt)){
			map_.atPosition("elevation", pos_pt) = z_temp;
		}
	}

	//move map (avoids asynchronicity) (real talk: makes sure you move the map after adding laser scan)
	
}

/*
Method that runs the script at some rate
*/
void Map_Maker::runtime(){

	//set a rate to update the map at (20 Hz)
	ros::Rate rate(20);

	while(ros::ok()){

		//get tf right now

		//move map

		ros::spinOnce();
		rate.sleep();

	}

}

int main(int argc, char** argv){
	//initialize node and publisher
	ros::init(argc, argv, "map");
	ros::NodeHandle nh("map_node");

	//create map_maker object for updating and publishing map
	Map_Maker mm(nh);
}
