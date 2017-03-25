#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
//#include "tf/message_filter.h"
//#include "message_filters/subscriber.h"
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>

using namespace grid_map;

class Map_Maker{

public:
	//class methods
	Map_Maker(ros::NodeHandle nh);
	void laser_cb(const sensor_msgs::LaserScan::ConstPtr& scan);
	// void msg_cb(const boost::shared_ptr<const sensor_msgs::LaserScan>& scan_ptr);
	// void state_cb(const );
	void runtime();

	//class members
	//message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
	//tf::TransformListener tf_;

	ros::Subscriber laser_sub;						//subscribes to LaserScan
	//ros::Subscriber state_sub;						//subscribes to state
	ros::Publisher map_pub;								//publishes our map
	tf::TransformListener listener_;					//does tf stuff
	laser_geometry::LaserProjection projector_;			///transforms the LaserScan to PointCloud2
	GridMap map;
};

/*
Constructor for Map_Maker. Accepts nodehandle and initializes subscriber, publisher, and GridMap
*/
Map_Maker::Map_Maker(ros::NodeHandle nh){
	//initialize subscriber and publisher
	laser_sub = nh.subscribe("laser_scan", 20, boost::bind(&Map_Maker::laser_cb, this, _1));

	//initialize map publisher
	map_pub = nh.advertise<grid_map_msgs::GridMap>("local_map",1);

	//initialize GridMap
	map = new GridMap({"elevation"});
	map.setFrameId("map");
	map.setGeometry(Length(5.0,5.0),0.5);

	//check that it worked
	ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
		map.getLength().x(), map.getLength().y(),
		map.getSize()(0), map.getSize()(1));

	runtime();
}

//  Callback to register with tf::MessageFilter to be called when transforms are available
// void msgCallback(const boost::shared_ptr<const geometry_msgs::PointStamped>& point_ptr){
//
// }

/*
Callback for laser scan. Accepts scan and updates map
*/
void Map_Maker::laser_cb(const sensor_msgs::LaserScan::ConstPtr& scan){

	//check to make sure that the transform exists
	if(!listener_.waitForTransform(
		scan_in->header.frame_id,
		"/base_link",
		//timestamp on scan is first point so need to check there is a transform for last point
		scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
		ros::Duration(1.0))){
     return;
	}

	sensor_msgs::PointCloud2 cloud;
	projector_.transformLaserScantoPointCloud("/base_link",*scan,cloud,listener_);

	//now do stuff with point cloud (probably iterate through points and add to map)
}

/*
Callback for state. Moves the map when the state changes.
*/
// void Map_Maker::state_cb(const ){
//
// }

/*
Method that runs the script at some rate
*/
void runtime(){

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
	ros::NodeHandle nh("~");

	//create map_maker object for updating and publishing map
	mm = Map_Maker(nh);
}
