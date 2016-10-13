#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <sstream>

int[5] cam_array = [0,0,0,0,0];

/*
callback to set values of cam_array. 1 indicates that camera (n) is being sampled by rover camera
controller and its images are being published as compressed images over the rocket
*/
void cmd_callback(const std::String::ConstPtr& msg){
	cam_array = msg;		//must change b/c rover_command will have lots of fields
}

void image_callback(const sensor_msgs::ImageConstPtr& msg){
	
}

int main(int argc, char **argv){
	//initialize node
	ros::init(argc, argv, "camera_base");

	//node handle
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	
	//subscriber for camera commands
	ros::Subscriber cmd_sub = nh.subscribe("rover_command", 1000, cmd_callback);
	
	for (int i=0;i<5,i++){
		if cam_array[i] == 1{
			stringstream ss;
			ss << "cam_" << i << "/compressed";
			node_name = ss.str();
			
			image_transport::Subscriber img_sub = it.subscribe(node_name,1,image_callback);
		}
	}
	
	ros::spin();
}
