// //class rover pub

// #include "ros/ros.h"
// #include "std_msgs/String.h"
// #include <image_transport/image_transport.h>
// #include "rover_msgs/RoverState.h"

// class rover_pub{
// 	private:
// 		//image transporter node
// 		image_transport::ImageTransport it;
		
// 		//image subsribers
// 		image_transport::Subscriber img_sub0;
// 		image_transport::Subscriber img_sub1;
// 		image_transport::Subscriber img_sub2;
// 		image_transport::Subscriber img_sub3;
// 		image_transport::Subscriber img_sub4;
		
// 		//image publishers
// 		image_transport::Publisher img_pub0;
// 		image_transport::Publisher img_pub1;
// 		image_transport::Publisher img_pub2;
// 		image_transport::Publisher img_pub3;
// 		image_transport::Publisher img_pub4;
		
// 		//JOY OBJECT
		
		
// 		//camera control stuff
// 		int counter;
// 		const int NUM_CAM;
		
// 		ros::Time begin;
// }

// rover_pub::rover_pub(ros::NodeHandle nh){
// 	//set image_transport node
// 	image_transport::ImageTransport it(nh);
	
// 	//initialize subscribers
// 	img_sub0 = it.subscribe("usb_cam0/image_raw/theora",10,image_callback);
// 	img_sub1 = it.subscribe("usb_cam1/image_raw/theora",10,image_callback);
// 	img_sub2 = it.subscribe("usb_cam2/image_raw/theora",10,image_callback);
// 	img_sub3 = it.subscribe("usb_cam3/image_raw/theora",10,image_callback);
// 	img_sub4 = it.subscribe("usb_cam4/image_raw/theora",10,&Rover_pub::image_callback,this);
	
// 	//initialize publishers (do not yet publish)
// 	img_pub0 = it.advertise("base_cam0/image_raw/theora",1);
// 	img_pub1 = it.advertise("base_cam1/image_raw/theora",1);
// 	img_pub2 = it.advertise("base_cam2/image_raw/theora",1);
// 	img_pub3 = it.advertise("base_cam3/image_raw/theora",1);
// 	img_pub4 = it.advertise("base_cam4/image_raw/theora",1);
	
// 	//INITIALIZE JOY OBJECT
	
// 	//initialize camera control stuff
// 	counter = 0;
// 	NUM_CAM = 5;
	
// 	//initialize timer
// 	begin = ros::Time::now();
// }

// void rover_pub::joy_callback(WHATEVER MESSAGE TYPE msg){
// 	//ASK BRIAN AND MICHAEL
	
// 	//use ROS timer for debouncing
// 	ros::Time tempTime = ros::Time::now();
	
// 	//if it's been over 1/4 second since you switched, switch
// 	if(temp - begin > 0.25){
// 		counter ++;
// 		if(counter == NUM_CAM)
// 			counter = 0;
// 		begin = temp;
// 	}
		
// 	ROS_INFO_STREAM("Camera " << counter < "is selected");
// 	return;
// }

// void rover_pub::image_callback(const sensor_msgs::ImageConstPtr& msg){
// 	switch counter{
// 		case 0:
		
// 		img_pub0.publish(msg);
// 		ROS_INFO_STREAM("Camera " << counter < "is publishing");
// 		break;
		
// 		case 1:
		
// 		img_pub1.publish(msg);
// 		ROS_INFO_STREAM("Camera " << counter < "is publishing");
// 		break;
		
// 		case 2:
		
// 		img_pub2.publish(msg);
// 		ROS_INFO_STREAM("Camera " << counter < "is publishing");
// 		break;
		
// 		case 3:
		
// 		img_pub3.publish(msg);
// 		ROS_INFO_STREAM("Camera " << counter < "is publishing");
// 		break;
		
// 		default:
		
// 		img_pub4.publish(msg);
// 		ROS_INFO_STREAM("Camera " << counter < "is publishing");
// 		break;
// 	}
// }

// /*
// different callback for each
// pass in msg type and subscriber number
// */

int main (){


	return 1;
}