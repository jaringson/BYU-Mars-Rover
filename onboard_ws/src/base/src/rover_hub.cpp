//class rover pub

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <rover_msgs/RoverState.h>
#include "rover_hub.h"


namespace rover_hub
{

Rover_hub::Rover_hub():
    it(image_transport::ImageTransport(nh_))

{
    //set image_transport node

    // TODO change this subscriber to subscribe to rover states
    state_sub = nh_.subscribe<rover_msgs::RoverState>("rover_state_cmd", 1, &Rover_hub::toggle_callback, this);

    //initialize subscribers
    img_sub0 = it.subscribe("/rgb/image_raw_color",10, &Rover_hub::image_callback0, this);

    img_sub1 = it.subscribe("/usb_cam1/image_raw",10, &Rover_hub::image_callback1, this);

    img_sub2 = it.subscribe("/usb_cam2/image_raw",10, &Rover_hub::image_callback2, this);

    img_sub3 = it.subscribe("/usb_cam3/image_raw",10, &Rover_hub::image_callback3, this);

    img_sub4 = it.subscribe("/usb_cam4/image_raw",10, &Rover_hub::image_callback4, this);

    //initialize publishers (do not yet publish)
    img_pub0 = it.advertise("base_cam0",1);
    img_pub1 = it.advertise("base_cam1",1);
    img_pub2 = it.advertise("base_cam2",1);
    img_pub3 = it.advertise("base_cam3",1);
    img_pub4 = it.advertise("base_cam4",1);


    //initialize camera control stuff
    counter = 0;

    //camera_toggle = RoverState();

    //initialize timer
    begin = ros::Time::now();
}

Rover_hub::~Rover_hub()
{

}

// TODO instead of sensor Msgs, we wand RoverState msgs.
void Rover_hub::toggle_callback(const rover_msgs::RoverState::ConstPtr& msg){
//ASK BRIAN AND MICHAEL

//use ROS timer for debouncing
ros::Time tempTime = ros::Time::now();

bool camtoggle = msg->camtoggle1;
//if it's been over 1/4 second since you switched, switch
if((tempTime - begin).toSec() > 0.25 && camtoggle){
    counter ++;
    if(counter == num_cam)
        counter = 0;
    begin = tempTime;
}

ROS_INFO_STREAM("Camera " << counter < "is selected");
return;
}

void Rover_hub::image_callback0(const sensor_msgs::ImageConstPtr& msg){

    if (counter == 0){


        img_pub0.publish(msg);
        ROS_INFO_STREAM("Camera 0 is publishing. counter number is: " << counter);
    }

}
void Rover_hub::image_callback1(const sensor_msgs::ImageConstPtr& msg){

    if (counter == 1){

        img_pub1.publish(msg);
        ROS_INFO_STREAM("Camera 1 is publishing. counter number is: " << counter);
    }
}

void Rover_hub::image_callback2(const sensor_msgs::ImageConstPtr& msg){
    if (counter == 2){


        img_pub2.publish(msg);
        ROS_INFO_STREAM("Camera 2 is publishing. counter number is: " << counter);
    }

}

void Rover_hub::image_callback3(const sensor_msgs::ImageConstPtr& msg){

    if(counter ==3){


        img_pub3.publish(msg);
        ROS_INFO_STREAM("Camera 3 is publishing. counter number is: " << counter);
    }
}

void Rover_hub::image_callback4(const sensor_msgs::ImageConstPtr& msg){
    if (counter == 4){

        img_pub4.publish(msg);
        ROS_INFO_STREAM("Camera 4 is publishing. counter number is: " << counter);
    }
}


}

/*
different callback for each
pass in msg type and subscriber number
*/

