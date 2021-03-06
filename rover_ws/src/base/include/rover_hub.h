#ifndef ROVER_HUB_H
#define ROVER_HUB_H


#include <ros/ros.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <rover_msgs/RoverState.h>
#include "sensor_msgs/CompressedImage.h"

namespace rover_hub
{

class Rover_hub
{

public:

    Rover_hub();
    ~Rover_hub();

private:
    // ros stuff for this node
    

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    image_transport::ImageTransport it;

    //
    ros::Subscriber state_sub;

    //image subsribers
    image_transport::Subscriber img_sub0;
    image_transport::Subscriber img_sub1;
    image_transport::Subscriber img_sub2;
    image_transport::Subscriber img_sub3;
    image_transport::Subscriber img_sub4;

    //image publishers
    image_transport::Publisher img_pub0;
    image_transport::Publisher img_pub1;
    image_transport::Publisher img_pub2;
    image_transport::Publisher img_pub3;
    image_transport::Publisher img_pub4;


    //calback functions
    //change
    void toggle_callback(const rover_msgs::RoverState::ConstPtr& msg);

    void image_callback0(const sensor_msgs::ImageConstPtr& msg);
    void image_callback1(const sensor_msgs::ImageConstPtr& msg);
    void image_callback2(const sensor_msgs::ImageConstPtr& msg);
    void image_callback3(const sensor_msgs::ImageConstPtr& msg);
    void image_callback4(const sensor_msgs::ImageConstPtr& msg);
    //JOY OBJECT


    //camera control stuff
    int counter;
    #define num_cam 4

    ros::Time begin;


};
} // end namespace

#endif //ROVER_HUB_H
