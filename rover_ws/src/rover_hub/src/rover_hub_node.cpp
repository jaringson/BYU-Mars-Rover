#include "ros/ros.h"
#include "rover_hub.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rover_hub_node");

//    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    int rate;
    nh_private.param<int>("rate", rate, 100);

    rover_hub::Rover_hub rh;
    ros::Rate loop_rate(rate);
    while(ros::ok())
    {
        rh;
        ros::spinOnce();
//        ro.publish_image();

        loop_rate.sleep();
        //ROS_WARN_STREAM("Spinning");
    }

//    ros::spin();
    return 0;
}
