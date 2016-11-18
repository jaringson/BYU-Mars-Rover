/*!
 * \file vision_node.cpp
 * \author Gary Ellingson
 *
 * \brief This file provides the entry point for the vision node
 *
 * You will create one of these files for each node in your package. The file should be named <node name>_node.cpp. In
 * general you will want to keep this file as short as possible and do as much of the work as possible within a separate
 * class. A minimal <node name>_node.cpp file will generally have a main function that does the following:
 *   1. Initialize the node
 *   2. Create a NodeHandle for the node
 *   3. Instantiate an object of the main class for your node
 *   4. Spin
 */

#include "ros/ros.h"
#include "vision.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    int rate;
    nh_private.param<int>("rate", rate, 10);

    vision::Vision vis;
    ros::Rate loop_rate(rate);
    while(ros::ok())
    {
        ros::spinOnce();
        vis.publish_image();

        loop_rate.sleep();
        //ROS_WARN_STREAM("Spinning");
    }

//    ros::spin();
    return 0;
}
