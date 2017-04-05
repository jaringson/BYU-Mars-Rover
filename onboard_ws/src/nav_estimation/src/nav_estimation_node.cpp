#include <ros/ros.h>
#include "nav_estimator.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nav_estimation");
    ros::NodeHandle nh;

    nav_estimator::NavEstimator est;
    ros::spin();
}
