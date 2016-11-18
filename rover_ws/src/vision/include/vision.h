#ifndef VISION_H
#define VISION_H

#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/SetCameraInfo.h>

namespace vision
{

class Vision
{
public:
    Vision();
    ~Vision();

    void publish_image();
    void publish_camera_info();

protected:
    // ros stuff for this node
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    //publications
    ros::Publisher image_publisher_;
    ros::Publisher camera_publisher_;

    ros::Publisher left_image_publisher_;
    ros::Publisher left_camera_publisher_;

    //services
    ros::ServiceServer srv_request_set_cam_info_;
    ros::ServiceServer srv_request_set_left_cam_info_;

    bool requestSetCamInfoService(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &res);
    bool requestSetLeftCamInfoService(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &res);

    int device1, device2;
    cv::VideoCapture v1, v2;
    bool use_stereo_;
    std::string file_path_;
};

} //end namespace

#endif //VISION_H
