/*
 * \file vision.cpp
 * \author Gary Ellingson
 *
 * \brief This is the implementation file for the Vision class
 */

#include "vision.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

namespace vision
{

Vision::Vision():
    nh_(ros::NodeHandle()),
    nh_private_(ros::NodeHandle("~"))
{
    nh_private_.param<int>("camera1", device1, 0);
    nh_private_.param<int>("camera2", device2, 1);
    nh_private_.param<bool>("useStereo", use_stereo_, false);
    nh_private_.param<std::string>("filePath", file_path_, "~/rostesting/rover_ws/src/vision/config");

    if(use_stereo_)
    {
        if(!v1.open(device1) || !v2.open(device2))
            ROS_ERROR("could not open one or both cameras");

        image_publisher_ = nh_.advertise<sensor_msgs::Image>("right_image", 1);
        camera_publisher_ = nh_.advertise<sensor_msgs::CameraInfo>("right_camera", 1);

        srv_request_set_cam_info_ = nh_.advertiseService("right_camera/set_camera_info", &Vision::requestSetCamInfoService, this);

        left_image_publisher_ = nh_.advertise<sensor_msgs::Image>("left_image", 1);
        left_camera_publisher_ = nh_.advertise<sensor_msgs::CameraInfo>("left_camera", 1);

        srv_request_set_left_cam_info_ = nh_.advertiseService("left_camera/set_camera_info", &Vision::requestSetLeftCamInfoService, this);
    }
    else
    {
        if(!v1.open(device1))
            ROS_ERROR("could not open the camera");

        image_publisher_ = nh_.advertise<sensor_msgs::Image>("image", 1);
        camera_publisher_ = nh_.advertise<sensor_msgs::CameraInfo>("camera", 1);

        srv_request_set_cam_info_ = nh_.advertiseService("camera/set_camera_info", &Vision::requestSetCamInfoService, this);
    }
}

Vision::~Vision()
{
    v1.release(); v2.release();
}

void Vision::publish_image()
{
    cv::Mat frame1, frame2;

    cv_bridge::CvImage cv_img;

    if(v1.isOpened())
    {
        v1 >> frame1;
        cv_img.image = frame1;
        cv_img.encoding = sensor_msgs::image_encodings::BGR8;
        image_publisher_.publish(cv_img.toImageMsg());
    }

    if(use_stereo_ && v2.isOpened())
    {
        v2 >> frame2;
        cv_img.image = frame2;
        cv_img.encoding = sensor_msgs::image_encodings::BGR8;
        left_image_publisher_.publish(cv_img.toImageMsg());
    }

    publish_camera_info();
}

void Vision::publish_camera_info()
{
    cv::Mat K, R, P;
    int h, w, x, y;

    {
        sensor_msgs::CameraInfo cam;
        std::stringstream s;
        s << file_path_ << "/cameraParam.yaml";
        cv::FileStorage fs(s.str(), cv::FileStorage::READ);

        fs["height"] >> h; cam.height = (uint32_t)h;
        fs["width"] >> w; cam.width = (uint32_t)w;
        fs["distortion_model"] >> cam.distortion_model;
        fs["D"] >> cam.D;
        fs["camMatrix"] >> K; for(int i(0);i<9;i++) {cam.K[i] = K.at<double>(i);}
        fs["sterRect"] >> R; for(int i(0);i<9;i++) {cam.R[i] = R.at<double>(i);}
        fs["project"] >> P; for(int i(0);i<12;i++) {cam.P[i] = P.at<double>(i);}
        //fs["binning_x"] >> x; cam.binning_x = (uint32_t)x; fs["binning_y"] >> y; cam.binning_y = (uint32_t)y;
        //fs["roi_h"] >> h; cam.roi.height = (uint32_t)h; fs["roi_w"] >> w; cam.roi.width = (uint32_t)w;
        fs.release();
        camera_publisher_.publish(cam);
    }

    if(use_stereo_)
    {
        sensor_msgs::CameraInfo cam;
        std::stringstream s;
        s << file_path_ << "/leftCameraParam.yaml";
        cv::FileStorage fs(s.str(), cv::FileStorage::READ);

        fs["height"] >> h; cam.height = (uint32_t)h;
        fs["width"] >> w; cam.width = (uint32_t)w;
        fs["distortion_model"] >> cam.distortion_model;
        fs["D"] >> cam.D;
        fs["camMatrix"] >> K; for(int i(0);i<9;i++) {cam.K[i] = K.at<double>(i);}
        fs["sterRect"] >> R; for(int i(0);i<9;i++) {cam.R[i] = R.at<double>(i);}
        fs["project"] >> P; for(int i(0);i<12;i++) {cam.P[i] = P.at<double>(i);}
        //fs["binning_x"] >> x; cam.binning_x = (uint32_t)x; fs["binning_y"] >> y; cam.binning_y = (uint32_t)y;
        //fs["roi_h"] >> h; cam.roi.height = (uint32_t)h; fs["roi_w"] >> w; cam.roi.width = (uint32_t)w;
        fs.release();
        left_camera_publisher_.publish(cam);
    }
}

bool Vision::requestSetCamInfoService(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &res)
{
    std::stringstream s;
    s << file_path_ << "/cameraParam.yaml";
    cv::FileStorage fs(s.str(), cv::FileStorage::WRITE);
    fs << "height" << (int)req.camera_info.height << "width" << (int)req.camera_info.width;
    fs << "distortion_model" << req.camera_info.distortion_model << "D" << req.camera_info.D;
    fs << "camMatrix" << cv::Mat(3,3,CV_64FC1,&req.camera_info.K);
    fs << "sterRect" << cv::Mat(3,3,CV_64FC1,&req.camera_info.R);
    fs << "project" << cv::Mat(3,4,CV_64FC1,&req.camera_info.P);
    //fs << "binning_x" << (int)req.camera_info.binning_x << "binning_y" << (int)req.camera_info.binning_y;
    //fs << "roi_h" << (int)req.camera_info.roi.height << "roi_w" << (int)req.camera_info.roi.width;
    fs.release();
    res.success = true;
    return true;
}

bool Vision::requestSetLeftCamInfoService(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &res)
{
    std::stringstream s;
    s << file_path_ << "/leftCameraParam.yaml";
    cv::FileStorage fs(s.str(), cv::FileStorage::WRITE);
    fs << "height" << (int)req.camera_info.height << "width" << (int)req.camera_info.width;
    fs << "distortion_model" << req.camera_info.distortion_model << "D" << req.camera_info.D;
    fs << "camMatrix" << cv::Mat(3,3,CV_64FC1,&req.camera_info.K);
    fs << "sterRect" << cv::Mat(3,3,CV_64FC1,&req.camera_info.R);
    fs << "project" << cv::Mat(3,4,CV_64FC1,&req.camera_info.P);
    //fs << "binning_x" << (int)req.camera_info.binning_x << "binning_y" << (int)req.camera_info.binning_y;
    //fs << "roi_h" << (int)req.camera_info.roi.height << "roi_w" << (int)req.camera_info.roi.width;
    fs.release();
    res.success = true;
    return true;
}

} //namespace vision

