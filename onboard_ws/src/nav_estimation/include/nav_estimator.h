#ifndef NAV_ESTIMATOR
#define NAV_ESTIMATOR

#define GRAVITY 9.81
#define EARTH_RADIUS 6378145.0

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include "rover_msgs/NavState.h"
#include "rover_msgs/Drive.h"

#include <math.h>
#include <Eigen/Eigen>

namespace nav_estimator
{

class NavEstimator{
public:
    NavEstimator();
    ~NavEstimator();

private:
    // ros stuff for this node
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber gps_subscriber_;
    ros::Subscriber gps_vel_subscriber_;
    ros::Subscriber imu_subscriber_;
    ros::Subscriber drive_command_subscriber_;
    ros::Publisher state_publisher_;

    std::string gps_topic_;
    std::string gps_vel_topic_;
    std::string imu_topic_;
    std::string drive_command_topic_;
    std::string state_topic_;

    //callbacks
    void gpsCallback(const sensor_msgs::NavSatFix &msg);
    void gpsVelCallback(const geometry_msgs::TwistStamped &msg);
    void imuCallback(const sensor_msgs::Imu &msg);
    void driveCallback(const rover_msgs::Drive &msg);

    //initialization
    bool gps_initialized_;
    double base_latitude_;
    double base_longitude_;
    double base_altitude_;
    ros::Time old;
    void initialize();

    double lpf_a_;
    double drive_gain_;
    float alpha;
    int N_;
    float phat;
    float qhat;
    float rhat;
    float Vwhat;
    float phihat;
    float thetahat;
    float psihat;
    Eigen::VectorXf xhat_p; // 7
    Eigen::MatrixXf P_p;  // 7x7

//    float gps_n_old;
//    float gps_e_old;
//    float gps_Vg_old;
//    float gps_course_old;

    Eigen::MatrixXf Q_p;  // 7x7
    Eigen::MatrixXf R_p;  // 6x6
    Eigen::VectorXf f_p;  // 7
    Eigen::MatrixXf A_p;  // 7x7
    float h_p;
    Eigen::VectorXf C_p;  // 7
    Eigen::VectorXf L_p;  // 7

    void prediction(float Ts);

    struct input_s{
        bool gps_vel_new;
        bool gps_new;
        float gps_n;
        float gps_e;
        float gps_h;
        float gps_Vg;
        float gps_course;
    };

    struct input_s input;

    void update();

    void publish();
};
} //end namespace


#endif //NAV_ESITMATOR
