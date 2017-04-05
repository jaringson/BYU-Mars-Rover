#include "nav_estimator.h"
#include "tf/transform_datatypes.h"

namespace nav_estimator
{

NavEstimator::NavEstimator():
    nh_(ros::NodeHandle()),
    nh_private_(ros::NodeHandle("~")),
    xhat_p(7),
    P_p(7,7),
    Q_p(7,7),
    R_p(7,7),
    f_p(7),
    A_p(7,7),
    C_p(7),
    L_p(7)
{
    phat = 0;
    qhat = 0;
    rhat = 0;
    phihat = 0;
    thetahat = 0;
    psihat = 0;
    Vwhat = 0;

    input.gps_vel_new = false;
    input.gps_new = false;

    nh_private_.param<std::string>("gps_topic", gps_topic_, "fix");
    nh_private_.param<std::string>("gps_vel_topic", gps_vel_topic_, "vel");
    nh_private_.param<std::string>("imu_topic", imu_topic_, "imu");
    nh_private_.param<std::string>("drive_topic", drive_command_topic_, "drive_command");
    nh_private_.param<std::string>("state_topic", state_topic_, "nav_state");
    nh_private_.param<int>("iterations_per_prediction", N_, 10);
    nh_private_.param<double>("low_pass_filter", lpf_a_, 50.0);
    nh_private_.param<double>("drive_gain", drive_gain_, 0.003511); // 2rev/sec & 11 in dia wheel

    gps_subscriber_ = nh_.subscribe(gps_topic_, 1, &NavEstimator::gpsCallback, this);
    gps_vel_subscriber_ = nh_.subscribe(gps_vel_topic_, 1, &NavEstimator::gpsVelCallback, this);
    imu_subscriber_ = nh_.subscribe(imu_topic_, 1, &NavEstimator::imuCallback, this);
    drive_command_subscriber_ = nh_.subscribe(drive_command_topic_, 1, &NavEstimator::driveCallback, this);
    state_publisher_ = nh_.advertise<rover_msgs::NavState>(state_topic_, 1);
}

NavEstimator::~NavEstimator()
{}

double radians(double degrees){ return M_PI*degrees/180.0;  }
float fradians(float degrees){ return M_PI*degrees/180.0;  }

double degrees(double radians){ return 180.0*radians/M_PI;  }

void NavEstimator::gpsCallback(const sensor_msgs::NavSatFix& msg)
{
        // reset for non-Kalman filter publishing
        base_latitude_ = msg.latitude;
        base_longitude_ = msg.longitude;
        base_altitude_ = msg.altitude;


    if(!gps_initialized_ && !(msg.latitude != msg.latitude))
    {
        base_latitude_ = msg.latitude;
        base_longitude_ = msg.longitude;
        base_altitude_ = msg.altitude;

        initialize();
        gps_initialized_ = true;
    }
    else
    {
        input.gps_n = EARTH_RADIUS * radians(msg.latitude - base_latitude_);
        input.gps_e = EARTH_RADIUS * cos(radians(base_latitude_)) * radians(msg.longitude - base_longitude_);
        input.gps_h = msg.altitude - base_altitude_;

        // reset for non-Kalman filter publishing
        base_latitude_ = msg.latitude;
        base_longitude_ = msg.longitude;
        base_altitude_ = msg.altitude;

        if(!(input.gps_n != input.gps_n))
        {
            input.gps_new = true;
            update();
        }
    }
}

void NavEstimator::gpsVelCallback(const geometry_msgs::TwistStamped& msg)
{
    input.gps_Vg = sqrt(msg.twist.linear.x*msg.twist.linear.x + msg.twist.linear.y*msg.twist.linear.y);
    input.gps_course = atan2(msg.twist.linear.y,msg.twist.linear.x);

    if(!(input.gps_Vg != input.gps_Vg))
    {
        input.gps_vel_new;
        update();
    }
}

void NavEstimator::imuCallback(const sensor_msgs::Imu &msg)
{
    ros::Time now = ros::Time::now();
    float Ts = (float)(now.sec - old.sec) + (float)(((int32_t)now.nsec - (int32_t)old.nsec)/1e9);
    old = now;

    if(Ts > 0.5)
        ROS_WARN_STREAM("estimation time step was to big: " << Ts);
    else
    {
        alpha = exp(-lpf_a_*Ts);

        phat = alpha*phat + (1-alpha)*msg.angular_velocity.x;
        qhat = alpha*qhat + (1-alpha)*msg.angular_velocity.y;
        rhat = alpha*rhat + (1-alpha)*msg.angular_velocity.z;

        double phi, theta, psi;
        tf::Quaternion q(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(phi, theta, psi);
        phihat = phi;
        thetahat = theta;
        psihat = psi;

        prediction(Ts);

        if(gps_initialized_)
            publish();
    }
}

void NavEstimator::driveCallback(const rover_msgs::Drive &msg)
{
    Vwhat = ((msg.rw - 1500) - (msg.lw - 1500))*drive_gain_/2; // there may be a minus sign wrong here
}

void NavEstimator::initialize()
{
    //filter initialization
    xhat_p = Eigen::VectorXf::Zero(7);
    xhat_p(2) = 0.0f;
    P_p = Eigen::MatrixXf::Identity(7,7);
    P_p(0,0) = 0.03;
    P_p(1,1) = 0.03;
    P_p(2,2) = 0.01;
    P_p(3,3) = fradians(5.0f);
    P_p(4,4) = 0.04;
    P_p(5,5) = 0.04;
    P_p(6,6) = fradians(5.0f);

    old = ros::Time::now();
}

void NavEstimator::prediction(float Ts)
{
    float psidot, tmp, Vgdot;
    if(fabsf(xhat_p(2)) < 0.01f)
    {
        xhat_p(2) = 0.01;
    }

    for(int i=0;i<N_;i++)
    {
        psidot = (qhat*sinf(phihat) + rhat*cosf(phihat))/cosf(thetahat);
        tmp = -psidot*Vwhat*(xhat_p(4)*cosf(xhat_p(6)) + xhat_p(5)*sinf(xhat_p(6)))/xhat_p(2);
        Vgdot = ((Vwhat*cosf(xhat_p(6)) + xhat_p(4))*(-psidot*Vwhat*sinf(xhat_p(6))) + (Vwhat*sinf(xhat_p(6)) + xhat_p(5))*(psidot*Vwhat*cosf(xhat_p(6))))/xhat_p(2);

        f_p = Eigen::VectorXf::Zero(7);
        f_p(0) = xhat_p(2)*cosf(xhat_p(3));
        f_p(1) = xhat_p(2)*sinf(xhat_p(3));
        f_p(2) = Vgdot;
        f_p(3) = GRAVITY/xhat_p(2)*tanf(phihat)*cosf(xhat_p(3) - xhat_p(6));
        f_p(6) = psidot;

        A_p = Eigen::MatrixXf::Zero(7,7);
        A_p(0,2) = cos(xhat_p(3));
        A_p(0,3) = -xhat_p(2)*sinf(xhat_p(3));
        A_p(1,2) = sin(xhat_p(3));
        A_p(1,3) = xhat_p(2)*cosf(xhat_p(3));
        A_p(2,2) = -Vgdot/xhat_p(2);
        A_p(2,4) = -psidot*Vwhat*sinf(xhat_p(6))/xhat_p(2);
        A_p(2,5) = psidot*Vwhat*cosf(xhat_p(6))/xhat_p(2);
        A_p(2,6) = tmp;
        A_p(3,2) = -GRAVITY/powf(xhat_p(2),2)*tanf(phihat)*cosf(xhat_p(3) - xhat_p(6));
        A_p(3,3) = -GRAVITY/xhat_p(2)*tanf(phihat)*sinf(xhat_p(3) - xhat_p(6));
        A_p(3,6) = GRAVITY/xhat_p(2)*tanf(phihat)*sinf(xhat_p(3) - xhat_p(6));

        xhat_p += f_p *(Ts/N_);
        P_p += (A_p*P_p + P_p*A_p.transpose() + Q_p)*(Ts/N_);
    }
}

void NavEstimator::update()
{
    Eigen::MatrixXf I_p;
    I_p = Eigen::MatrixXf::Identity(7,7);
    Eigen::MatrixXf denom(1,1);

    if(input.gps_new)
    {
        input.gps_new = false;

        // gps North position
        h_p = xhat_p(0);
        C_p = Eigen::VectorXf::Zero(7);
        C_p(0) = 1;
        denom(0,0) = (R_p(0,0) + (C_p.transpose()*P_p*C_p)(0,0));
        L_p = (P_p*C_p) * denom.inverse();
        P_p = (I_p - L_p*C_p.transpose())*P_p;
        xhat_p = xhat_p + L_p*(input.gps_n - h_p);
        
        // gps East position
        h_p = xhat_p(1);
        C_p = Eigen::VectorXf::Zero(7);
        C_p(1) = 1;
        denom(0,0) = (R_p(1,1) + (C_p.transpose()*P_p*C_p)(0,0));
        L_p = (P_p*C_p) * denom.inverse();
        P_p = (I_p - L_p*C_p.transpose())*P_p;
        xhat_p = xhat_p + L_p*(input.gps_e - h_p);
    }

    if(input.gps_vel_new)
    {
        input.gps_vel_new = false;

        // gps ground speed
        h_p = xhat_p(2);
        C_p = Eigen::VectorXf::Zero(7);
        C_p(2) = 1;
        denom(0,0) = (R_p(2,2) + (C_p.transpose()*P_p*C_p)(0,0));
        L_p = (P_p*C_p) * denom.inverse();
        P_p = (I_p - L_p*C_p.transpose())*P_p;
        xhat_p = xhat_p + L_p*(input.gps_Vg - h_p);

        // gps course
        //wrap course measurement
        float gps_course = fmodf(input.gps_course, fradians(360.0f));

        while(gps_course - xhat_p(3) > fradians(180.0f)) gps_course = gps_course - fradians(360.0f);
        while(gps_course - xhat_p(3) < fradians(-180.0f)) gps_course = gps_course + fradians(360.0f);
        h_p = xhat_p(3);
        C_p = Eigen::VectorXf::Zero(7);
        C_p(3) = 1;
        denom(0,0) = (R_p(3,3) + (C_p.transpose()*P_p*C_p)(0,0));
        L_p = (P_p*C_p) * denom.inverse();
        P_p = (I_p - L_p*C_p.transpose())*P_p;
        xhat_p = xhat_p + L_p*(gps_course - h_p);
    }

    if(xhat_p(0) > 3000 || xhat_p(0) < -3000 || xhat_p(1) > 3000 || xhat_p(1) < -3000)
        gps_initialized_ = false;

    for(int i=0;i<7;i++)
    {
        if(xhat_p(i) != xhat_p(i))
        {
            initialize();

            switch(i)
            {
            case 0:
                xhat_p(i) = input.gps_n;
                break;
            case 1:
                xhat_p(i) = input.gps_e;
                break;
            case 2:
                xhat_p(i) = input.gps_Vg;
                break;
            case 3:
                xhat_p(i) = psihat;
                break;
            case 6:
                xhat_p(i) = psihat;
                break;
            default:
                xhat_p(i) = 0;
            }
        }
    }
}

void NavEstimator::publish()
{
    rover_msgs::NavState state_msg;
    state_msg.position[0] = xhat_p(0);
    state_msg.position[1] = xhat_p(1);
    state_msg.position[2] = input.gps_h;
    state_msg.Vw = Vwhat;
    state_msg.phi = phihat;
    state_msg.theta = thetahat;
    state_msg.psi = psihat;
    state_msg.chi = xhat_p(2);
    state_msg.p = phat;
    state_msg.q = qhat;
    state_msg.r = rhat;
    state_msg.Vg = xhat_p(3);
    state_msg.base_latitude = base_latitude_;
    state_msg.base_longitude = base_longitude_;
    state_msg.base_altitude = base_altitude_;

    state_publisher_.publish(state_msg);
}

}
