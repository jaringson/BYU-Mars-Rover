#include <iostream>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <string>

using namespace std;

class Arm_IK {
    private:
        string chain_start;
        string chain_end;
        string urdf_param;
        double timeout;
        double eps; 
       
        TRAC_IK::TRAC_IK ik_solver; 
        ros::NodeHandle nh;
        
    public:
        Arm_IK(string, string, string, double);
        Arm_IK(int, char**);
        void SolverInit();
};

Arm_IK::Arm_IK(string chain_start, string chain_end, string urdf_param, double timeout)
 : ik_solver("", "", "/robot_description", 0, 1e-5)
{
    
    SolverInit();
    eps = 1e-5;
    
    
}

Arm_IK::Arm_IK(int argc, char** argv)
 : ik_solver("", "", "", 0, 0)
{
    // Start ROS node
    ros::NodeHandle nh("~");
    
    // Get Parameters from server
    nh.param("chain_start", chain_start, string(""));
    nh.param("chain_end", chain_end, string(""));
    nh.param("timeout",timeout,0.005);
    nh.param("urdf_param",urdf_param, string("/robot_description"));
    cout << urdf_param << endl;
    
    // Validate if Parameters were loaded
    if (chain_start=="" || chain_end=="") {
        ROS_FATAL("Missing chain info in launch file");
        exit(-1);
    }
    
}

void Arm_IK::SolverInit() 
{
    string chain_start = "link1";
    string chain_end = "link7";
    string urdf_param = "/robot_description";
    double timeout = 0.005;
    TRAC_IK::TRAC_IK ik_solver(chain_start, chain_end, urdf_param, timeout, eps);
}

int main(int argc, char** argv) {
    
    if (true) {
        int num_samples;
        double timeout;
        std::string chain_start;
        std::string chain_end;
        std::string urdf_param;
        // Start ROS node
        ros::init(argc, argv, "arm_ik_tests");
        ros::NodeHandle nh("~");
        
        // Get Parameters from server
        nh.param("num_samples", num_samples, 1000);
        nh.param("chain_start", chain_start, string(""));
        nh.param("chain_end", chain_end, string(""));
        nh.param("timeout",timeout,0.005);
        nh.param("urdf_param",urdf_param, string("/robot_description"));
        
        // Validate if Parameters were loaded
        if (chain_start=="" || chain_end=="") {
            ROS_FATAL("Missing chain info in launch file");
            exit(-1);
        }
        if (num_samples < 1)
            num_samples = 1;
            
        Arm_IK ik(chain_start, chain_end, urdf_param, timeout);
    }
    else {
        ros::init(argc, argv, "arm_ik_tests");
        Arm_IK(argc,argv);
    }
    
    cout << "Hello World" << endl;
    return 0;
}
