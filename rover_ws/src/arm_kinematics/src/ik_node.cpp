#include <iostream>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <string>

using namespace std;

class Arm_IK {
    private:
       double eps; 
       
       TRAC_IK::TRAC_IK ik_solver; 
        
    public:
        Arm_IK(std::string, std::string, std::string, double);
};

Arm_IK::Arm_IK(std::string chain_start, std::string chain_end, std::string urdf_param, double timeout)
 : ik_solver(chain_start, chain_end, urdf_param, timeout, 1e-5)
{
    eps = 1e-5;
    
    
    
}


int main(int argc, char** argv) {
    
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
    
    cout << "Hello World" << endl;
    return 0;
}
