#include <iostream>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <string>

using namespace std;

class Arm_IK {
    private:
       double eps; 
        
        
        
    public:
        Arm_IK();
};

Arm_IK::Arm_IK() {
        
    eps = 1e-5;
}


int main(int argc, char** argv) {
    
    int num_samples;
    double timeout;
    string chain_start;
    string chain_end;
    string urdf_param;
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
        
    Arm_IK ik();
    cout << "Hello World" << endl;
    return 0;
}
