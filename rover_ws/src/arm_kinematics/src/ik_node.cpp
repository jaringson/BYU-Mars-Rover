#include <iostream>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>

using namespace std;

class Arm_IK {
    private:
        string chain_start;
        string chain_end;
        string urdf_param;
        double timeout;
        double eps; 
        
        int numJoints;
       
        TRAC_IK::TRAC_IK ik_solver; 
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Subscriber sub;
        
        KDL::Chain chain;
        KDL::JntArray nominal;
        KDL::JntArray Ll, Ul; // Upper and lower joint limits
        KDL::Frame pose; // End Effector Pose
        KDL::JntArray result; // New joint angles
        
    public:
        Arm_IK(int, char**);
        bool SolverInit();
        void poseMessageReceived(const geometry_msgs::Pose&);
};


Arm_IK::Arm_IK(int argc, char** argv)
 : ik_solver("link1", "link7", "/robot_description", 0, 0)
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
    
    // Set up Publisher and Subscriber
    pub = nh.advertise<sensor_msgs::JointState>("joint_cmd",1000);
    sub = nh.subscribe("pose_cmd", 1000, &Arm_IK::poseMessageReceived, this);
    
    // Start Solver
    bool valid = SolverInit();
    
    // Initialize Variables
    nominal.resize(numJoints);
    for (int i=0; i<numJoints; i++) {
        nominal(i) = 0;
    }
    
    
}

bool Arm_IK::SolverInit() 
{
    // Set up TRAC_IK Solver
    TRAC_IK::TRAC_IK ik_solver(chain_start, chain_end, urdf_param, timeout, eps);

    // Get KDL Chain
    bool valid = ik_solver.getKDLChain(chain);
    if (!valid) {
        ROS_ERROR("There was no valid KDL chain found");
    }
    
    // Get Joint Limits
    valid = ik_solver.getKDLLimits(Ll,Ul);
    if (!valid) {
        ROS_ERROR("There were no valid KDL joint limits found");
    }
    
    // Get Number of Joints
    numJoints = chain.getNrOfJoints();
    assert(chain.getNrOfJoints() == Ll.data.size());
    assert(chain.getNrOfJoints() == Ul.data.size());
    
    return valid;       
}

void Arm_IK::poseMessageReceived(const geometry_msgs::Pose& posemsg) {
    pose.p = KDL::Vector(posemsg.position.x, posemsg.position.y, posemsg.position.z);
    pose.M = KDL::Rotation::Quaternion(posemsg.orientation.x, posemsg.orientation.y, posemsg.orientation.z, posemsg.orientation.w);
    
    int status = -1;
    do {
        status = ik_solver.CartToJnt(nominal, pose, result);
    } while(status < 0);
}

int main(int argc, char** argv) {
    
    if (false) {

    }
    else {
        ros::init(argc, argv, "arm_ik_tests");
        Arm_IK(argc,argv);
    }
    
    cout << "Hello World" << endl;
    return 0;
}
