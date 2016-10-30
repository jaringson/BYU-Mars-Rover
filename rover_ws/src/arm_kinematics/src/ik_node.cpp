#include <iostream>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <ostream>

using namespace std;

class Arm_IK {
    private:
        string chain_start;
        string chain_end;
        string urdf_param;
        double timeout;
        double eps; 
        
        int numJoints;
       
        TRAC_IK::TRAC_IK* ik_solver; 
        ros::NodeHandle nh;
        ros::Publisher pub_joints;
        ros::Subscriber sub_joints;
        ros::Publisher pub_pose;
        ros::Subscriber sub_pose;
        
        KDL::Chain chain;
        KDL::ChainFkSolverPos_recursive*  fk_solver;
        KDL::JntArray nominal;
        KDL::JntArray Ll, Ul; // Upper and lower joint limits
        KDL::Frame pose; // End Effector Pose
        KDL::JntArray JointAngles; // New joint angles
        
    public:
        Arm_IK(int, char**);
        ~Arm_IK() {delete fk_solver;};
        bool SolverInit();
        void poseMessageReceived(const geometry_msgs::Pose&);
        void jointMessageReceived(const sensor_msgs::JointState&);
};


Arm_IK::Arm_IK(int argc, char** argv)
// : ik_solver("link1", "link7", "/robot_description", 0, 0)
{
    // Start ROS node
    ros::NodeHandle nh("~");
    
    // Get Parameters from server
    nh.param("chain_start", chain_start, string(""));
    nh.param("chain_end", chain_end, string(""));
    nh.param("timeout",timeout,0.005);
    nh.param("urdf_param",urdf_param, string("/robot_description"));
    eps = 1e-5;
    
    // Validate if Parameters were loaded
    if (chain_start=="" || chain_end=="") {
        ROS_FATAL("Missing chain info in launch file");
        exit(-1);
    }
    
    // Set up Publisher and Subscriber
    sub_joints = nh.subscribe("/joint_ik",1000, &Arm_IK::jointMessageReceived, this);
    pub_pose = nh.advertise<geometry_msgs::Pose>("/pose_cmd",1000);
    
    sub_pose = nh.subscribe("/pose_ik", 1000, &Arm_IK::poseMessageReceived, this);
    pub_joints = nh.advertise<sensor_msgs::JointState>("/joint_cmd",1000);
    
    cout << "Subscribers set up" << endl;
    
    // Start Solver
    bool valid = SolverInit();
    cout << "Number of Segments: " << chain.getNrOfSegments() << endl;
    cout << "Number of Joints: " << chain.getNrOfJoints() << endl;
    
    // Initialize Variables
    nominal.resize(numJoints);
    JointAngles.resize(numJoints);
    for (int i=0; i<numJoints; i++) {
        nominal(i) = 0;
        JointAngles(i) = 0;
    }

}

bool Arm_IK::SolverInit() 
{
    // Set up TRAC_IK Solver
    //TRAC_IK::TRAC_IK ik_solver(chain_start, chain_end, urdf_param, timeout, eps);
	ik_solver = new TRAC_IK::TRAC_IK(chain_start, chain_end, urdf_param, timeout, eps);
	
    // Get KDL Chain
    bool valid = ik_solver->getKDLChain(chain);
    if (!valid) {
        ROS_ERROR("There was no valid KDL chain found");
    }
    
    // Get Joint Limits
    valid = ik_solver->getKDLLimits(Ll,Ul);
    if (!valid) {
        ROS_ERROR("There were no valid KDL joint limits found");
    }
    
    // Get Number of Joints
    numJoints = chain.getNrOfJoints();
    assert(chain.getNrOfJoints() == Ll.data.size());
    assert(chain.getNrOfJoints() == Ul.data.size());

    // Set up KDL Forward Kinematics Solver
    fk_solver = new KDL::ChainFkSolverPos_recursive(chain);
    return valid;  
}

void Arm_IK::poseMessageReceived(const geometry_msgs::Pose& posemsg) {
    cout << "Received message: convert to joints" << endl;
    
    // Convert from Pose message to KDL Frame
    pose.p = KDL::Vector(posemsg.position.x, posemsg.position.y, posemsg.position.z);
    pose.M = KDL::Rotation::Quaternion(posemsg.orientation.x, posemsg.orientation.y, posemsg.orientation.z, posemsg.orientation.w);
    
    // Run IK Solver
    int status = -1;
    do {
        status = ik_solver->CartToJnt(nominal, pose, JointAngles);
    } while(status < 0);
	//status = ik_solver->CartToJnt(nominal, pose, JointAngles);
	cout << status << endl;
	cout << "Finished Solving" << endl;

    // Convert from KDL Joints to JointState Message
    sensor_msgs::JointState ikmsg;
    for (int i = 0; i<numJoints; i++) {
        ikmsg.position.push_back(JointAngles(i));
		ikmsg.velocity.push_back(0);
		string joint = "joint";
		stringstream sstm;
		sstm << joint << i+1;
		ikmsg.name.push_back(sstm.str());
    }

	// Update header
	ikmsg.header.stamp = ros::Time::now();
	ikmsg.header.frame_id = "IK_Node";
	
	fk_solver->JntToCart(JointAngles,pose);      
    pub_joints.publish(ikmsg);
    
}

void Arm_IK::jointMessageReceived(const sensor_msgs::JointState& jointmsgs) {
    cout << "Received message: convert to pose" << endl;
    
    // Convert jointstate message to KDL
    for (int i = 0; i <numJoints; i++) {
        JointAngles(i) = jointmsgs.position[i];
    }
    
    // Run FK Solver
    fk_solver->JntToCart(JointAngles,pose);
    
    // Convert KDL Frame to Pose Message
    geometry_msgs::Pose posemsg;
    posemsg.position.x = 0;//pose.p[0];
    posemsg.position.y = pose.p[1];
    posemsg.position.z = pose.p[2];
    double x,y,z,w;
    pose.M.GetQuaternion(x,y,z,w);
    posemsg.orientation.x = x;
    posemsg.orientation.y = y;
    posemsg.orientation.z = z;  
    posemsg.orientation.w = w;

    pub_pose.publish(posemsg);
    
}

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "arm_ik_tests");
    
    Arm_IK a(argc,argv);
    
    ros::spin();
    
    cout << "Hello World" << endl;
    return 0;
}
