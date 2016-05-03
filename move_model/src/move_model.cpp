#include <string>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

std::string jtag[35]
={	"WaistPitch",
	"WaistYaw",
	"L_ShoulderPitch",
	"L_ShoulderRoll",
	"L_ShoulderYaw",
	"L_ElbowRoll",
	"L_WristYaw",
	"L_WristRoll",
	"L_HandYaw",
	"L_Gripper_1",
	"L_Gripper_2",
	"R_ShoulderPitch",
	"R_ShoulderRoll",
	"R_ShoulderYaw",
	"R_ElbowRoll",
	"R_WristYaw",
	"R_WristRoll",
	"R_HandYaw",
	"R_Gripper_1",
	"R_Gripper_2",
	"HeadYaw",
	"HeadPitch",
	"ChestLidar",
	"L_HipYaw",
	"L_HipRoll",
	"L_HipPitch",
	"L_KneePitch",
	"L_AnklePitch",
	"L_AnkleRoll",
	"R_HipYaw",
	"R_HipRoll",
	"R_HipPitch",
	"R_KneePitch",
	"R_AnklePitch",
	"R_AnkleRoll"};

int main(int argc, char** argv) {
	


    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/thormang/joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    const double degree = M_PI/180;

    // robot state
    double tilt = 0, tinc = degree, swivel=0, angle=0, height=0, hinc=0.005;

    geometry_msgs::TransformStamped odom_trans;

    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "Pelvis";
    odom_trans.child_frame_id = "Waist";
	
    // message declarations

	joint_state.name.resize(35);
        joint_state.position.resize(35);
    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        odom_trans.header.stamp = ros::Time::now();

	for(int i=0;i<35;i++)
	{
        joint_state.name[i] =jtag[i];
	joint_state.position[i] = 0;
	}

	joint_state.position[0] = 1.57;
        joint_state.position[1] = 1.57;        
        joint_state.position[2] = 1.57;
	
      
        joint_pub.publish(joint_state);
       // broadcaster.sendTransform(odom_trans);
     
	ros::spinOnce();
        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}

