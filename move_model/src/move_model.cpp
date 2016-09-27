#include <string>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float32MultiArray.h>
#include "thormang_ctrl_msgs/JointState.h"


geometry_msgs::PointStamped clicked_point;
bool clicked = false;
/*std::string jtag[35]
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
    "R_AnkleRoll"};*/
std::string jtag[35]
={  "R_ShoulderPitch",
    "L_ShoulderPitch",
    "R_ShoulderRoll",
    "L_ShoulderRoll",
    "R_ShoulderYaw",
    "L_ShoulderYaw",
    "R_ElbowRoll",
    "L_ElbowRoll",
    "R_WristYaw",
    "L_WristYaw",
    "R_WristRoll",
    "L_WristRoll",
    "R_HandYaw",
    "L_HandYaw",
    "R_HipYaw",
    "L_HipYaw",
    "R_HipRoll",
    "L_HipRoll",
    "R_HipPitch",
    "L_HipPitch",
    "R_KneePitch",
    "L_KneePitch",
    "R_AnklePitch",
    "L_AnklePitch",
    "R_AnkleRoll",
    "L_AnkleRoll",
    "WaistYaw",
    "WaistPitch",
    "HeadYaw",
    "HeadPitch", // this is 32 dof real robot state
    "R_Gripper_1",
    "R_Gripper_2",
    "L_Gripper_1",
    "L_Gripper_2",
    "ChestLidar"
    };

double joint_angles[50] = {0.0f,};

void Joint_cb(const thormang_ctrl_msgs::JointState::ConstPtr& joints)
{
    for(unsigned int i=0;i<joints->id.size();i++)
    {

          joint_angles[joints->id[i] - 1] = joints->angle[i];
    }




}
int main(int argc, char** argv) {
	
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/thormang/joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);
    ros::Subscriber joint_sub = n.subscribe("/thormang_ctrl/joint_state",1,&Joint_cb);
	sensor_msgs::JointState joint_state;
	joint_state.name.resize(35);
        joint_state.position.resize(35);

    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();

	for(int i=0;i<35;i++)
	{
        joint_state.name[i] =jtag[i];
        joint_state.position[i] = joint_angles[i];
	}
      
        joint_pub.publish(joint_state);
    
	ros::spinOnce();
        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}

