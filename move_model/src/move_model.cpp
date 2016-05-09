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
geometry_msgs::PointStamped clicked_point;
bool clicked = false;
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
/*void click_cb(const geometry_msgs::PointStamped::ConstPtr& points)
{
	clicked_point.header = points->header;
	clicked_point.point = points->point;
	clicked = true;
	ROS_INFO("%f %f %f",clicked_point.point.x,clicked_point.point.y,clicked_point.point.z);
}*/
int main(int argc, char** argv) {
	
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/thormang/joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);
/////////////////////////////
/*    ros::Subscriber clicked_sub;
    	clicked_sub = n.subscribe<geometry_msgs::PointStamped>("/clicked_point",1,click_cb);

	std_msgs::Float32MultiArray custom_pose;
	visualization_msgs::InteractiveMarker custom_recog;
	custom_recog.header.frame_id ="Pelvis";
	custom_recog.name = "custom_recog";
	custom_recog.description = "recog_marker";
	custom_recog.scale = 0.3;

	visualization_msgs::Marker recog_marker;
	recog_marker.type = visualization_msgs::Marker::SPHERE;
	recog_marker.scale.x = 0.02;
	recog_marker.scale.y = 0.02;
	recog_marker.scale.z = 0.02;
	recog_marker.color.r = 1;
	recog_marker.color.g = 0;
	recog_marker.color.b = 0;
	recog_marker.color.a = 1;

	visualization_msgs::InteractiveMarkerControl custom_recog_control;
	custom_recog_control.always_visible = true;
	custom_recog_control.markers.push_back(recog_marker);
	custom_recog.controls.push_back(custom_recog_control);

	visualization_msgs::InteractiveMarkerControl rot_control;

	rot_control.orientation.w = 1;
	rot_control.orientation.x = 1;
	rot_control.orientation.y = 0;
	rot_control.orientation.z = 0;
	rot_control.name = "rotate_x";
	rot_control.interaction_mode=visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	custom_recog.controls.push_back(rot_control);
	rot_control.name = "move_x";
	rot_control.interaction_mode =visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	custom_recog.controls.push_back(rot_control);

	rot_control.orientation.w = 1;
	rot_control.orientation.x = 0;
	rot_control.orientation.y = 1;
	rot_control.orientation.z = 0;
	rot_control.name = "rotate_z";
	rot_control.interaction_mode =visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	custom_recog.controls.push_back(rot_control);

	rot_control.name = "move_z";
	rot_control.interaction_mode =visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	custom_recog.controls.push_back(rot_control);

	rot_control.orientation.w = 1;
	rot_control.orientation.x = 0;
	rot_control.orientation.y = 0;
	rot_control.orientation.z = 1;
	rot_control.name = "rotate_y";
	rot_control.interaction_mode =visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	custom_recog.controls.push_back(rot_control);


	rot_control.name = "move_y";
	rot_control.interaction_mode =visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	custom_recog.controls.push_back(rot_control);
	interactive_markers::InteractiveMarkerServer server("recog_server");
	server.insert(custom_recog);*/
////////////////////////////
	sensor_msgs::JointState joint_state;
	joint_state.name.resize(35);
        joint_state.position.resize(35);

    while (ros::ok()) {
//////////////////////////
//	server.get("custom_recog",custom_recog);       
/////////////////////
        //update joint_state
        joint_state.header.stamp = ros::Time::now();

	for(int i=0;i<35;i++)
	{
        joint_state.name[i] =jtag[i];
	joint_state.position[i] = 0;
	}
      
        joint_pub.publish(joint_state);
    
	ros::spinOnce();
        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}

