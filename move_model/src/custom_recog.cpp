#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/tf.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <termios.h>
#include <geometry_msgs/PointStamped.h>
#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>
bool key_evet = false;
int key_cmd[2] = {0,};
#define X_PLUS      1
#define X_MINUS     2
#define Y_PLUS      3
#define Y_MINUS     4
#define Z_PLUS      5
#define Z_MINUS     6
#define ROLL_PLUS   7
#define ROLL_MINUS  8
#define PITCH_PLUS  9
#define PITCH_MINUS 10
#define YAW_PLUS    11
#define YAW_MINUS   12
#define LINEAR      0.003
#define ORI         0.003
void key_cb(const std_msgs::Int32MultiArray::ConstPtr& key_input)
{
    int i=0;
    for(std::vector<int>::const_iterator it = key_input->data.begin(); it!= key_input->data.end();++it)
    {
        key_cmd[i] = *it;
        i++;
    }
    key_evet = true;
    return;
}
bool mouse_evet = false;
geometry_msgs::PointStamped click_marker;
void click_cb(const geometry_msgs::PointStamped::ConstPtr& mouse_pose)
{
    click_marker.header = mouse_pose->header;
    click_marker.point = mouse_pose->point;
    ROS_INFO("%f %f %f",click_marker.point.x,click_marker.point.y,click_marker.point.z);
    mouse_evet = true;
}
int main(int argc, char** argv)
{
    ros::init(argc,argv,"click_marker");
    ros::NodeHandle nh;

    ros::Subscriber keyboard_evet;
    keyboard_evet = nh.subscribe<std_msgs::Int32MultiArray>("key_cmd",1,key_cb);

    ros::Subscriber click_evet;
    click_evet = nh.subscribe<geometry_msgs::PointStamped>("/clicked_point",1,click_cb);
    ros::Publisher custom_recog_data;
    custom_recog_data = nh.advertise<std_msgs::Float32MultiArray>("custom_recog_point",1000);

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
    recog_marker.color.b = 0;
    recog_marker.color.g = 0;
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
    rot_control.interaction_mode =visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
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
       server.insert(custom_recog);
       ros::Rate rate(100);

    while(ros::ok())
    {

        server.get("custom_recog",custom_recog);       
        if(key_evet)
        {

            switch(key_cmd[0])
            {
                case X_PLUS:
                custom_recog.pose.position.x = custom_recog.pose.position.x + LINEAR;
                break;
                case X_MINUS:
                custom_recog.pose.position.x = custom_recog.pose.position.x - LINEAR;
                break;
                case Y_PLUS:
                custom_recog.pose.position.y = custom_recog.pose.position.y + LINEAR;
                break;
                case Y_MINUS:
                custom_recog.pose.position.y = custom_recog.pose.position.y - LINEAR;
                break;
                case Z_PLUS:
                custom_recog.pose.position.z = custom_recog.pose.position.z + LINEAR;
                break;
                case Z_MINUS:
                custom_recog.pose.position.z = custom_recog.pose.position.z - LINEAR;
                break;
                case ROLL_PLUS:
                custom_recog.pose.orientation.x = custom_recog.pose.orientation.x + ORI;
                break;
                case ROLL_MINUS:
                custom_recog.pose.orientation.x = custom_recog.pose.orientation.x - ORI;
                break;
                case PITCH_PLUS:
                custom_recog.pose.orientation.y = custom_recog.pose.orientation.y + ORI;
                break;
                case PITCH_MINUS:
                custom_recog.pose.orientation.y = custom_recog.pose.orientation.y - ORI;
                break;
                case YAW_PLUS:
                custom_recog.pose.orientation.z = custom_recog.pose.orientation.z + ORI;
                break;
                case YAW_MINUS:
                custom_recog.pose.orientation.z = custom_recog.pose.orientation.z - ORI;
                break;
            }
            custom_recog.pose.orientation.w = 1;
            server.setPose("custom_recog",custom_recog.pose);
            server.applyChanges();
            key_evet = false;
        }
        if(mouse_evet)
        {
            custom_recog.pose.position.x = click_marker.point.x;
            custom_recog.pose.position.y = click_marker.point.y;
            custom_recog.pose.position.z = click_marker.point.z;
            custom_recog.pose.orientation.x = 0;
            custom_recog.pose.orientation.y = 0;
            custom_recog.pose.orientation.z = 0;
            custom_recog.pose.orientation.w = 1;

            server.setPose("custom_recog",custom_recog.pose);
            server.applyChanges();
            mouse_evet = false;
        }
        tf::Quaternion q(custom_recog.pose.orientation.x,custom_recog.pose.orientation.y,custom_recog.pose.orientation.z,custom_recog.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double r,p,y;
        m.getRPY(r,p,y);
        custom_pose.data.push_back(custom_recog.pose.position.x);
        custom_pose.data.push_back(custom_recog.pose.position.y);
        custom_pose.data.push_back(custom_recog.pose.position.z);
        custom_pose.data.push_back(r);
        custom_pose.data.push_back(p);
        custom_pose.data.push_back(y);

        server.applyChanges();
        custom_recog_data.publish(custom_pose);
        custom_pose.data.clear();
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
