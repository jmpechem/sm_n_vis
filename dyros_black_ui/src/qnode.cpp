/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/dyros_black_ui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace dyros_black_ui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
    init_argv(argv),
    isConnected(false)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"dyros_black_ui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
    // Add your ros communications here.
    smach_publisher = n.advertise<std_msgs::String>("/transition", 5);
    joint_ctrl_publisher = n.advertise<thormang_ctrl_msgs::JointSet>("/thormang_ctrl/joint_ctrl",5);
    task_cmd_publisher = n.advertise<thormang_ctrl_msgs::TaskCmd>("/thormang_ctrl/task_cmd",5);
    recog_cmd_publisher = n.advertise<thormang_ctrl_msgs::RecogCmd>("/thormang_ctrl/recog_cmd",5);
    walking_cmd_publisher = n.advertise<thormang_ctrl_msgs::WalkingCmd>("/thormang_ctrl/walking_cmd",5);

    ft_sensor_calib_publisher = n.advertise<std_msgs::Float32>("/ati_ft_sensor/calibration", 5);

    joint_state_subscirber = n.subscribe("/thormang_ctrl/joint_state",1,&QNode::jointStateCallback,this);
    recog_point_subscriber = n.subscribe("/custom_recog_point",1, &QNode::recogInfoCallback, this);

    isConnected = true;
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"dyros_black_ui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
    // Add your ros communications here.
    smach_publisher = n.advertise<std_msgs::String>("/transition", 5);
    joint_ctrl_publisher = n.advertise<thormang_ctrl_msgs::JointSet>("/thormang_ctrl/joint_ctrl",5);
    task_cmd_publisher = n.advertise<thormang_ctrl_msgs::TaskCmd>("/thormang_ctrl/task_cmd",5);
    recog_cmd_publisher = n.advertise<thormang_ctrl_msgs::RecogCmd>("/thormang_ctrl/recog_cmd",5);
    walking_cmd_publisher = n.advertise<thormang_ctrl_msgs::WalkingCmd>("/thormang_ctrl/walking_cmd",5);

    ft_sensor_calib_publisher = n.advertise<std_msgs::Float32>("/ati_ft_sensor/calibration", 5);

    joint_state_subscirber = n.subscribe("/thormang_ctrl/joint_state",1,&QNode::jointStateCallback,this);
    recog_point_subscriber = n.subscribe("/custom_recog_point",1, &QNode::recogInfoCallback, this);

    isConnected = true;
	start();
	return true;
}

void QNode::run() {
    ros::Rate loop_rate(100);
	while ( ros::ok() ) {


        ros::spinOnce();
        loop_rate.sleep();
	}
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::send_ft_calib(float time)
{
    if(isConnected)
    {
        std_msgs::Float32 msg;
        msg.data = time;

        ft_sensor_calib_publisher.publish(msg);
    }
}

void QNode::send_transition(std::string str)
{
    if(isConnected)
    {
        std_msgs::String msg;
        msg.data = str;
        smach_publisher.publish(msg);
    }
}
void QNode::send_joint_ctrl(int id, double angle)
{
    if(isConnected)
    {
        thormang_ctrl_msgs::JointSet msg;
        msg.angle = angle;
        msg.id = id;

        joint_ctrl_publisher.publish(msg);
    }
}


void QNode::send_walking_cmd(thormang_ctrl_msgs::WalkingCmd& walking_msg)
{
    walking_cmd_publisher.publish(walking_msg);
}

void QNode::send_recog_cmd(thormang_ctrl_msgs::RecogCmd& recog_msg)
{
    recog_cmd_publisher.publish(recog_msg);
}

void QNode::send_task_cmd(thormang_ctrl_msgs::TaskCmd& task_msg)
{    
    task_cmd_publisher.publish(task_msg);
}

void QNode::jointStateCallback(const thormang_ctrl_msgs::JointStateConstPtr &msg)
{
    joint_msg = *msg;
    jointStateUpdated();
}
void QNode::recogInfoCallback(const std_msgs::Float32MultiArrayConstPtr& msg)
{
    recog_info_msg = *msg;
    recogInfoUpdated();
}

void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace dyros_black_ui
