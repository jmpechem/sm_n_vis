/**
 * @file /include/dyros_black_ui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef dyros_black_ui_QNODE_HPP_
#define dyros_black_ui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <vector>

#include "thormang_ctrl_msgs/JointSet.h"
#include "thormang_ctrl_msgs/JointState.h"
#include "thormang_ctrl_msgs/RecogCmd.h"
#include "thormang_ctrl_msgs/TaskCmd.h"
#include "thormang_ctrl_msgs/WalkingCmd.h"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace dyros_black_ui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

    void send_transition(std::string str);
    void send_joint_ctrl(int id, double angle);
    void send_walking_cmd(thormang_ctrl_msgs::WalkingCmd& walking_msg);
    void send_recog_cmd(thormang_ctrl_msgs::RecogCmd& recog_msg);
    void send_task_cmd(thormang_ctrl_msgs::TaskCmd& task_msg);

    thormang_ctrl_msgs::JointState joint_msg;


Q_SIGNALS:

    void jointStateUpdated();
    void loggingUpdated();
    void rosShutdown();

private:
	int init_argc;
    char** init_argv;
    ros::Publisher smach_publisher;
    ros::Publisher joint_ctrl_publisher;

    ros::Publisher walking_cmd_publisher;
    ros::Publisher task_cmd_publisher;
    ros::Publisher recog_cmd_publisher;

    ros::Subscriber joint_state_subscirber;

    QStringListModel logging_model;

    bool isConnected;
    void jointStateCallback(const thormang_ctrl_msgs::JointStateConstPtr& msg);

};

}  // namespace dyros_black_ui

#endif /* dyros_black_ui_QNODE_HPP_ */
