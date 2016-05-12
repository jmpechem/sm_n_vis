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
#include "thormang_ctrl_msgs/JointSet.h"


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

Q_SIGNALS:
    void loggingUpdated();
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    ros::Publisher smach_publisher;
    ros::Publisher joint_ctrl_publisher;

    QStringListModel logging_model;
};

}  // namespace dyros_black_ui

#endif /* dyros_black_ui_QNODE_HPP_ */
