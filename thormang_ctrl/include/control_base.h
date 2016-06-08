#ifndef _CONTROL_BASE_H
#define _CONTROL_BASE_H

// STD Library
#include <vector>

// System Library
#include <termios.h>

// ROS Library
#include <ros/ros.h>

// ROS Messages
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include "smach_msgs/SmachContainerStatus.h"

#include "thormang_ctrl_msgs/JointSet.h"
#include "thormang_ctrl_msgs/JointState.h"
#include "thormang_ctrl_msgs/WalkingCmd.h"
#include "thormang_ctrl_msgs/RecogCmd.h"
#include "thormang_ctrl_msgs/TaskCmd.h"

// User Library
#include "Walking_Controller.h"
#include "Upperbody_Controller.h"

class controlBase
{

public:
    controlBase();
    virtual ~controlBase(){}
    // Default User Call function
    void parameter_initialize(); // initialize all parameter function(q,qdot,force else...)
    virtual void readdevice(); // read device means update all subscribed sensor data and user command
    virtual void update()=0; // update controller based on readdevice
    virtual void compute(); // compute algorithm and update all class object
    virtual void reflect()=0; // reflect next step actuation such as motor angle else
    virtual void writedevice()=0; // publish to actuate devices
    double Rounding( double x, int digit );

    int getch();

    bool check_state_changed();    
protected:
    ros::NodeHandle nh;

    ros::Subscriber walkingCmdSub;
    ros::Subscriber taskCmdSub;
    ros::Subscriber recogCmdSub;

    ros::Publisher smachPub;
    ros::Subscriber smachSub;

    ros::Publisher jointStateUIPub;
    ros::Subscriber jointCtrlSub;
    bool jointCtrlMsgRecv;


    thormang_ctrl_msgs::WalkingCmd walkingCmdMsg;
    thormang_ctrl_msgs::JointSet jointCtrlMsg;
    thormang_ctrl_msgs::TaskCmd  taskCmdMsg;

    string smach_state;
    string before_state;

    vector<int> jointID;
    vector<int> jointInvID;


    int uiUpdateCount;
    bool isFirstBoot;



    // Math
    int key_cmd;
    bool _Init_walking_flag;
    bool _Walking_flag;

    bool _Joint_flag;
    bool _CLIK_flag;


    VectorXD q; // current q
    VectorXD q_dot; // current qdot
    VectorXD torque; // current joint toruqe
    Vector6D LFT; // current left ft sensor values
    Vector6D RFT; // current right ft sensor values
    Vector3D Gyro; // current gyro sensor values
    VectorXD _desired_q; // current desired joint values
    int total_dof; //

    int         _index;
    VectorXD    _target_q;
    MatrixXD    _target_x;

    WalkingCtrl _WalkingCtrl;
    UpperCtrl   _UpperCtrl;

    void WalkingCheckState();
    void WalkingLoop();
    void UpperBodyCheckState();
    void UpperBodyLoop();

private:

    // Callback functions
    void SmachCallback(const smach_msgs::SmachContainerStatusConstPtr& smach);
    void UIJointCtrlCallback(const thormang_ctrl_msgs::JointSetConstPtr& joint);
    void WalkingCmdCallback(const thormang_ctrl_msgs::WalkingCmdConstPtr& msg);
    void TaskCmdCallback(const thormang_ctrl_msgs::TaskCmdConstPtr& msg);
    void RecogCmdCallback(const thormang_ctrl_msgs::RecogCmdConstPtr& msg);
};



#endif
