#ifndef _CONTROL_BASE_H
#define _CONTROL_BASE_H

// STD Library
#include <vector>

// System Library
#include <termios.h>

// ROS Library
#include <ros/ros.h>
#include <rosrt/rosrt.h>

// ROS Messages
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
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

extern const string JointName[40];
extern const int jointIDs[40];

enum body_select {UPPER, WALKING, HEAD};

class controlBase
{

public:
    controlBase();
    virtual ~controlBase(){}
    // Default User Call function
    void parameter_initialize(); // initialize all parameter function(q,qdot,force else...)
    virtual void readdevice(); // read device means update all subscribed sensor data and user command
    virtual void update(); // update controller based on readdevice
    virtual void compute(); // compute algorithm and update all class object
    virtual void reflect(); // reflect next step actuation such as motor angle else
    virtual void writedevice()=0; // publish to actuate devices
    virtual void wait()=0;  // wait
    double Rounding( double x, int digit );
    int getch();

    bool check_state_changed();
protected:
    ros::NodeHandle nh;

    rosrt::Subscriber<thormang_ctrl_msgs::WalkingCmd> walkingCmdSub;
    rosrt::Subscriber<thormang_ctrl_msgs::TaskCmd> taskCmdSub;
    rosrt::Subscriber<thormang_ctrl_msgs::RecogCmd> recogCmdSub;

    rosrt::Publisher<std_msgs::String> smachPub;
    rosrt::Subscriber<smach_msgs::SmachContainerStatus> smachSub;

    rosrt::Publisher<thormang_ctrl_msgs::JointState> jointStateUIPub;
    rosrt::Subscriber<thormang_ctrl_msgs::JointSet> jointCtrlSub;
    rosrt::Subscriber<std_msgs::Float32MultiArray> recogPointSub;

    // rosrt implement
    // Publisher Message Preallocator
    thormang_ctrl_msgs::JointStatePtr jointStateMsgPtr;
    std_msgs::StringPtr smachMsgPtr;

    // Subscriber Pointers
    thormang_ctrl_msgs::WalkingCmdConstPtr walkingMsgPtr;
    thormang_ctrl_msgs::TaskCmdConstPtr taskMsgPtr;
    thormang_ctrl_msgs::RecogCmdConstPtr recogMsgPtr;
    thormang_ctrl_msgs::JointSetConstPtr jointSetMsgPtr;
    smach_msgs::SmachContainerStatusConstPtr smachStatusMsgPtr;
    std_msgs::Float32MultiArrayConstPtr recogPointPtr;
    /*
    ros::Subscriber walkingCmdSub;
    ros::Subscriber taskCmdSub;
    ros::Subscriber recogCmdSub;

    ros::Publisher smachPub;
    ros::Subscriber smachSub;

    ros::Publisher jointStateUIPub;
    ros::Subscriber jointCtrlSub;
    */
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



    float recogPoint[6];


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
    int         _cnt;
    VectorXD    _target_q;
    MatrixXD    _target_x;

    WalkingCtrl _WalkingCtrl;
    UpperCtrl   _UpperCtrl;

    void WalkingCheckState();
    void WalkingLoop();
    void UpperBodyCheckState();
    void UpperBodyLoop();

    void updateDesired(body_select body, VectorXD &update_q);


private:

    void make_id_inverse_list();
    /*
    // Callback functions
    void SmachCallback(const smach_msgs::SmachContainerStatusConstPtr& smach);
    void UIJointCtrlCallback(const thormang_ctrl_msgs::JointSetConstPtr& joint);
    void WalkingCmdCallback(const thormang_ctrl_msgs::WalkingCmdConstPtr& msg);
    void TaskCmdCallback(const thormang_ctrl_msgs::TaskCmdConstPtr& msg);
    void RecogCmdCallback(const thormang_ctrl_msgs::RecogCmdConstPtr& msg);
    */
};



#endif
