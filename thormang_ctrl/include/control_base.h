#ifndef _CONTROL_BASE_H
#define _CONTROL_BASE_H

// STD Library
#include <vector>

// System Library
#include <termios.h>

// ROS Library
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/transform_datatypes.h>

// ROS Messages
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>
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

enum body_select {UPPER,WAIST, WALKING, HEAD};

class controlBase
{

public:
    controlBase(ros::NodeHandle &nh);
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
    ros::Subscriber walkingCmdSub;
    ros::Subscriber taskCmdSub;
    ros::Subscriber recogCmdSub;

    realtime_tools::RealtimePublisher<std_msgs::String> smachPub;
    ros::Subscriber smachSub;

    realtime_tools::RealtimePublisher<thormang_ctrl_msgs::JointState> jointStateUIPub;
    ros::Subscriber jointCtrlSub;
    ros::Subscriber recogPointSub;

    // rosrt implement
    // Publisher Message Preallocator

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

    bool _Init_Egress_flag;
    bool _Egress_flag;

    bool _Waist_flag;
    bool _Joint_flag;
    bool _CLIK_flag;




    VectorXD q; // current q
    VectorXD q_dot; // current qdot
    VectorXD torque; // current joint toruqe
    Vector6D leftFootFT; // current left ft sensor values
    Vector6D rightFootFT; // current right ft sensor values
    Vector3D gyro; // current gyro sensor values
    Vector3D accelometer; // current accelometer values
    VectorXD _desired_q; // current desired joint values
    VectorXD _walking_q; // temporary walking q values
    VectorXD _upper_output_q; // upper output q values
    VectorXD _walking_output_q; // walking output q values
    Matrix3D Gyro_Matrix;


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
    void WholebodyLoop();

    void updateDesired(body_select body, VectorXD &update_q);


private:

    void make_id_inverse_list();

    // Callback functions
    void SmachCallback(const smach_msgs::SmachContainerStatusConstPtr& smach);
    void UIJointCtrlCallback(const thormang_ctrl_msgs::JointSetConstPtr& joint);
    void WalkingCmdCallback(const thormang_ctrl_msgs::WalkingCmdConstPtr& msg);
    void TaskCmdCallback(const thormang_ctrl_msgs::TaskCmdConstPtr& msg);
    void RecogCmdCallback(const thormang_ctrl_msgs::RecogCmdConstPtr& msg);
    void RecogPointCallback(const std_msgs::Float32MultiArrayConstPtr& msg);

};



#endif
