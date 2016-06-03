#include <ros/ros.h>
#include <iostream>
#include <string>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

#include "rt_dynamixel_msgs/JointState.h"
#include "rt_dynamixel_msgs/JointSet.h"

#include "rt_dynamixel_msgs/ModeSetting.h"
#include "rt_dynamixel_msgs/MotorSetting.h"

#include "smach_msgs/SmachContainerStatus.h"

#include "thormang_ctrl_msgs/JointSet.h"
#include "thormang_ctrl_msgs/JointState.h"
#include "thormang_ctrl_msgs/WalkingCmd.h"
#include "thormang_ctrl_msgs/RecogCmd.h"
#include "thormang_ctrl_msgs/TaskCmd.h"

//math library
#include "Walking_Controller.h"

#include <vector>

#include <termios.h>
#include <iostream>
#include <fstream>


#define TotalJointNumber 28


class realrobot{
 public:
   realrobot(); // constructor for initialize node

   // --------- CALLBACK
   void JointCallback(const rt_dynamixel_msgs::JointStateConstPtr& joint); // current joint value callback
   void SmachCallback(const smach_msgs::SmachContainerStatusConstPtr& smach);
   void UIJointCtrlCallback(const thormang_ctrl_msgs::JointSetConstPtr& joint);
   void WalkingCmdCallback(const thormang_ctrl_msgs::WalkingCmdConstPtr& msg);
   void TaskCmdCallback(const thormang_ctrl_msgs::TaskCmdConstPtr& msg);
   void RecogCmdCallback(const thormang_ctrl_msgs::RecogCmdConstPtr& msg);
   //void ftCallback(const vrep_common::ForceSensorData::ConstPtr& Lft); // current left ft sensor value callback

   void change_dxl_mode(int mode);
   void set_torque(int value);
   void set_aim_position(int id, double radian);
   void make_id_inverse_list();

   void parameter_initialize(); // initialize all parameter function(q,qdot,force else...)
   void readdevice(); // read device means update all subscribed sensor data and user command
   void update(); // update controller based on readdevice
   void compute(); // compute algorithm and update all class object
   void reflect(); // reflect next step actuation such as motor angle else
   void writedevice(); // publish to actuate devices

   int getch();

 private:
   ros::NodeHandle nh; ///< node handle for real robot Class

   ros::ServiceClient dxlModeSetClient; ///< dynamixel mode select service
   ros::ServiceClient dxlMotorSetClient; ///< dynmamixel motor setting service

   ros::Publisher dxlJointSetPub;
   ros::Subscriber dxlJointSub;

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
   rt_dynamixel_msgs::JointSet jointSetMsg;

   // ros::Subscriber taskCtrlSub;

   int dxlMode; ///< current dynamixel mode
   int dxlTorque;

   string smach_state;

   vector<int> jointID;
   vector<int> jointInvID;


   int uiUpdateCount;

   int key_cmd;
   bool _Init_walking_flag;
   bool _Walking_flag;

   bool isFirstBoot;

   VectorXD q; // current q
   VectorXD q_dot; // current qdot
   VectorXD torque; // current joint toruqe
   Vector6D LFT; // current left ft sensor values
   Vector6D RFT; // current right ft sensor values
   Vector3D Gyro; // current gyro sensor values
   VectorXD _desired_q; // current desired joint values
   int total_dof; //

 //  ros::Publisher vrepJointSetPub;
   // ros::Subscriber gyroSub;

   // ros::Subscriber ftSub;

 public:
   WalkingCtrl _WalkingCtrl;

};
