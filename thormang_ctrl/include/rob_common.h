
#include "control_base.h"

#include "rt_dynamixel_msgs/JointState.h"
#include "rt_dynamixel_msgs/JointSet.h"

#include "rt_dynamixel_msgs/ModeSetting.h"
#include "rt_dynamixel_msgs/MotorSetting.h"


#define TotalJointNumber 28


class realrobot : public controlBase{
public:
   realrobot(); // constructor for initialize node
   //void ftCallback(const vrep_common::ForceSensorData::ConstPtr& Lft); // current left ft sensor value callback

   void change_dxl_mode(int mode);
   void set_torque(int value);
   void set_aim_position(int id, double radian);
   //void make_id_inverse_list();

   void update(); // update controller based on readdevice
   void reflect(); // reflect next step actuation such as motor angle else
   void writedevice(); // publish to actuate devices


private:
   void JointCallback(const rt_dynamixel_msgs::JointStateConstPtr& joint);

private:
   ros::ServiceClient dxlModeSetClient; ///< dynamixel mode select service
   ros::ServiceClient dxlMotorSetClient; ///< dynmamixel motor setting service

   ros::Publisher dxlJointSetPub;
   ros::Subscriber dxlJointSub;

   bool jointCtrlMsgRecv;

   rt_dynamixel_msgs::JointSet jointSetMsg;

   // ros::Subscriber taskCtrlSub;

   int dxlMode; ///< current dynamixel mode
   int dxlTorque;


 //  ros::Publisher vrepJointSetPub;
   // ros::Subscriber gyroSub;

   // ros::Subscriber ftSub;

};
