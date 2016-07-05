
#include "control_base.h"


#include "rt_dynamixel_msgs/JointState.h"
#include "rt_dynamixel_msgs/JointSet.h"

#include "rt_dynamixel_msgs/ModeSetting.h"
#include "rt_dynamixel_msgs/MotorSetting.h"

using namespace std;

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
   void wait();


private:
   ros::ServiceClient dxlModeSetClient; ///< dynamixel mode select service
   ros::ServiceClient dxlMotorSetClient; ///< dynmamixel motor setting service

   rosrt::Publisher<rt_dynamixel_msgs::JointSet> dxlJointSetPub;
   rosrt::Subscriber<rt_dynamixel_msgs::JointState> dxlJointSub;

   rosrt::Subscriber<sensor_msgs::Imu> imuSub;
   rosrt::Subscriber<geometry_msgs::WrenchStamped> leftFootFTSub;
   rosrt::Subscriber<geometry_msgs::WrenchStamped> rightFootFTSub;

   bool jointCtrlMsgRecv;

   // rt_dynamixel_msgs::JointSet jointSetMsg;
   rt_dynamixel_msgs::JointSetPtr dxlJointSetMsgPtr;
   rt_dynamixel_msgs::JointStateConstPtr dxlJointStatePtr;

   sensor_msgs::ImuConstPtr imuMsgPtr;
   geometry_msgs::WrenchStampedConstPtr leftFootFTMsgPtr;
   geometry_msgs::WrenchStampedConstPtr rightFootFTMsgPtr;
   // ros::Subscriber taskCtrlSub;

   int dxlMode; ///< current dynamixel mode
   int dxlTorque;

   ros::Rate rate;


 //  ros::Publisher vrepJointSetPub;
   // ros::Subscriber gyroSub;

   // ros::Subscriber ftSub;

};
