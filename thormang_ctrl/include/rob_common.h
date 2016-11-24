
#include "control_base.h"

#include <math.h>
#include <fstream>

#include "rt_dynamixel_msgs/JointState.h"
#include "rt_dynamixel_msgs/JointSet.h"

#include "rt_dynamixel_msgs/ModeSetting.h"
#include "rt_dynamixel_msgs/MotorSetting.h"

#include "imu_3dm_gx4/FilterOutput.h"

using namespace std;

class realrobot : public controlBase{
public:
   realrobot(ros::NodeHandle &nh); // constructor for initialize node
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

   ofstream gyroLogFileOut;
   ofstream Matrix_RPY_Out;
   realtime_tools::RealtimePublisher<rt_dynamixel_msgs::JointSet> dxlJointSetPub;

   ros::Subscriber dxlJointSub;

   ros::Subscriber imuSub;
   ros::Subscriber imuFilterSub;
   ros::Subscriber leftFootFTSub;
   ros::Subscriber rightFootFTSub;

   ros::Time befoTime;
   ros::Time nowTime;
   ros::Time dt;

   bool jointCtrlMsgRecv;

   // rt_dynamixel_msgs::JointSet jointSetMsg;
   // rt_dynamixel_msgs::JointStateConstPtr dxlJointStatePtr;

   /*
   sensor_msgs::ImuConstPtr imuMsgPtr;
   geometry_msgs::WrenchStampedConstPtr leftFootFTMsgPtr;
   geometry_msgs::WrenchStampedConstPtr rightFootFTMsgPtr;
   */
   // ros::Subscriber taskCtrlSub;

   bool _jointRecv;
   bool _imuRecv;
   bool _ftlfRecv;
   bool _ftrfRecv;


   int dxlMode; ///< current dynamixel mode
   int dxlTorque;

   ros::Rate rate;

   void jointCallback(const rt_dynamixel_msgs::JointStateConstPtr msg);
   void imuCallback(const sensor_msgs::ImuConstPtr msg);
   void imuFilterCallback(const imu_3dm_gx4::FilterOutputConstPtr msg);
   void AngleComplementaryFilter(double dt, double cutoff_freq, double pitch_i, double roll_i, double SensorData[6], double &pitch, double &roll);
   void Angle2Matrix(double roll, double pitch, Matrix3D& Rotation_Matrix);

   void leftFootFTCallback(const geometry_msgs::WrenchStampedConstPtr msg);
   void rightFootFTCallback(const geometry_msgs::WrenchStampedConstPtr msg);

 //  ros::Publisher vrepJointSetPub;
   // ros::Subscriber gyroSub;

   // ros::Subscriber ftSub;

};
