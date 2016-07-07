
#include "control_base.h"


#include "rt_dynamixel_msgs/JointState.h"
#include "rt_dynamixel_msgs/JointSet.h"

#include "rt_dynamixel_msgs/ModeSetting.h"
#include "rt_dynamixel_msgs/MotorSetting.h"

#include "imu_3dm_gx4/FilterOutput.h"

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
   /*
   rosrt::Subscriber<rt_dynamixel_msgs::JointState> dxlJointSub;

   rosrt::Subscriber<sensor_msgs::Imu> imuSub;
   rosrt::Subscriber<geometry_msgs::WrenchStamped> leftFootFTSub;
   rosrt::Subscriber<geometry_msgs::WrenchStamped> rightFootFTSub;
*/
   ros::Subscriber dxlJointSub;

   ros::Subscriber imuSub;
   ros::Subscriber imuFilterSub;
   ros::Subscriber leftFootFTSub;
   ros::Subscriber rightFootFTSub;

   bool jointCtrlMsgRecv;

   // rt_dynamixel_msgs::JointSet jointSetMsg;
   rt_dynamixel_msgs::JointSetPtr dxlJointSetMsgPtr;
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

   void jointCallback(const rt_dynamixel_msgs::JointStateConstPtr msg)
   {

       for(int i=0; i<total_dof; i++)
       {
           for (int j=0; j<msg->id.size(); j++)
           {
               if(jointID[i] == msg->id[j])
               {
                   q(i) = msg->angle[j];
                   if(isFirstBoot)
                   {    _desired_q(i) = msg->angle[j]; }

                   q_dot(i) = msg->velocity[j];
                   torque(i) = msg->current[j];
                   jointStateMsgPtr->error[i] = msg->updated[j];
               }
           }
       }
       if(isFirstBoot)
       {isFirstBoot = false;}
       _jointRecv = true;
   }
   void imuCallback(const sensor_msgs::ImuConstPtr msg)
   {

       double rcvDatas[6];
       rcvDatas[0] = msg->linear_acceleration.x;
       rcvDatas[1] = msg->linear_acceleration.y;
       rcvDatas[2] = msg->linear_acceleration.z;

       rcvDatas[3] = msg->angular_velocity.x;
       rcvDatas[4] = msg->angular_velocity.y;
       rcvDatas[5] = msg->angular_velocity.z;

       AngleComplementaryFilter(0.004, 0.1, gyro[1], gyro[0], rcvDatas, gyro[1], gyro[0]);
       //ROS_INFO("r=%.2lf p=%.2lf",gyro[0]* 57.295791433, gyro[1]* 57.295791433);
       // ROS_INFO("roll = %.2lf, pitch = %.2lf", gyro[0] * 57.295791433, gyro[1] * 57.295791433);
       /*
       gyro[0] = msg->angular_velocity.x;
       gyro[1] = msg->angular_velocity.y;
       gyro[2] = msg->angular_velocity.z;

       accelometer[0] = msg->linear_acceleration.x;
       accelometer[1] = msg->linear_acceleration.y;
       accelometer[2] = msg->linear_acceleration.z;
       */
       _imuRecv = true;
   }

   void imuFilterCallback(const imu_3dm_gx4::FilterOutputConstPtr msg)
   {

       tf::Quaternion q(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
       tf::Matrix3x3 m(q);

       // m.getRPY(gyro[0],gyro[1],gyro[2]);

       // ROS_INFO("r=%.2lf p=%.2lf y=%.2lf",gyro[0]* 57.295791433, gyro[1]* 57.295791433, gyro[2]* 57.295791433);
   }

   void AngleComplementaryFilter(double dt, double cutoff_freq, double pitch_i, double roll_i, double SensorData[6], double &pitch, double &roll)
   {
    //sensor data = acc x, y, z, ang rate pitch, roll, yaw
    //double cutoff_freq = 0.1; //recommand value:0.1~0.2
    float wn = 2.0*3.1415*cutoff_freq;
    float alpha_const = (1.0/wn/dt)/(1.0+1.0/wn/dt);

    float pitch_Acc, roll_Acc;

    pitch_Acc = (atan2(SensorData[0],SensorData[2]));
    pitch = (alpha_const*(pitch_i-SensorData[4]*dt) + (1.0-alpha_const)*pitch_Acc);

    roll_Acc = -(atan2(SensorData[1],SensorData[2]));
    roll = (alpha_const*(roll_i-SensorData[3]*dt) + (1.0-alpha_const)*roll_Acc);
   }




   void leftFootFTCallback(const geometry_msgs::WrenchStampedConstPtr msg)
   {
       leftFootFT[0] = msg->wrench.force.x;
       leftFootFT[1] = msg->wrench.force.y;
       leftFootFT[2] = msg->wrench.force.z;
       leftFootFT[3] = msg->wrench.torque.x;
       leftFootFT[4] = msg->wrench.torque.y;
       leftFootFT[5] = msg->wrench.torque.z;
       _ftlfRecv = true;


   }
   void rightFootFTCallback(const geometry_msgs::WrenchStampedConstPtr msg)
   {
       rightFootFT[0] = msg->wrench.force.x;
       rightFootFT[1] = msg->wrench.force.y;
       rightFootFT[2] = msg->wrench.force.z;
       rightFootFT[3] = msg->wrench.torque.x;
       rightFootFT[4] = msg->wrench.torque.y;
       rightFootFT[5] = msg->wrench.torque.z;
       _ftrfRecv = true;
   }

 //  ros::Publisher vrepJointSetPub;
   // ros::Subscriber gyroSub;

   // ros::Subscriber ftSub;

};
