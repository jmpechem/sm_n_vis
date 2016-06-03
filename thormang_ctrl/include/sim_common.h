#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <iostream>
#include <string.h>

// Used data structures:
#include "vrep_common/ProximitySensorData.h"
#include "vrep_common/VrepInfo.h"
#include "vrep_common/JointSetStateData.h"
#include "vrep_common/ForceSensorData.h"
#include "vrep_common/simRosGetObjectHandle.h"
#include "vrep_common/simRosStartSimulation.h"
#include "vrep_common/simRosStopSimulation.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <smach_msgs/SmachContainerStatus.h>
// Used API services:
#include "vrep_common/simRosEnablePublisher.h"
#include "vrep_common/simRosEnableSubscriber.h"
#include "vrep_common/simRosSynchronous.h"
#include "vrep_common/simRosSynchronousTrigger.h"
//math library
#include "Walking_Controller.h"
#include "Upperbody_Controller.h"

#include <termios.h>
#include <iostream>
#include <fstream>

#define TotalJointNumber 28


class simulation{
 public:   
   simulation(); // constructor for initialize node
   void infoCallback(const vrep_common::VrepInfo::ConstPtr& info); // vrep information(ex. simulation update time) callback
   void JointCallback(const sensor_msgs::JointState::ConstPtr& joint); // vrep current joint value callback
   void LftCallback(const vrep_common::ForceSensorData::ConstPtr& Lft); // vrep current left ft sensor value callback
   void RftCallback(const vrep_common::ForceSensorData::ConstPtr& Rft); // vrep current left ft sensor value callback
   void SmachCallback(const smach_msgs::SmachContainerStatusConstPtr &smach);
//   vrep_common::JointSetStateData Initialize_handler(ros::ServiceClient vrepHandleClient);
   void vrep_start();
   void vrep_initialize();
   void parameter_initialize(); // initialize all parameter function(q,qdot,force else...)
   void readdevice(); // read device means update all subscribed sensor data and user command
   void update(); // update controller based on readdevice
   void compute(); // compute algorithm and update all class object
   void reflect(); // reflect next step actuation such as motor angle else
   void writedevice(); // publish to actuate devices
   int getch(); // keyboard command input(will removed in future)
   void vrep_end();
   void vrep_stop();

   bool check_state_changed();

 private:
   ros::NodeHandle nh; // node handle for simlation Class  
   vrep_common::JointSetStateData JointSetValue; // Vrep desired joint value(for sending to vrep simulator)
   vrep_common::simRosSynchronous srv_startsync; // Vrep Time synchonization service
   ros::ServiceClient client_startsync; // Vrep synchronize start service publisher
   ros::ServiceClient client_startTrig; // Vrep simulation trigger start service

   ros::ServiceClient start_simulation;
   ros::ServiceClient end_simulation;
   ros::Publisher vrepJointSetPub;
   ros::Publisher smachPub;
   ros::Subscriber smachSub;
   string smach_state;
   string before_state;
   bool simulationRunning;
   float simulationTime; // current simulation time
   ros::ServiceClient vrepHandleClient;
   vrep_common::simRosSynchronousTrigger srv_startTrig; // Vrep service start trigger

   VectorXD q; // current q from vrep
   VectorXD q_dot; // current qdot from vrep
   VectorXD torque; // current joint toruqe from vrep
   Vector6D LFT; // current left ft sensor values from vrep
   Vector6D RFT; // current right ft sensor values from vrep 
   Vector3D Gyro; // current gyro sensor values from vrep
   VectorXD _desired_q; // current desired joint values from vrep
   int total_dof; // 

 //  ros::Publisher vrepJointSetPub;
   ros::Subscriber subInfo;
   ros::Subscriber Jointsub;
   ros::Subscriber Lftsub;
   ros::Subscriber Rftsub;

   int key_cmd;
   bool _Init_walking_flag;
   bool _Walking_flag;

   bool _Joint_flag;
   bool _CLIK_flag;


 public:
   WalkingCtrl _WalkingCtrl;
   UpperCtrl   _UpperCtrl;

   int 	       _index;
   VectorXD    _target_q;
   MatrixXD    _target_x;
};
