
#include "control_base.h"

// Used data structures:
#include "vrep_common/ProximitySensorData.h"
#include "vrep_common/VrepInfo.h"
#include "vrep_common/JointSetStateData.h"
#include "vrep_common/ForceSensorData.h"
#include "vrep_common/simRosGetObjectHandle.h"
#include "vrep_common/simRosStartSimulation.h"
#include "vrep_common/simRosStopSimulation.h"
// Used API services:
#include "vrep_common/simRosEnablePublisher.h"
#include "vrep_common/simRosEnableSubscriber.h"
#include "vrep_common/simRosSynchronous.h"
#include "vrep_common/simRosSynchronousTrigger.h"


class simulation : public controlBase{
 public:   
   simulation(ros::NodeHandle &nh); // constructor for initialize node
   virtual ~simulation() { vrep_end(); }
   void infoCallback(const vrep_common::VrepInfo::ConstPtr& info); // vrep information(ex. simulation update time) callback
   void JointCallback(const sensor_msgs::JointState::ConstPtr& joint); // vrep current joint value callback
   void LftCallback(const vrep_common::ForceSensorData::ConstPtr& Lft); // vrep current left ft sensor value callback
   void RftCallback(const vrep_common::ForceSensorData::ConstPtr& Rft); // vrep current left ft sensor value callback
//   vrep_common::JointSetStateData Initialize_handler(ros::ServiceClient vrepHandleClient);
   void vrep_start();
   void vrep_initialize();
   void vrep_end();
   void vrep_stop();

   void update(); // update controller based on readdevice
   void compute(); // compute algorithm and update all class object
   void writedevice(); // publish to actuate devices
   void wait();


 private:

   vrep_common::JointSetStateData JointSetValue; // Vrep desired joint value(for sending to vrep simulator)
   vrep_common::simRosSynchronous srv_startsync; // Vrep Time synchonization service
   ros::ServiceClient client_startsync; // Vrep synchronize start service publisher
   ros::ServiceClient client_startTrig; // Vrep simulation trigger start service

   ros::ServiceClient start_simulation;
   ros::ServiceClient end_simulation;
   ros::Publisher vrepJointSetPub;

   bool simulationRunning;
   float simulationTime; // current simulation time
   ros::ServiceClient vrepHandleClient;
   vrep_common::simRosSynchronousTrigger srv_startTrig; // Vrep service start trigger

   ros::Rate rate;

 //  ros::Publisher vrepJointSetPub;
   ros::Subscriber subInfo;
   ros::Subscriber Jointsub;
   ros::Subscriber Lftsub;
   ros::Subscriber Rftsub;


};
