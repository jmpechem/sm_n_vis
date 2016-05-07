

#include <iostream>
#include <vector>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

#include "vrep_common/JointSetStateData.h"
#include "vrep_common/simRosGetObjectHandle.h"

#include "rt_dynamixel_msgs/JointSet.h"
#include "rt_dynamixel_msgs/JointState.h"

#include "smach_msgs/SmachContainerStatus.h"
#include "smach_msgs/SmachContainerInitialStatusCmd.h"

#include "data_bridge.h"

using namespace std;


string JointName[] = {"WaistPitch","WaistYaw",
                     "R_ShoulderPitch","R_ShoulderRoll","R_ShoulderYaw","R_ElbowRoll","R_WristYaw","R_WristRoll","R_HandYaw",
                     "L_ShoulderPitch","L_ShoulderRoll","L_ShoulderYaw","L_ElbowRoll","L_WristYaw","L_WristRoll","L_HandYaw",
                     "R_HipYaw","R_HipRoll","R_HipPitch","R_KneePitch","R_AnklePitch","R_AnkleRoll",
                     "L_HipYaw","L_HipRoll","L_HipPitch","L_KneePitch","L_AnklePitch","L_AnkleRoll"};


int JointID[28] = {1,};
int nJoints = 28;

class SMACHControl
{
private:
    ros::Subscriber smach_sub;
    ros::Publisher smach_pub;

public:
    SMACHControl()
    {  }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thormang_controller");
    ros::NodeHandle nh;

    dyros_robot_state* dataObjectPtr;
    // ros::Subscriber


    if(1)   // vrep mode TODO: replace this line to launch input
    {
        dataObjectPtr = new vrep_bridge(nh,nJoints,JointName,JointID);
    }
    else
    {
        dataObjectPtr = new real_bridge(nh,nJoints,JointName,JointID);
    }



    ros::Rate rate(400);
    while(ros::ok())
    {

        // Read(auto), Process

        // ex)
        // process(dataObjectPtr);


        // Write
        dataObjectPtr->setJoint();

        // Wait next loop
        ros::spinOnce();
        rate.sleep();
    }

    delete dataObjectPtr;
    return 0;
}
