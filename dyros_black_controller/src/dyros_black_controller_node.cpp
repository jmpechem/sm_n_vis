

#include <iostream>
#include <vector>

#include <ros/ros.h>

#include "data_bridge.h"
#include "smach_control.h"


using namespace std;


string JointName[] = {"WaistPitch","WaistYaw",
                     "R_ShoulderPitch","R_ShoulderRoll","R_ShoulderYaw","R_ElbowRoll","R_WristYaw","R_WristRoll","R_HandYaw",
                     "L_ShoulderPitch","L_ShoulderRoll","L_ShoulderYaw","L_ElbowRoll","L_WristYaw","L_WristRoll","L_HandYaw",
                     "R_HipYaw","R_HipRoll","R_HipPitch","R_KneePitch","R_AnklePitch","R_AnkleRoll",
                     "L_HipYaw","L_HipRoll","L_HipPitch","L_KneePitch","L_AnklePitch","L_AnkleRoll"};


int JointID[28] = {1,};
int nJoints = 28;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "thormang_controller");
    ros::NodeHandle nh;

    dyros_robot_state* dataObjectPtr;
    SMACHControl smach(nh);
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

        if(smach.getState() == "Joint")
        {

        }

        // Write
        dataObjectPtr->setJoint();

        // Wait next loop
        ros::spinOnce();
        rate.sleep();
    }

    delete dataObjectPtr;
    return 0;
}
