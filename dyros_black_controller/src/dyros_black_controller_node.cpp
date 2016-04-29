
#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

#include "vrep_common/JointSetStateData.h"
#include "vrep_common/simRosGetObjectHandle.h"

#include "rt_dynamixel_msgs/JointSet.h"
#include "rt_dynamixel_msgs/JointState.h"

using namespace std;

/*
virtual class dyros_robot_state
{

};
*/

class real_bridge
{
    ros::Publisher jointSetPub;

    ros::Subscriber jointSub;
    ros::Subscriber ftSub;
    ros::Subscriber gyroSub;
    int nDOF;

public:
    real_bridge(ros::NodeHandle &nh, int dof) : nDOF(dof)
    {
        jointSetPub = nh.advertise<rt_dynamixel_msgs::JointSet>("rt_dynamixel/joint_set", 1);
    }
};

class vrep_bridge
{
    // Initializer
    ros::ServiceClient vrepHandleClient;
    // Publisher
    ros::Publisher vrepJointSetPub;
    // Subscriber
    ros::Subscriber vrepJointSub;
    ros::Subscriber vrepLFTSub;
    ros::Subscriber vrepRFTSub;
    ros::Subscriber vrepGyroSub;
    // Joint Object (set)
    vrep_common::JointSetStateData jointSetObject;
    int nDOF; ///< a number of DOF

public:
    vrep_bridge(ros::NodeHandle &nh, int dof) : nDOF(dof)
    {
        vrepHandleClient = nh.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
        vrepJointSetPub = nh.advertise<vrep_common::JointSetStateData>("vrep/JointSet",1);
        // TODO: Write subscirber codes

        initialize_handler();
    }
    void joint_callback()
    {

    }
    void LFT_callback()
    {

    }
    void RLT_callback()
    {

    }

    void initialize_handler()
    {
        // Reset
        jointSetObject.handles.data.clear();
        jointSetObject.setModes.data.clear();
        jointSetObject.values.data.clear();

        string JointName[] = {"WaistPitch","WaistYaw",
                             "R_ShoulderPitch","R_ShoulderRoll","R_ShoulderYaw","R_ElbowRoll","R_WristYaw","R_WristRoll","R_HandYaw",
                             "L_ShoulderPitch","L_ShoulderRoll","L_ShoulderYaw","L_ElbowRoll","L_WristYaw","L_WristRoll","L_HandYaw",
                             "R_HipYaw","R_HipRoll","R_HipPitch","R_KneePitch","R_AnklePitch","R_AnkleRoll",
                             "L_HipYaw","L_HipRoll","L_HipPitch","L_KneePitch","L_AnklePitch","L_AnkleRoll"};


        vrep_common::simRosGetObjectHandle srv;

        jointSetObject.values.data.resize(nDOF);
        for (int i=0; i<nDOF; i++)
        {
            srv.request.objectName = JointName[i];
            if (vrepHandleClient.call(srv))
            {
                jointSetObject.handles.data.push_back(srv.response.handle);
                jointSetObject.setModes.data.push_back(1);
                jointSetObject.values.data[i] = 0.0;
            }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thormang_controller");
    ros::NodeHandle nh;

    // ros::Subscriber

    while(ros::ok())
    {

    }

    return 0;
}
