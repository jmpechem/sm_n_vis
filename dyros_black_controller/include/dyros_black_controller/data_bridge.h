
#include <vector>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

#include "vrep_common/JointSetStateData.h"
#include "vrep_common/simRosGetObjectHandle.h"

#include "rt_dynamixel_msgs/JointSet.h"
#include "rt_dynamixel_msgs/JointState.h"


using namespace std;

struct motor_state
{
    int id;
    string name;
    double position;
    double velocity;
    double effort;

    double desired_position;
};
struct ft_state
{
    double force_x;
    double force_y;
    double force_z;

    double torque_x;
    double torque_y;
    double torque_z;
};

struct gyro_state
{
    double roll;
    double pitch;
    double yaw;

    double gyro_x;
    double gyro_y;
    double gyro_z;

    double acc_x;
    double acc_y;
    double acc_z;

    double temperature;
};

class dyros_robot_state
{
public:
    dyros_robot_state(int joints, string *jointName, int *jointID) : nJoints(joints)
    {
        for(int i=0; i<joints; i++)
        {
            joint[i].name = jointName[i];
            joint[i].id = jointID[i];
        }
    }

    // Mutex need?
    virtual ~dyros_robot_state() { }
    virtual void setJoint() = 0; // Set to joint[i].desired_position
    void syncJoint()    // desired = now
    {
        for (int i=0; i<nJoints; i++)
        {
            joint[i].desired_position = joint[i].position;
        }
    }


    motor_state joint[50]; ///< joint datas \n Joint datas are stored along input joint name
    int nJoints; ///< the number of joints

    ft_state ft_sensor[2];  ///< ft sensor datas
    gyro_state gyro; ///< gyro datas
};


class real_bridge : public dyros_robot_state
{
    ros::Publisher jointSetPub;

    ros::Subscriber jointSub;
    ros::Subscriber ftSub;
    ros::Subscriber gyroSub;

    rt_dynamixel_msgs::JointSet jointMsg;
    vector<int> id2index;

public:
    real_bridge(ros::NodeHandle &nh, int joints, string *jointName, int *jointID) : dyros_robot_state(joints,jointName,jointID)
    {
        jointSetPub = nh.advertise<rt_dynamixel_msgs::JointSet>("rt_dynamixel/joint_set", 1);
        jointSub = nh.subscribe("rt_dynamixel/joint_state",1,&real_bridge::joint_callback,this);
        initMessage();
    }
    virtual ~real_bridge() {}

    void initMessage()
    {
        jointMsg.id.clear();
        jointMsg.angle.clear();
        jointMsg.angle.resize(nJoints);
        int maxID = 0;
        for(int i=0; i<nJoints; i++)
        {
            if(joint[i].id > maxID) maxID = joint[i].id;
        }
        id2index.resize(maxID);
        for(int i=0; i<nJoints; i++)
        {
            jointMsg.id.push_back(joint[i].id);
            id2index[joint[i].id] = i;
        }
    }

    void setJoint()
    {
        for(int i=0; i<nJoints; i++)
        {
            jointMsg.angle[i] = joint[i].desired_position;
        }
        jointSetPub.publish(jointMsg);
    }


    // Callback
    void joint_callback(const rt_dynamixel_msgs::JointStateConstPtr joint_msg)
    {
        // Update value (Need Mutex?)
        for (unsigned int i=0; i<joint_msg->id.size(); i++)
        {
            int index = id2index[ joint_msg->id[i] ];
            joint[index].position = joint_msg->angle[i];
            joint[index].velocity = joint_msg->velocity[i];
            joint[index].effort = joint_msg->current[i];
        }
    }
};

class vrep_bridge : public dyros_robot_state
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

public:
    vrep_bridge(ros::NodeHandle &nh, int joints, string *jointName, int *jointID) : dyros_robot_state(joints,jointName,jointID)
    {
        vrepHandleClient = nh.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
        vrepJointSetPub = nh.advertise<vrep_common::JointSetStateData>("vrep/JointSet",1);
        vrepJointSub = nh.subscribe("vrep/joint",1,&vrep_bridge::joint_callback,this); // TODO: modify topic name.
        // TODO: Write subscirber codes

        initialize_handler();
    }

    virtual ~vrep_bridge() {}

    void initialize_handler()
    {
        // Reset
        jointSetObject.handles.data.clear();
        jointSetObject.setModes.data.clear();
        jointSetObject.values.data.clear();


        vrep_common::simRosGetObjectHandle srv;

        jointSetObject.values.data.resize(nJoints);
        for (int i=0; i<nJoints; i++)
        {
            srv.request.objectName = joint[i].name;
            if (vrepHandleClient.call(srv))
            {
                jointSetObject.handles.data.push_back(srv.response.handle);
                jointSetObject.setModes.data.push_back(1);
                jointSetObject.values.data[i] = 0.0;
            }
        }
    }

    void setJoint()
    {
        for(int i=0; i<nJoints; i++)
        {
            jointSetObject.values.data[i] = joint[i].desired_position;
        }
        vrepJointSetPub.publish(jointSetObject);
    }

    // Callback

    void joint_callback(sensor_msgs::JointStateConstPtr joint_msg)
    {

    }
    void LFT_callback()
    {

    }
    void RLT_callback()
    {

    }
};
