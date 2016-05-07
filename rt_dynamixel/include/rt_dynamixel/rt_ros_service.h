
#ifndef RT_ROS_SERVICE_H_
#define RT_ROS_SERVICE_H_


#include <ros/ros.h>
#include <rosrt/rosrt.h>

#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>
#include <rtdk.h>

#include "rt_dynamixel_pro.h"
#include "rt_dynamixel_msgs/JointState.h"
#include "rt_dynamixel_msgs/JointSet.h"
#include "rt_dynamixel_msgs/ModeSetting.h"
#include "rt_dynamixel_msgs/MotorSetting.h"

#include "dxl_lists.h"

using namespace DXL_PRO;

// global ----------------------------------------------
extern RTDynamixelPro dxlDevice[4];

extern int nTotalMotors;
extern int nDXLCount[4];
extern dxl_inverse dxlID2Addr[60]; // max ID = 50,

extern dxl_pro_data& dxl_from_id(int id);

// ------------------------------------------------------


// rt_task_proc -----------------------------------------
void publisher_proc(void *arg);
void subscribe_proc(void *arg);
void motor_set_proc(void *arg);
// ------------------------------------------------------

class RTROSPublisher
{
    RT_TASK rttTaskObject;
public:
    rosrt::Publisher<rt_dynamixel_msgs::JointState> pubState;
    rt_dynamixel_msgs::JointStatePtr jointMsg;


    RTROSPublisher(ros::NodeHandle &nh);
    virtual ~RTROSPublisher()
    {        rt_task_delete(&rttTaskObject);    }
    void start()
    {        rt_task_start(&rttTaskObject, &publisher_proc, (void*)this);    }

};

class RTROSSubscriber
{
    RT_TASK rttSubscriber;
public:
    rosrt::Subscriber<rt_dynamixel_msgs::JointSet> subSetter;

    RTROSSubscriber(ros::NodeHandle &nh);
    virtual ~RTROSSubscriber()
    {        rt_task_delete(&rttSubscriber);    }

    void start()
    {        rt_task_start(&rttSubscriber, &subscribe_proc, (void*)this);    }

};



class RTROSMotorSettingService
{
private:
    RT_TASK rttMotorSetTask;

    bool modeSwitch(rt_dynamixel_msgs::ModeSettingRequest &req,
                    rt_dynamixel_msgs::ModeSettingResponse &res);

    bool motorSet(rt_dynamixel_msgs::MotorSettingRequest &req,
                    rt_dynamixel_msgs::MotorSettingResponse &res);

public:
    ros::ServiceServer modeServer;
    ros::ServiceServer motorServer;

    rt_dynamixel_msgs::MotorSettingRequest motorRequest;

    RTROSMotorSettingService(ros::NodeHandle &nh);
    virtual ~RTROSMotorSettingService()
    {        rt_task_delete(&rttMotorSetTask);    }



};



#endif // RT_ROS_SERVICE_H_
