
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

#include "dynamixel_list.h"

using namespace DXL_PRO;

// Main Functions
void make_dxl_count();
void make_inverse_access_data();
bool dxl_initailize();
dxl_pro_data& dxl_from_id(int id);

// Global Variables
RTDynamixelPro dxlDevice[4] = {0, 1, 2, 3}; // index set

int nTotalMotors;
int nDXLCount[4] = {0, };

dxl_inverse dxlID2Addr[60] = { 0, };    // max ID = 50,
// How to use: dxlLists[dxlID2Addr[id].channel][dxlID2Addr[id].index];

void publisher_proc(void *arg);
void motion_init_proc(void *arg);
void subscribe_proc(void *arg);

class RTROSPublisher
{
    RT_TASK rttTaskObject;
public:
    RTROSPublisher(ros::NodeHandle &nh)
    {
        pubState.initialize(nh.advertise<rt_dynamixel_msgs::JointState>("rt_dyanmixel_state",0),
                            1, rt_dynamixel_msgs::JointState());
        rt_task_create(&rttTaskObject,"dxl ros pub",0,10,0);
        jointMsg = pubState.allocate();

        jointMsg->id.resize(nTotalMotors);
        jointMsg->angle.resize(nTotalMotors);
        jointMsg->velocity.resize(nTotalMotors);
        jointMsg->current.resize(nTotalMotors);

        int _cnt=0;
        for(int i=0;i<4;i++)
            for(int j=0;j<nDXLCount[i];j++)
            {
                jointMsg->id[_cnt++] = dxlLists[i][j].id;
            }

    }
    virtual ~RTROSPublisher()
    {
        rt_task_delete(&rttTaskObject);
    }
    void start()
    {
        rt_task_start(&rttTaskObject, &publisher_proc, (void*)this);
    }

    rosrt::Publisher<rt_dynamixel_msgs::JointState> pubState;
    rt_dynamixel_msgs::JointStatePtr jointMsg;
};

class RTROSSubscriber
{
    RT_TASK rttSubscriber;
public:
    RTROSSubscriber(ros::NodeHandle &nh)
    {
        rt_task_create(&rttSubscriber,"dxl ros sub",0,9,0);
        subSetter.initialize(3,nh,"rt_dynamixel/joint_set");
    }
    virtual ~RTROSSubscriber()
    {
        rt_task_delete(&rttSubscriber);
    }

    void start()
    {
        rt_task_start(&rttSubscriber, &subscribe_proc, (void*)this);
    }

    rosrt::Subscriber<rt_dynamixel_msgs::JointSet> subSetter;
};


/**
 * @brief main proc
 * @return
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "rt_dynamixel_test");
    rosrt::init();

    ros::NodeHandle nh;

    RTROSPublisher rtRosPublisher(nh);
    RTROSSubscriber rtRosSubscriber(nh);

    if(dxl_initailize() == false) return -1;

    //dxlDevice[0].bControlLoopEnable = true;
    rtRosPublisher.start();
    rtRosSubscriber.start();

    ros::spin();

    return 0;
}










void make_dxl_count()
{
    int i,j;
    for(i=0;i<4;i++)
    {
        for(j=0;j<10;j++)
        {
            if(dxlLists[i][j].type == 0)  { break; }
        }
        nDXLCount[i] = j;
    }
    nTotalMotors =  nDXLCount[0] + nDXLCount[1] + nDXLCount[2] + nDXLCount[3];
}

void make_inverse_access_data()
{
    int i,j;
    for(i=0;i<4;i++)
    {
        for(j=0;j<nDXLCount[i];j++)
        {
            dxlID2Addr[dxlLists[i][j].id].channel = i;
            dxlID2Addr[dxlLists[i][j].id].index = j;
        }
    }
}


bool dxl_initailize()
{
    make_dxl_count();
    make_inverse_access_data();

    dxlDevice[0].ComPort->SetPortName("rtser0");
    dxlDevice[1].ComPort->SetPortName("rtser1");
    dxlDevice[2].ComPort->SetPortName("rtser2");
    dxlDevice[3].ComPort->SetPortName("rtser3");

    int i;
    bool error = 0;

    for(i=0; i<4;i++)
    {
        dxlDevice[i].setIDList(nDXLCount[i], dxlLists[i]);
        error = dxlDevice[i].Connect();
        if(error == false)
        {
            ROS_ERROR("Error on openning serial device: rtser%d",i);
            return false;
        }
        dxlDevice[i].startThread();
    }


    return true;
}


dxl_pro_data& dxl_from_id(int id)
{
    return dxlDevice[dxlID2Addr[id].channel][dxlID2Addr[id].index];
}




/**
 * @brief motion_init_proc
 * @param arg a param from rt_task_start()
 * @detail Set all dynamixels to init position
 */
void motion_init_proc(void *arg)
{
    int i,j;
    for(i=0;i<4;i++)
    {
        dxlDevice[i].setReturnDelayTime(0);
        dxlDevice[i].setAllAcceleration(0);
        dxlDevice[i].setAllVelocity(0);
        dxlDevice[i].setAllTorque(1);
    }
}


void publisher_proc(void *arg)
{
    RTROSPublisher* pObj = (RTROSPublisher*)arg;
    int i,j;

    rt_task_set_periodic(NULL, TM_NOW, 2e6);    // 1e6 -> 1ms   5e5 -> 500us

    while (1)
    {
        rt_task_wait_period(NULL); //wait for next cycle

        // Data set
        for(i=0;i<4;i++)
        {
            dxlDevice[i].mutex_acquire();
        }

        int _cnt = 0;
        for(i=0;i<4;i++)
        {
            for(j=0;j<nDXLCount[i];j++)
            {
                // SI
                pObj->jointMsg->angle[_cnt] = dxlDevice[i][j].position_rad();
                pObj->jointMsg->velocity[_cnt] = dxlDevice[i][j].velocity_radsec();
                pObj->jointMsg->current[_cnt] = dxlDevice[i][j].current_amp();
                _cnt++;
            }
        }

        for(i=0;i<4;i++)
        {
            dxlDevice[i].mutex_release();
        }

        pObj->pubState.publish(pObj->jointMsg);
    }
}



void subscribe_proc(void *arg)
{
    RTROSSubscriber* rtsub = (RTROSSubscriber*)arg;
    rt_task_set_periodic(NULL, TM_NOW, 1e6);    // 1e6 -> 1ms
    int i;
    while(1)
    {
        rt_task_wait_period(NULL); //wait for next cycle

        rt_dynamixel_msgs::JointSetConstPtr rcvMsg = rtsub->subSetter.poll();
        if(rcvMsg) // if message recieved ( if not rcvMsg == NULL )
        {
            // Data set
            for(i=0;i<4;i++)
            {
                dxlDevice[i].mutex_acquire();
            }

            for(i=0;i< (int)rcvMsg->id.size();i++)
            {
                dxl_from_id(rcvMsg->id[i]).aim_radian = rcvMsg->angle[i];
            }

            for(i=0;i<4;i++)
            {
                dxlDevice[i].mutex_release();
            }
        }
    }
}


class RTROSMotorSettingService
{

    bool modeSwitch(rt_dynamixel_msgs::ModeSettingRequest &req,
                    rt_dynamixel_msgs::ModeSettingResponse &res)
    {

        switch (req.mode)
        {
        case rt_dynamixel_msgs::ModeSettingRequest::CONTROL_RUN:
            for(int i=0;i<4;i++)
            {
                dxlDevice[i].bControlLoopEnable = true;
            }
            break;
        case rt_dynamixel_msgs::ModeSettingRequest::DISABLE:
            for(int i=0;i<4;i++)
            {
                dxlDevice[i].bControlLoopEnable = false;
            }

            // wait for process end
            for(int i=0;i<4;i++)
                while(dxlDevice[i].bControlLoopProcessing) {}


            break;
        case rt_dynamixel_msgs::ModeSettingRequest::SETTING:
            for(int i=0;i<4;i++)
            {
                dxlDevice[i].bControlLoopEnable = false;
            }

            // wait for process end
            for(int i=0;i<4;i++)
                while(dxlDevice[i].bControlLoopProcessing) {}


            break;

        default:
            break;
        }


        return true;
    }

    bool motorSet(rt_dynamixel_msgs::MotorSettingRequest &req,
                    rt_dynamixel_msgs::MotorSettingResponse &res)
    {

        return true;
    }

public:
    ros::ServiceServer modeServer;
    ros::ServiceServer motorServer;

    rt_dynamixel_msgs::MotorSettingRequest motorRequest;

    RTROSMotorSettingService(ros::NodeHandle &nh)
    {
        modeServer = nh.advertiseService("rt_dynamixel/mode",&RTROSMotorSettingService::modeSwitch,this);
        motorServer = nh.advertiseService("rt_dynamixel/motor_set",&RTROSMotorSettingService::modeSwitch,this);
    }



};

void motor_set_proc(void *arg)
{

}

/*
void motion_init_proc(void *arg)
{
    int i,j;

    rt_task_set_periodic(NULL, TM_NOW, 250e6);    // 1e6 -> 1ms   250ms

    for(i=0;i<4;i++)
    {
        dxlDevice[i].setReturnDelayTime(0);
        dxlDevice[i].setAllAcceleration(30);
        dxlDevice[i].setAllVelocity(1000);
        dxlDevice[i].setAllTorque(1);
    }



    while(1)
    {
        rt_task_wait_period(NULL); //wait for next cycle

        for(i=0;i<4;i++)
        {
            dxlDevice[i].setAllDegree(0.0);
            dxlDevice[i].rttLoopStartTime = rt_timer_read();
            dxlDevice[i].rttLoopTimeoutTime = dxlDevice[i].rttLoopStartTime + 1e7;
            dxlDevice[i].getAllStatus();
        }

        printf("[INFO] Position List \n");

        bool isDone = true;
        for(i=0;i<4;i++)
        {
            for(j=0;j<nDXLCount[i];j++)
            {
                if(abs(dxlDevice[i][j].position) > 200) isDone = false;
                printf("%d\t", dxlDevice[i][j].position);
                printf("\n");
            }

        }
        if(isDone == true)
        {
            printf("[INFO] Motor Init Complete \n");
            break;
        }

    }

    rt_task_sleep(3000e6);

    for(i=0;i<4;i++)
    {
        dxlDevice[i].setAllAcceleration(0);
        dxlDevice[i].setAllVelocity(0);
    }
}
*/
