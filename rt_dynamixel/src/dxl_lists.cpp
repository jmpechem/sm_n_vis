
#include "dxl_lists.h"

using namespace DXL_PRO;

// Defines (Debug, Test, ...)
#define DXL_TEST_SET

#ifdef DXL_TEST_SET
dxl_pro_data dxlLists[4][10] = {
    {
        // Index: 0
        {1, H54},
    },    {
        // Index: 1
    },    {
        // Index: 2
    },    {
        // Index: 3
    }
};   // Max 4channels, 10 motors
#else
dxl_pro_data dxlLists[4][10] = {
    {
        // Index: 0
        {1, H54},
        {3, H54},
        {5, H54},
        {7, H54},
        {9, H54},
        {11, H42},
        {13, H42}
    },    {
        // Index: 1
        {2, H54},
        {4, H54},
        {6, H54},
        {8, H54},
        {10, H54},
        {12, H42},
        {14, H42}
    },    {
        // Index: 2
        {15, H54},
        {17, H54},
        {19, H54},
        {21, H54},
        {23, H54},
        {25, H54},
        {27, H42}
    },    {
        // Index: 3
        {16, H54},
//        {18, H54},
        {20, H54},
        {22, H54},
        {24, H54},
        {26, H54}
    }
};   // Max 4channels, 10 motors
#endif
// Position P, Velocity P, Velocity I
dxl_gains dxlGains[4][10] =
{
    {
        // Index: 0
        {1, 1,1,1},
        {3, 1,1,1},
        {5, 1,1,1},
        {7, 1,1,1},
        {9, 1,1,1},
        {11, 1,1,1},
        {13, 1,1,1}
    },    {
        // Index: 1
        {2, 1,1,1},
        {4, 1,1,1},
        {6, 1,1,1},
        {8, 1,1,1},
        {10, 1,1,1},
        {12, 1,1,1},
        {14, 1,1,1}
    },    {
        // Index: 2
        {15, 1,1,1},
        {17, 1,1,1},
        {19, 1,1,1},
        {21, 1,1,1},
        {23, 1,1,1},
        {25, 1,1,1},
        {27, 1,1,1}
    },    {
        // Index: 3
        {16, 1,1,1},
        {18, 1,1,1},
        {20, 1,1,1},
        {22, 1,1,1},
        {24, 1,1,1},
        {26, 1,1,1}
    }
};

// Global Variables
RTDynamixelPro dxlDevice[4] = {0, 1, 2, 3}; // index set
int nTotalMotors;
int nDXLCount[4] = {0, };

dxl_inverse dxlID2Addr[60] = { 0, };    // max ID = 50,
// How to use: dxlLists[dxlID2Addr[id].channel][dxlID2Addr[id].index];
















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



bool dynamixel_motor_init()
{
    RT_TASK rtt_dxl_init;
    bool isDone = false;

    rt_task_create(&rtt_dxl_init,"rt_dxl_init",0,30,T_JOINABLE);
    rt_task_start(&rtt_dxl_init,&motion_init_proc,&isDone);
    rt_task_join(&rtt_dxl_init);
    rt_task_delete(&rtt_dxl_init);

    return isDone;
}

/**
 * @brief motion_init_proc
 * @param arg a param from rt_task_start()
 * @detail Set all dynamixels to init position
 */
void motion_init_proc(void *arg)
{
    bool *isDone = (bool*)arg;
    bool isUpdateComplete[4] = {false, };
    int nRecv[4] = {0, };
    for(int c=0; c<10;c++)
    {
        for(int i=0;i<4;i++)
        {
            dxlDevice[i].rttLoopStartTime = rt_timer_read();
            dxlDevice[i].rttLoopTimeoutTime = dxlDevice[i].rttLoopStartTime + 10e6; // 5ms
            nRecv[i] = dxlDevice[i].getAllStatus();
            if(nRecv[i] == dxlDevice[i].getMotorNum())
            {
                isUpdateComplete[i] = true;
            }
            dxlDevice[i].setReturnDelayTime(0);
            dxlDevice[i].setAllAcceleration(0);
            dxlDevice[i].setAllVelocity(0);
            dxlDevice[i].setAllTorque(0);
            // Gain Set
            /*
            for(int j=0; j<nDXLCount[i]; j++)
            {
                int err;
                dxlDevice[i].setPositionGain(j,dxlGains[i][j].position_p_gain, &err);
                dxlDevice[i].setVelocityGain(j,dxlGains[i][j].velocity_i_gain,
                                             dxlGains[i][j].velocity_p_gain, &err);
            }
            */

        }
        rt_task_sleep(1e8);

    }
    // check all motor read
    for(int i=0;i<4;i++)
        if(isUpdateComplete[i] == false)
        {
            *isDone = false;
           return;
        }

    // Update all radian angle
    for(int i=0;i<4; i++)
        for(int j=0;j<dxlDevice[i].getMotorNum(); j++)
        {
            dxlDevice[i][j].aim_radian = dxlDevice[i][j].position_rad();
        }
    *isDone = true;
}
