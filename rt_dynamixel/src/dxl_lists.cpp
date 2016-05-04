
#include "dxl_lists.h"

using namespace DXL_PRO;

// Defines (Debug, Test, ...)
// #define DXL_TEST_SET

dxl_pro_data dxlLists[4][10] = {
    {
#ifdef DXL_TEST_SET
        {56, H54},
#else
        // Index: 0
        {1, H54},
        {3, H54},
        {5, H54},
        {7, H54},
        {9, H54},
        {11, H42},
        {13, H42}
#endif
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




