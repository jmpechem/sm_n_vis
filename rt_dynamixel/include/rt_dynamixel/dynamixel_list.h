#include "rt_dynamixel_pro.h"

// Defines (Debug, Test, ...)
#define DXL_TEST_SET

using namespace DXL_PRO;

dxl_pro_data dxlLists[4][10] = {
    {
#ifdef DXL_TEST_SET
        {57, H54},
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

