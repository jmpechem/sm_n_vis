
#ifndef DYNAMIXEL_LIST_H_
#define DYNAMIXEL_LIST_H_


#include "rt_dynamixel_pro.h"
#include <ros/ros.h>

using namespace DXL_PRO;


void make_dxl_count();
void make_inverse_access_data();
bool dxl_initailize();
dxl_pro_data& dxl_from_id(int id);

void motion_init_proc(void *arg);
bool dynamixel_motor_init();
bool check_vaild_dxl_from_id(int id);


extern dxl_pro_data dxlLists[4][10];
extern dxl_gains dxlGains[4][10];

extern RTDynamixelPro dxlDevice[4]; // index set

extern int nTotalMotors;
extern int nDXLCount[4];

extern dxl_inverse dxlID2Addr[60];    // max ID = 50,

#endif
