//////////////////////////////////////////////////////////////////
//   	              thormang controller 			//
//			made by jimin lee			//
//////////////////////////////////////////////////////////////////

//#include "sim_common.h"  // if you want to use vrep use sim common
#include "rob_common.h" // if you want to use real robot use rob common

//
/*
int main(int argc, char **argv)
{
  ros::init(argc, argv, "thormang_ctrl");
  
  simulation simobj;
  simobj.vrep_start();
  simobj.vrep_initialize();
  simobj.parameter_initialize();
  ros::Rate r(100);

  while(ros::ok())
  {
     simobj.readdevice();
     simobj.update();
     simobj.compute();
     simobj.reflect();
     simobj.writedevice();
     r.sleep();
  }
  simobj.vrep_end();
  return 0;
}
*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thormang_ctrl");

    realrobot robotObj;

    ros::Rate r(100);

    while(ros::ok())
    {
          robotObj.readdevice();
          robotObj.update();
          robotObj.compute();
          robotObj.reflect();
          robotObj.writedevice();
          r.sleep();
    }

    return 0;
}
