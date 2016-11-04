//////////////////////////////////////////////////////////////////
//   	              thormang controller 			//
//			made by jimin lee			//
//////////////////////////////////////////////////////////////////

#include "sim_common.h"  // if you want to use vrep use sim common
#include "rob_common.h" // if you want to use real robot use rob common

#include <fstream>

using namespace std;
//
/*
class logger
{
public:
    logger() {
        fileName = "ctrl jitter";
        ros::Time::now
    }

private:
    fstream file;
    string fileName;

};
*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thormang_ctrl");
  
// controlBase *ctrObj = new realrobot;
controlBase *ctrObj = new simulation;


  while(ros::ok())
  {
     ctrObj->readdevice();
     ctrObj->update();
     ctrObj->compute();
     ctrObj->reflect();
     ctrObj->writedevice();
     ctrObj->wait();
  }

  delete ctrObj;

  return 0;
}

/*
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
}*/
