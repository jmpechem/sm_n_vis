#include "jm_global/global_map.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "jm_global_map");
  ros::NodeHandle nh;

  Global_Map_Builder g_map;

  ros::Rate rate(10.0);


  while(nh.ok())
    {
      g_map.Update();
      ros::spinOnce();
    }
}
