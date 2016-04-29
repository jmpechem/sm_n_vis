#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <dynamixel_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

class SNA{	// sweep and assembly class
	public:
	SNA();

	private:
}
SNA::SNA(){

}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"sweep_n_assembly");
	SNA sna;	
	ros::spin();
	return 0;
}
