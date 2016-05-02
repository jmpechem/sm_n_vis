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
#include <std_msgs/Bool.h>
#include <pcl_ros/transforms.h>
#include <pcl/PCLPointCloud2.h>

#define winding 	-1.57
#define unwinding 	1.57
class SNA{	// sweep and assembly class
	public:
	SNA();
	void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud);
	void dxl_status_cb(const dynamixel_msgs::JointState::ConstPtr& msg);
	bool is_moving_;
	private:
	ros::NodeHandle nh_;
	ros::Subscriber cloud_; 
	ros::Subscriber dxl_status_;
	ros::Publisher  assembled_cloud_pub_;
	sensor_msgs::PointCloud2 input_;
	pcl::PointCloud<pcl::PointXYZ> assembly_;
	sensor_msgs::PointCloud2 output_;
};
SNA::SNA(){
	is_moving_ = false;
	cloud_ = nh_.subscribe<sensor_msgs::PointCloud2>("/cloud",100,&SNA::cloud_cb,this);
	dxl_status_ = nh_.subscribe("/tilt_controller/state",100,&SNA::dxl_status_cb,this);
	assembled_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/assembled_cloud",100,false);
}
void SNA::dxl_status_cb(const dynamixel_msgs::JointState::ConstPtr& msg)
{
	is_moving_ = msg->is_moving;//current_pos;
}
void SNA::cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
	pcl::PCLPointCloud2 pcl_pc2;
	pcl_conversions::toPCL(*cloud,pcl_pc2);
	pcl::PointCloud<pcl::PointXYZ> buffer_cloud;
	pcl::fromPCLPointCloud2(pcl_pc2,buffer_cloud);
	pcl::PointXYZ temp;
	
	for(size_t i=0;i<buffer_cloud.size();i++)
	{	
		temp.x = buffer_cloud.points[i].x;
		temp.y = buffer_cloud.points[i].y;
		temp.z = buffer_cloud.points[i].z;		
		assembly_.push_back(temp);	
	}
	pcl::toROSMsg(assembly_,output_);
	output_.header.frame_id = "/base";	
	assembled_cloud_pub_.publish(output_);
	if(!is_moving_)
	{			
		assembly_.clear();
	}
	else
	{

	}
}


int main(int argc, char** argv)
{
	ros::init(argc,argv,"sweep_n_assembly");
	SNA sna;	
	ros::spin();
	return 0;
}
