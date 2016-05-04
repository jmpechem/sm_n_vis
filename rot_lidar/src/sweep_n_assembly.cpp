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
#include <std_msgs/Int32.h>
#include <cmath>
#define SWEEP_CMD	1

#define LEFT		0
#define RIGHT	 	3.14
class SNA{	// sweep and assembly class
	public:
	SNA();
	void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud);
	void dxl_status_cb(const dynamixel_msgs::JointState::ConstPtr& msg);
	void command_cb(const std_msgs::Int32::ConstPtr& cmd);
	private:
	ros::NodeHandle nh_;
	ros::Subscriber cloud_; 
	ros::Subscriber dxl_status_;
	ros::Subscriber cmd_;
	ros::Publisher  assembled_cloud_pub_;
	ros::Publisher  dxl_control_pub_;
	sensor_msgs::PointCloud2 input_;
	pcl::PointCloud<pcl::PointXYZ> assembly_;
	sensor_msgs::PointCloud2 output_;
	sensor_msgs::PointCloud2 remap_;
	bool is_moving_;
	bool has_cmd_;
	bool dxl_init_;
	double  compare_[2];
	double dxl_err_;
	std_msgs::Float64 rot_cmd;
};
SNA::SNA(){
	dxl_init_ = true;
	is_moving_ = false;
	has_cmd_ = false;
	compare_[0] = 0.0f;
	compare_[1] = 0.0f;
	dxl_err_ = 0.0f;
	cloud_ = nh_.subscribe<sensor_msgs::PointCloud2>("/cloud",100,&SNA::cloud_cb,this);
	cmd_ = nh_.subscribe<std_msgs::Int32>("sweep_cmd",100,&SNA::command_cb,this);
	dxl_status_ = nh_.subscribe("/tilt_controller/state",100,&SNA::dxl_status_cb,this);
	assembled_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/assembled_cloud",100,false);
	dxl_control_pub_ = nh_.advertise<std_msgs::Float64>("/tilt_controller/command",100,false);	
}
void SNA::dxl_status_cb(const dynamixel_msgs::JointState::ConstPtr& msg)
{
	is_moving_ = msg->is_moving;
	dxl_err_ = msg->error;
	compare_[0] = std::abs(msg->current_pos - LEFT);
	compare_[1] = std::abs(RIGHT - msg->current_pos);
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
	if(!is_moving_)
	{			
		assembly_.clear();
	}
	else
	{
		output_.header.frame_id = "/base";	
		assembled_cloud_pub_.publish(output_);
		if(dxl_err_ < 0.01){
		remap_ = output_;
		assembled_cloud_pub_.publish(remap_);
		}
	}
}
void SNA::command_cb(const std_msgs::Int32::ConstPtr& cmd)
{
	if(dxl_init_)
	{
		rot_cmd.data = RIGHT;
		dxl_control_pub_.publish(rot_cmd);
		dxl_init_ = false;
	}
	else if(cmd->data == SWEEP_CMD)
	{
				
		if(compare_[0] < compare_[1]){		
		rot_cmd.data = RIGHT;
		dxl_control_pub_.publish(rot_cmd);
		}
		else if(compare_[0] >= compare_[1]){
		rot_cmd.data = LEFT;
		dxl_control_pub_.publish(rot_cmd);
		}
		has_cmd_ = true;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"sweep_n_assembly");
	SNA sna;	
	ros::spin();
	return 0;
}
