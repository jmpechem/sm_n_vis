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
#define winding 	0
#define unwinding 	1.57
using namespace ros;
class Match_Filter{
	public:
	Match_Filter();
	void scanCB(const sensor_msgs::LaserScan::ConstPtr& scan);
	void dxlCB(const dynamixel_msgs::JointState::ConstPtr& enc);
	double dxl_enc_;	
	geometry_msgs::TransformStamped rot_tf;
	private:
	ros::NodeHandle nh_; // node handle for match_filter Class
	laser_geometry::LaserProjection projector_; // LaserScan to Point Cloud TF projector
	ros::Subscriber enc_; // for dynamixel encoder data
	ros::Subscriber scan_; // for UTM-30lx-ew laser scanner
	tf::TransformListener tfListener_; // laser scanner tf to point cloud
	ros::Subscriber enc; // dynamixel encoder receiver
	ros::Publisher point_cloud_pub_; // point cloud publisher
};

Match_Filter::Match_Filter(){
	scan_ = nh_.subscribe<sensor_msgs::LaserScan>("/lidar_scan",100,&Match_Filter::scanCB,this);
	enc = nh_.subscribe("/tilt_controller/state",1000,&Match_Filter::dxlCB,this);
	point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud",100,false);
	tfListener_.setExtrapolationLimit(ros::Duration(0.1));
}
void Match_Filter::dxlCB(const dynamixel_msgs::JointState::ConstPtr& enc){
        dxl_enc_ = enc->current_pos;
	ROS_INFO("%f",enc->current_pos);	
}

void Match_Filter::scanCB(const sensor_msgs::LaserScan::ConstPtr& scan){
	if(!tfListener_.waitForTransform(scan->header.frame_id,"/lidar_link",scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
        ros::Duration(1.0))){
     return;
}
	
	sensor_msgs::PointCloud2 cloud_in,cloud_out;
	projector_.transformLaserScanToPointCloud("/lidar_link",*scan,cloud_in,tfListener_);

	rot_tf.header.frame_id = "/lidar_link";
	rot_tf.transform.translation.x = 0;
	rot_tf.transform.translation.y = 0;
	rot_tf.transform.translation.z = 0;
	rot_tf.transform.rotation.x = dxl_enc_;
	rot_tf.transform.rotation.y = 0;
	rot_tf.transform.rotation.z = 0;
	rot_tf.transform.rotation.w = 1;
	tf2::doTransform(cloud_in,cloud_out,rot_tf);
	point_cloud_pub_.publish(cloud_out);
	
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"rot_lidar2_pcl");
	Match_Filter filter;	
	ros::spin();
	return 0;
}
