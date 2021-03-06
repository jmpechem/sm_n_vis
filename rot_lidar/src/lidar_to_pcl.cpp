#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
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
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <std_msgs/Bool.h>
using namespace ros;
class Match_Filter{
	public:
	Match_Filter();
	void scanCB(const sensor_msgs::LaserScan::ConstPtr& scan);
	void dxlCB(const dynamixel_msgs::JointState::ConstPtr& enc);
	double dxl_enc_;
	bool dxl_ismove_;	
	std_msgs::Bool moving_now;
	geometry_msgs::TransformStamped rot_tf;
	sensor_msgs::PointCloud2 assembled_pcd_;
	private:
	ros::NodeHandle nh_; // node handle for match_filter Class
	laser_geometry::LaserProjection projector_; // LaserScan to Point Cloud TF projector
	ros::Subscriber enc_; // for dynamixel encoder data
	ros::Subscriber scan_; // for UTM-30lx-ew laser scanner
	tf::TransformListener tfListener_; // laser scanner tf to point cloud
	ros::Subscriber enc; // dynamixel encoder receiver
	ros::Publisher point_cloud_pub_; // point cloud publisher
	ros::Publisher dxl_ismove_pub_; // dxl moving check publisher
	tf::TransformBroadcaster enc_br_; // To broadcast dxl frame
	tf::Transform enc_tf_;	// making frame information of dxl frame
	tf::TransformBroadcaster cloud_br_;
	tf::Transform cloud_tf_;

};

Match_Filter::Match_Filter(){
	scan_ = nh_.subscribe<sensor_msgs::LaserScan>("/lidar_scan",100,&Match_Filter::scanCB,this);
	enc = nh_.subscribe("/tilt_controller/state",1000,&Match_Filter::dxlCB,this);
	point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud",100,false);
	dxl_ismove_pub_ = nh_.advertise<std_msgs::Bool>("dxl_ismove",100,false);
}
void Match_Filter::dxlCB(const dynamixel_msgs::JointState::ConstPtr& enc){
        dxl_enc_ = enc->current_pos;
	dxl_ismove_ = enc->is_moving;
	enc_tf_.setOrigin(tf::Vector3(0.0,0.0,0.0));
	tf::Quaternion q;
	//q.setRPY(dxl_enc_,0,0); // lidar mx28 axis aligned
//	q.setRPY(0,0,-dxl_enc_+M_PI/2); // lidar mx28 axis perpendicular
	q.setRPY(0,0,-dxl_enc_+M_PI/2); // lidar mx28 axis perpendicular
	enc_tf_.setRotation(q);
	enc_br_.sendTransform(tf::StampedTransform(enc_tf_, ros::Time::now(), "ChestLidar", "dxl_link"));	
	
}

void Match_Filter::scanCB(const sensor_msgs::LaserScan::ConstPtr& scan){		
	sensor_msgs::PointCloud2 cloud_in,cloud_out;
	projector_.transformLaserScanToPointCloud("/lidar_link",*scan,cloud_in,tfListener_);
	rot_tf.header.frame_id = "/ChestLidar";
//	lidar mx28 axis aligned mode
//	rot_tf.transform.rotation.x = enc_tf_.getRotation().x();
//	rot_tf.transform.rotation.y = enc_tf_.getRotation().y();
//	rot_tf.transform.rotation.z = enc_tf_.getRotation().z();
//	rot_tf.transform.rotation.w = enc_tf_.getRotation().w();

//	lidar mx28 axis perpendicular modeg
	tf::Quaternion q1;
	q1.setRPY(-M_PI/2,0,0);
	tf::Transform m1(q1);
	tf::Quaternion q2(enc_tf_.getRotation().x(),enc_tf_.getRotation().y(),enc_tf_.getRotation().z(),enc_tf_.getRotation().w());
	tf::Transform m2(q2);

	tf::Transform m4;
	m4 = m2*m1; // rotate lidar axis and revolute mx28 axis
	rot_tf.transform.rotation.x = m4.getRotation().x();
	rot_tf.transform.rotation.y = m4.getRotation().y();
	rot_tf.transform.rotation.z = m4.getRotation().z();
	rot_tf.transform.rotation.w = m4.getRotation().w();
	
	tf2::doTransform(cloud_in,cloud_out,rot_tf);
	point_cloud_pub_.publish(cloud_out);		
	moving_now.data = dxl_ismove_;
	dxl_ismove_pub_.publish(moving_now);
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"rot_lidar2_pcl");
	Match_Filter filter;	
	ros::spin();
	return 0;
}
