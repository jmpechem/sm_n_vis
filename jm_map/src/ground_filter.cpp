#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>


#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>
#include <vector>

using namespace std;
using namespace grid_map;

#define PI 3.14159265359
#define SLOPE_THRES   45
#define STEP_THRES    0.3
#define deg2rad(deg)  ((deg) * PI / 180.0)
#define rad2deg(rad)  ((deg) * 180.0 / PI)
/*class Ground_Filter{
	public:
	Ground_Filter();
	void GF_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud);
	double l2_norm(double input1, double input2);
	private:
	ros::NodeHandle nh_;
	ros::Subscriber cloud_;
	double r;

};
Ground_Filter::Ground_Filter(){
	r = sin(deg2rad(SLOPE_THRES));
	cloud_ = nh_.subscribe<sensor_msgs::PointCloud2>("pub_name",100,&Ground_Filter::GF_cb,this);
}
void Ground_Filter::GF_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud){
	// convert pointcloud2 to pointxyz
	pcl::PCLPointCloud2 pcl_pc2;
	pcl_conversions::toPCL(*cloud,pcl_pc2);
	pcl::PointCloud<pcl::PointXYZ> buf_cloud;
	pcl::fromPCLPointCloud2(pcl_pc2,buf_cloud);
	// pcl normal estimation using kdtree
	pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(buf_cloud.makeShared());
	normalEstimation.setKSearch(12);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	normalEstimation.compute(*normals);
	
	
}*/
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
bool _updated = false;

void Ground_Filter_cb(const sensor_msgs::PointCloud2::ConstPtr& input){
    cloud->clear();
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
    _updated = true;
}

int main(int argc, char** argv)
{  
////////////////////////////////////////////////////
  // using pcd data format code
////////////////////////////////////////////////////
/*  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/jimin/catkin_ws/src/jm_map/pcd_data_set/4.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read pcd file\n");
    return (-1);
  }  
    ROS_INFO("PCD file read success!!");

   pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normalEstimation;
   normalEstimation.setInputCloud(cloud);
   normalEstimation.setKSearch(12);
   pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
   normalEstimation.setSearchMethod(kdtree);
   pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
   normalEstimation.compute(*normals);

   double nx = 0.0f;
   double ny = 0.0f;
   double nz = 0.0f;
   double scale = 0.0f;
   double r;
   r = sin(deg2rad(SLOPE_THRES));
   // Normal filtered from ground
   pcl::PointCloud<pcl::PointXYZ>::Ptr assembly_ (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointXYZ temp;
   for(int i=0;i<normals->size();i++)
   {
      nx = normals->at(i).normal[0];
      ny = normals->at(i).normal[1];
      nz = normals->at(i).normal[2];
      scale = sqrt((nx*nx)+(ny*ny));
      if (scale < r){
            if(nz > 0){
                  temp.x = cloud->points[i].x;
                  temp.y = cloud->points[i].y;
                  temp.z = cloud->points[i].z;
                  assembly_->push_back(temp);
                       }
                    }
	}
   // Step filtered from ground
   pcl::PointXYZ filter_min;
   pcl::PointXYZ filter_max;
   pcl::getMinMax3D(*assembly_,filter_min,filter_max);
   double z_thres = 0.3;
   double z_support = 0.0f;
   for(int i=0;i<assembly_->size();i++)
   {
        if (filter_min.x == assembly_->points[i].x)
        {
                z_support = assembly_->points[i].z;
        }
   }
	pcl::PointCloud<pcl::PointXYZ> final_assembly_;
	for(int i=0;i<assembly_->size();i++)
	{
		if ( (abs(z_support)-abs(assembly_->points[i].z)) <= z_thres)
		{
			temp.x = assembly_->points[i].x;
			temp.y = assembly_->points[i].y;
			temp.z = assembly_->points[i].z;
			final_assembly_.push_back(temp);
		}
	}
	sensor_msgs::PointCloud2 output_;
	pcl::toROSMsg(final_assembly_,output_);	
	
  ros::init(argc, argv, "ground_filter");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("gf_cloud",100,false);
  output_.header.frame_id = "base_link";
  ros::Rate rate(30.0);
  while (nh.ok()) {
    ros::Time time = ros::Time::now();
    pub.publish(output_);
    rate.sleep();
  }*/
  ros::init(argc, argv, "ground_filter");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("gf_cloud",100,false);
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("shit",100,Ground_Filter_cb);

  ros::Rate rate(30.0);
  while (nh.ok()) {

      if(_updated){

          pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normalEstimation;
          normalEstimation.setInputCloud(cloud);
          normalEstimation.setKSearch(12);
          pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
          normalEstimation.setSearchMethod(kdtree);
          pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
          normalEstimation.compute(*normals);

          double nx = 0.0f;
          double ny = 0.0f;
          double nz = 0.0f;
          double scale = 0.0f;
          double r;
          r = sin(deg2rad(SLOPE_THRES));
          // Normal filtered from ground
          pcl::PointCloud<pcl::PointXYZ>::Ptr assembly_ (new pcl::PointCloud<pcl::PointXYZ>);
          pcl::PointXYZ temp;
          for(int i=0;i<normals->size();i++)
          {
             nx = normals->at(i).normal[0];
             ny = normals->at(i).normal[1];
             nz = normals->at(i).normal[2];
             scale = sqrt((nx*nx)+(ny*ny));
             if (scale < r){
                   if(nz > 0){
                         temp.x = cloud->points[i].x;
                         temp.y = cloud->points[i].y;
                         temp.z = cloud->points[i].z;
                         assembly_->push_back(temp);
                              }
                           }
               }
          // Step filtered from ground
          pcl::PointXYZ filter_min;
          pcl::PointXYZ filter_max;
          pcl::getMinMax3D(*assembly_,filter_min,filter_max);
          double z_thres = 0.3;
          double z_support = 0.0f;
          for(int i=0;i<assembly_->size();i++)
          {
               if (filter_min.x == assembly_->points[i].x)
               {
                       z_support = assembly_->points[i].z;
               }
          }
               pcl::PointCloud<pcl::PointXYZ> final_assembly_;
               for(int i=0;i<assembly_->size();i++)
               {
                       if ( (abs(z_support)-abs(assembly_->points[i].z)) <= z_thres)
                       {
                               temp.x = assembly_->points[i].x;
                               temp.y = assembly_->points[i].y;
                               temp.z = assembly_->points[i].z;
                               final_assembly_.push_back(temp);
                       }
               }
               sensor_msgs::PointCloud2 output_;
               pcl::toROSMsg(final_assembly_,output_);
               output_.header.frame_id = "base_link";
               ros::Time time = ros::Time::now();
               pub.publish(output_);
               _updated = false;
        }


    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
