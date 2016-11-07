#include <ros/ros.h>
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

#include <boost/thread/recursive_mutex.hpp>

//#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>
#include <vector>

using namespace std;


#define PI 3.14159265359
#define SLOPE_THRES   45
#define STEP_THRES    0.3
#define deg2rad(deg)  ((deg) * PI / 180.0)
#define rad2deg(rad)  ((deg) * 180.0 / PI)
/*
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
bool _updated = false;

void Ground_Filter_cb(const sensor_msgs::PointCloud2::ConstPtr& input){
    cloud->clear();
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
    _updated = true;
}
*/
int main(int argc, char** argv)
{  
////////////////////////////////////////////////////
  // using pcd data format code
////////////////////////////////////////////////////
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/jimin/catkin_ws/src/jm_map/pcd_data_set/4.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read pcd file\n");
    return (-1);
  }  
    ROS_INFO("PCD file read success!!");
/*
   pcl::PassThrough<pcl::PointXYZ> passThroughFilter;
   passThroughFilter.setInputCloud(cloud);
   passThroughFilter.setFilterFieldName("z");
   passThroughFilter.setFilterLimits(-10,0.5);
   passThroughFilter.filter(*cloud);
*/
  /*boost::recursive_mutex pointsMutex_;
  boost::recursive_mutex::scoped_lock scopedLockmap(pointsMutex_);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pass_cloud(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointXYZ point_min;
   pcl::PointXYZ point_max;
   pcl::getMinMax3D(*cloud,point_min,point_max);
   for(int i=0;i<cloud->size();i++)
   {
       if(cloud->points[i].z <= (point_min.z + 0.8))
         {
           pass_cloud->push_back(cloud->points[i]);
         }

   }
  scopedLockmap.unlock();*/
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

   pcl::PointXYZ filter_min;
   pcl::PointXYZ filter_max;
   pcl::getMinMax3D(*assembly_,filter_min,filter_max);
   pcl::PointCloud<pcl::PointXYZ> final_assembly_;
   double z_thres = 1.5;
   for(int i=0;i<assembly_->size();i++)
   {
       cout << "z : " << assembly_->points[i].z << "  min_z : " << filter_min.z << endl;
           if ( abs(filter_min.z-assembly_->points[i].z) <= z_thres)
           {
                   temp.x = assembly_->points[i].x;
                   temp.y = assembly_->points[i].y;
                   temp.z = assembly_->points[i].z;
                   final_assembly_.push_back(temp);
           }
   }
   // Step filtered from ground
   /*
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
	*/
	sensor_msgs::PointCloud2 output_;
	pcl::toROSMsg(final_assembly_,output_);
	//pcl::toROSMsg(*assembly_,output_);
	
  ros::init(argc, argv, "ground_filter");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("gf_cloud",100,false);
  output_.header.frame_id = "base_link";
  ros::Rate rate(15.0);
  while (nh.ok()) {
    ros::Time time = ros::Time::now();
    pub.publish(output_);
    rate.sleep();
  }
 /* ros::init(argc, argv, "ground_filter");
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
*/
  return 0;
}
