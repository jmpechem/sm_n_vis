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
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <vector>
#include <std_msgs/Float32MultiArray.h>
using namespace std;

#define PI 3.14159265359
#define SLOPE_THRES   90
#define STEP_THRES    3//0.3
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

pcl::PointCloud<pcl::PointXYZ>::Ptr clouds (new pcl::PointCloud<pcl::PointXYZ>);

class Ground_Filter{
      public:
      Ground_Filter();
      void Ground_Filter_CB(const sensor_msgs::PointCloud2::ConstPtr& input_cloud);
      void Ground_Filter_Param_CB(const std_msgs::Float32MultiArray::ConstPtr& params);
      void process();

      private:
      ros::NodeHandle nh_;
      ros::Subscriber input_cloud_sub_;
      ros::Subscriber input_param_sub_;
      ros::Publisher  output_cloud_pub_;
      float set_params[2];

};
Ground_Filter::Ground_Filter(){
 //    input_cloud_sub_ = nh_.subscribe("clouds",100,&Ground_Filter::Ground_Filter_CB,this);
 //    input_param_sub_ = nh_.subscribe("GF/params",100,&Ground_Filter::Ground_Filter_Param_CB,this);
     output_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("gf_cloud",100,false);
     set_params[0] = SLOPE_THRES;
     set_params[1] = STEP_THRES;
     clouds->clear();
}
void Ground_Filter::Ground_Filter_CB(const sensor_msgs::PointCloud2::ConstPtr &input_cloud)
{
    clouds->clear();
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input_cloud,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*clouds);
}
void Ground_Filter::Ground_Filter_Param_CB(const std_msgs::Float32MultiArray::ConstPtr &params)
{
    int cnt = 0;
    for(std::vector<float>::const_iterator it = params->data.begin(); it != params->data.end(); ++it)
    {
         set_params[cnt] = *it;
         cnt++;
    }

}
void Ground_Filter::process()
{
  if(clouds->empty())
  {
      if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/jimin/catkin_ws/src/jm_map/pcd_data_set/d2_ds.pcd", *clouds) == -1) //* load the file
      {
        PCL_ERROR ("Couldn't read pcd file\n");
        return ;
      }
       ROS_INFO("PCD file read success!!");
  }
  pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud(clouds);
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
  r = sin(deg2rad(set_params[0]));
  // Normal filtered from ground
  pcl::PointCloud<pcl::PointXYZ>::Ptr assembly_ (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ temp;
  for(int i=0;i<normals->size();i++)
  {
      temp.x = clouds->points[i].x;
      temp.y = clouds->points[i].y;
      temp.z = clouds->points[i].z;
      assembly_->push_back(temp);
      /*
     nx = normals->at(i).normal[0];
     ny = normals->at(i).normal[1];
     nz = normals->at(i).normal[2];
     scale = sqrt((nx*nx)+(ny*ny));
     if (scale < r){
           if(nz > 0){
                 temp.x = clouds->points[i].x;
                 temp.y = clouds->points[i].y;
                 temp.z = clouds->points[i].z;
                 assembly_->push_back(temp);
                      }
                   }*/
       }

  pcl::PointXYZ filter_min;
  pcl::PointXYZ filter_max;
  pcl::getMinMax3D(*assembly_,filter_min,filter_max);
  pcl::PointCloud<pcl::PointXYZ> final_assembly_;
  double z_thres = (double)set_params[1];
  for(int i=0;i<assembly_->size();i++)
  {
          if ( abs(filter_min.z-assembly_->points[i].z) <= z_thres)
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
  output_cloud_pub_.publish(output_);



  // Convert To ROS data type
/*
   pcl::PCLPointCloud2 cloud_p;
   pcl::toPCLPointCloud2(*h_clouds, cloud_p);

   sensor_msgs::PointCloud2 output;
   pcl_conversions::fromPCL(cloud_p, output);
   output.header.frame_id = "base_link";
   output_cloud_pub_.publish(output);
*/
}

int main(int argc, char** argv)
{  
////////////////////////////////////////////////////
  // using pcd data format code
////////////////////////////////////////////////////
  ros::init(argc, argv, "ground_filter");
  Ground_Filter gf;
  ros::NodeHandle nh;
   ros::Rate rate(10.0);
  while(nh.ok())
    {
    gf.process();
    ros::spinOnce();
    }

  /*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/jimin/catkin_ws/src/jm_map/pcd_data_set/E_stair.pcd", *cloud) == -1) //* load the file
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

   pcl::PointXYZ filter_min;
   pcl::PointXYZ filter_max;
   pcl::getMinMax3D(*assembly_,filter_min,filter_max);
   pcl::PointCloud<pcl::PointXYZ> final_assembly_;
   double z_thres = 1.8;
   for(int i=0;i<assembly_->size();i++)
   {
           if ( abs(filter_min.z-assembly_->points[i].z) <= z_thres)
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

  ros::Rate rate(15.0);
  while (nh.ok()) {

    ros::Time time = ros::Time::now();
    pub.publish(output_);
    rate.sleep();
  } */
  return 0;
}
