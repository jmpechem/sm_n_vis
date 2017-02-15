#pragma once

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

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
#include <boost/bind.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <string>
#include <vector>
#include <cmath>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>


#include <cv_bridge/cv_bridge.h>
#include "cv.h"
#include "highgui.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <opencv2/video/background_segm.hpp>
#include <cxcore.h>
#include <highgui.h>

#include <eigen3/Eigen/Dense>
#include <std_msgs/Int32MultiArray.h>
using namespace std;
using namespace grid_map;
using namespace cv;

#define PI 3.14159265359
#define deg2rad(deg)  ((deg) * PI / 180.0)
#define rad2deg(rad)  ((deg) * 180.0 / PI)
#define GRID_X_SIZE   10 //9 // 10
#define GRID_Y_SIZE   10 //9 // 10
#define GRID_X_OFFSET 5 //4.5 // 5
#define GRID_Y_OFFSET 0 //
#define GRID_RESOLUTION 0.05 //0.03 // 0.05
#define ROBOT_FOOTPRINT_SIZE   0.5
#define FLAT_PLANE_THRESHOLD   0.1
#define CIRCLE_SCALE           2
#define HEIGHT_LIMIT           8
#define FILTER_UPDATE_NUM      10//10
#define HEIGHT_THRESHOLD       0.3//0.3



class Global_Map_Builder{
      public:
      Global_Map_Builder();
      void Cloud2GridMap(const sensor_msgs::PointCloud2::ConstPtr& input);
      void PCD2GridMap();
      void Update();
      void Empty_Grid_Filter(int mask_size,GridMap& output_map, GridMap input_map);
      void Grid_Normal_Estimation(double gs_crit, double raidus,GridMap& output_map, GridMap input_map);
      void Global_Slope(double gs_crit,GridMap& output_map, GridMap input_map);
      void Global_Height(double foot_center_z,GridMap& output_map, GridMap input_map);
      void Global_Edge_Normal_Estimation(GridMap& output_map, GridMap input_map);
      void Global_Roughness(double gr_crit,double r_radius,GridMap& output_map, GridMap input_map);
      void Edge_Extraction(GridMap& output_map, GridMap input_map);
      void Global_Cost(double alpha,GridMap& output_map, GridMap input_map);
      void Global_Root(const std_msgs::Int32MultiArray::ConstPtr& root_array);
      int roundToInt(double x);

      private:
      ros::NodeHandle           nh_;
      ros::Subscriber           input_cloud_sub_;
      ros::Subscriber           grid_map_root_sub_;
      GridMap                   grid_map_;
      boost::recursive_mutex    MapMutex_;
      ros::Publisher            grid_pub_;
      std_msgs::Int32MultiArray root_arr_;
      std_msgs::Int32MultiArray root_x_;
      std_msgs::Int32MultiArray root_y_;

      int                       bird_view_map_[((int)(GRID_X_SIZE/GRID_RESOLUTION))*((int)(GRID_Y_SIZE/GRID_RESOLUTION))];

      bool is_first;
};
