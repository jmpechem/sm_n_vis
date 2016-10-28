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

#include <boost/thread/recursive_mutex.hpp>
#include <boost/bind.hpp>

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
#define GRID_X_SIZE   10
#define GRID_Y_SIZE   10
#define GRID_X_OFFSET 5
#define GRID_Y_OFFSET 0

 boost::recursive_mutex rawMapMutex_;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

void gf_cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& gf_cloud){
    cloud->clear();
     pcl::PCLPointCloud2 pcl_pc2;
     pcl_conversions::toPCL(*gf_cloud,pcl_pc2);
     pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
}

void grid_normal_estimation(GridMap& map,float radius,GridMap input_map){
  boost::recursive_mutex::scoped_lock scopedLockmap(rawMapMutex_);
  map=input_map;
  Eigen::Vector3d surfaceNormalPositiveAxis_;
  surfaceNormalPositiveAxis_ = grid_map::Vector3::UnitZ();
  vector<string> surfaceNormalTypes;
  surfaceNormalTypes.push_back("normal_x");
  surfaceNormalTypes.push_back("normal_y");
  surfaceNormalTypes.push_back("normal_z");

  for(GridMapIterator iter(map);!iter.isPastEnd();++iter){
    if(!map.isValid(*iter,"elevation")) continue;
    if(map.isValid(*iter,surfaceNormalTypes)) continue;
    Length submapLength = Length::Ones() * (2.0 * radius);
    Position submapPosition;
    map.getPosition(*iter,submapPosition);
    Index submapTopLeftIndex, submapBufferSize, requestedIndexInSubmap;
    getSubmapInformation(submapTopLeftIndex, submapBufferSize, submapPosition, submapLength,
                             requestedIndexInSubmap, submapPosition, submapLength,
                             map.getLength(), map.getPosition(), map.getResolution(),
                             map.getSize(), map.getStartIndex());

    const int maxNumberOfCells = submapBufferSize.prod();
    Eigen::MatrixXd points(3, maxNumberOfCells);

    size_t nPoints = 0;
       for (SubmapIterator submapIterator(map, submapTopLeftIndex, submapBufferSize); !submapIterator.isPastEnd(); ++submapIterator) {
         if (!map.isValid(*submapIterator,"elevation")) continue;
         Position3 point;
         map.getPosition3("elevation", *submapIterator, point);
         points.col(nPoints) = point;
         nPoints++;
       }

       const Eigen::Vector3d mean = points.leftCols(nPoints).rowwise().sum() / nPoints;
       const Eigen::MatrixXd NN = points.leftCols(nPoints).colwise() - mean;

       const Eigen::Matrix3d covarianceMatrix(NN * NN.transpose());
       Eigen::Vector3d eigenvalues = Eigen::Vector3d::Identity();
       Eigen::Matrix3d eigenvectors = Eigen::Matrix3d::Identity();
           if (covarianceMatrix.fullPivHouseholderQr().rank() >= 3) {
             const Eigen::EigenSolver<Eigen::MatrixXd> solver(covarianceMatrix);
             eigenvalues = solver.eigenvalues().real();
             eigenvectors = solver.eigenvectors().real();
           } else {
             ROS_DEBUG("Covariance matrix needed for eigen decomposition is degenerated. Expected cause: no noise in data (nPoints = %i)", (int) nPoints);
           }
       int smallestId(0);
       double smallestValue(numeric_limits<double>::max());
       for (int j = 0; j < eigenvectors.cols(); j++) {
           if (eigenvalues(j) < smallestValue) {
              smallestId = j;
              smallestValue = eigenvalues(j);
           }
           }
           Eigen::Vector3d eigenvector = eigenvectors.col(smallestId);
           if (eigenvector.dot(surfaceNormalPositiveAxis_) < 0.0) eigenvector = -eigenvector;
              map.at("normal_x",*iter)=eigenvector.x();
              map.at("normal_y",*iter)=eigenvector.y();
              map.at("normal_z",*iter)=eigenvector.z();
  }
  scopedLockmap.unlock();
}
void slope_filter(GridMap& map,double s_critic,GridMap input_map)
{
  boost::recursive_mutex::scoped_lock scopedLockmap(rawMapMutex_);
  map=input_map;
  double slope, slopeMax = 0.0;
  for (GridMapIterator iter(map); !iter.isPastEnd();++iter){
      if (!map.isValid(*iter, "normal_z")) continue;
       slope = acos(map.at("normal_z", *iter));

       if (slope < s_critic) {
             map.at("slope", *iter) = 1.0 - slope / s_critic;
           }
           else {
             map.at("slope", *iter) = 0.0;
           }

           if (slope > slopeMax) slopeMax = slope;
    }  
  scopedLockmap.unlock();
}
void roughness_filter(GridMap& map,double est_radius,double r_critic,GridMap input_map)
{
  boost::recursive_mutex::scoped_lock scopedLockmap(rawMapMutex_);
  map=input_map;
  double roughnessmax = 0.0;

  for (GridMapIterator iterator(map);!iterator.isPastEnd(); ++iterator) {

      if (!map.isValid(*iterator, "normal_x")) continue;

          const int maxNumberOfCells = ceil(pow(2*est_radius/map.getResolution(),2));
          Eigen::MatrixXd points(3, maxNumberOfCells);

          // Requested position (center) of circle in map.
          Position center;
          map.getPosition(*iterator, center);

          // Gather surrounding data.
          size_t nPoints = 0;
          for (CircleIterator submapIterator(map, center, est_radius);
              !submapIterator.isPastEnd(); ++submapIterator) {
            if (!map.isValid(*submapIterator, "elevation")) continue;
            Eigen::Vector3d point;
            map.getPosition3("elevation", *submapIterator, point);
            points.col(nPoints) = point;
            nPoints++;
          }

          const Eigen::Vector3d mean = points.leftCols(nPoints).rowwise().sum() / nPoints;


          double normalX = map.at("normal_x", *iterator);
          double normalY = map.at("normal_y", *iterator);
          double normalZ = map.at("normal_z", *iterator);
          double planeParameter = mean.x()*normalX + mean.y()*normalY + mean.z()*normalZ;
          double sum = 0.0;
          for (int i = 0; i < nPoints; i++) {
            double dist = normalX*points(0,i) + normalY*points(1,i) + normalZ*points(2,i) - planeParameter;
            sum += pow(dist,2);
          }
          double roughness = sqrt(sum / (nPoints -1));

          if (roughness < r_critic) {
            map.at("roughness", *iterator) = 1.0 - roughness / r_critic;
          }
          else {
            map.at("roughness", *iterator) = 0.0;
          }

          if (roughness > roughnessmax) roughnessmax = roughness;
        } 
  scopedLockmap.unlock();
}
void step_filter(GridMap& map,double firstWindowRadius_, double secondWindowRadius_,double nCellCritical_,double h_critic,GridMap input_map)
{
  boost::recursive_mutex::scoped_lock scopedLockmap(rawMapMutex_);
   map=input_map;
   map.add("step_height");
   double height, step;
   for (GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
     if (!map.isValid(*iterator, "elevation")) continue;
     height = map.at("elevation", *iterator);
     double heightMax, heightMin;

     Eigen::Vector2d center;
     map.getPosition(*iterator, center);

     bool init = false;
     for (CircleIterator submapIterator(map, center, firstWindowRadius_);!submapIterator.isPastEnd(); ++submapIterator) {
       if (!map.isValid(*submapIterator, "elevation")) continue;
       height = map.at("elevation", *submapIterator);       
       if (!init) {
         heightMax = height;
         heightMin = height;
         init = true;
         continue;
       }
       if (height > heightMax) heightMax = height;
       if (height < heightMin) heightMin = height;
     }
     if (init)
       map.at("step_height", *iterator) = heightMax - heightMin;
   }

   for (GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
     int nCells = 0;
     double stepMax = 0.0;
     bool isValid = false;

     // Requested position (center) of circle in map.
     Eigen::Vector2d center;
     map.getPosition(*iterator, center);

     // Compute the step height.
     for (CircleIterator submapIterator(map, center, secondWindowRadius_);!submapIterator.isPastEnd(); ++submapIterator) {
       if (!map.isValid(*submapIterator, "step_height")) continue;
       isValid = true;
       if (map.at("step_height", *submapIterator) > stepMax) { stepMax = map.at("step_height", *submapIterator); }
       if (map.at("step_height", *submapIterator) > h_critic) nCells++;
     }

     if (isValid) {
       step = std::min(stepMax, (double) nCells / (double) nCellCritical_ * stepMax);
       if (step < h_critic) {
         map.at("height", *iterator) = 1.0 - step / h_critic;
       } else {
         map.at("height", *iterator) = 0.0;
       }
     }
   }

   map.erase("step_height");
   scopedLockmap.unlock();
}

int main(int argc, char** argv)
{

  // Initialize node and publisher.
  ros::init(argc, argv, "traversability");
  ros::NodeHandle nh;

  ros::Subscriber cloud_ = nh.subscribe<sensor_msgs::PointCloud2>("/gf_cloud",100,gf_cloud_cb);
  ros::Publisher grid_pub = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);


  GridMap grid_map({"elevation","normal_x","normal_y","normal_z","roughness","slope","height","traverse"});
  grid_map.setGeometry(Length(GRID_X_SIZE,GRID_Y_SIZE), 0.03,Position(GRID_X_OFFSET,GRID_Y_OFFSET));
  grid_map.setFrameId("base_link");

  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",grid_map.getLength().x(), grid_map.getLength().y(),grid_map.getSize()(0), grid_map.getSize()(1));

  ros::Rate rate(10.0);
  while (nh.ok()) {
      for(unsigned int i=0; i<cloud->size();++i){
        auto& point = cloud->points[i];
        Index index;
        Position position(point.x,point.y);
        if(!grid_map.getIndex(position, index)) continue;
        auto& elevation = grid_map.at("elevation",index);
        if(!grid_map.isValid(index)){ elevation = point.z;continue;}
      }

      grid_normal_estimation(grid_map,0.06,grid_map);
      slope_filter(grid_map,0.707,grid_map); // 45 degree
      roughness_filter(grid_map,0.3,0.3,grid_map); // 0.3 radius 0.3 std
      step_filter(grid_map,0.09,0.09,5,0.2,grid_map); // first 0.08 second 0.08 cell 5 height 0.3
      cout << "Alg finished" << endl;

      boost::recursive_mutex::scoped_lock scopedLockmap(rawMapMutex_);
      pcl::PointCloud<pcl::PointXYZ>::Ptr grid_points(new pcl::PointCloud<pcl::PointXYZ>);
      for (GridMapIterator it(grid_map); !it.isPastEnd(); ++it) {
        Position3 pos;
        grid_map.getPosition3("elevation",*it,pos);
        pcl::PointXYZ pt(pos.x(),pos.y(),pos.z());
        grid_points->points.push_back(pt);
        }

        grid_points->width = grid_points->points.size();
        grid_points->height = 1;
        grid_points->is_dense = true;
        ros::Time time = ros::Time::now();
          grid_map.setTimestamp(time.toNSec());
          grid_map_msgs::GridMap message;
          GridMapRosConverter::toMessage(grid_map, message);
          grid_pub.publish(message);
          ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
           scopedLockmap.unlock();
    rate.sleep();
    ros::spinOnce();
 }
  return 0;
}
