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

#include <string>
#include <vector>

#include "jm_map/slic.h"
#include "jm_map/egbis.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace grid_map;
using namespace cv;

#define PI 3.14159265359
#define SLOPE_THRES   25
#define STEP_THRES    0.25
#define deg2rad(deg)  ((deg) * PI / 180.0)
#define rad2deg(rad)  ((deg) * 180.0 / PI)
#define GRID_X_SIZE   9 // 10
#define GRID_Y_SIZE   9 // 10
#define GRID_X_OFFSET 4.5 // 5
#define GRID_Y_OFFSET 0 //
#define GRID_RESOLUTION 0.03 // 0.05
#define ROBOT_FOOTPRINT_SIZE   0.5
#define FLAT_PLANE_THRESHOLD   0.1
#define CIRCLE_SCALE           2


 boost::recursive_mutex rawMapMutex_;
 boost::recursive_mutex markerMutex_;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

void gf_cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& gf_cloud){
    cloud->clear();
     pcl::PCLPointCloud2 pcl_pc2;
     pcl_conversions::toPCL(*gf_cloud,pcl_pc2);
     pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
}

/*
ros::Publisher markers_pub;
bool b_is_marker_pub = false;
visualization_msgs::Marker build_marker_map(float x,float y,float z,float r,float g, float b, int index){

  visualization_msgs::Marker marker_data;
  marker_data.header.frame_id = "base_link";
  marker_data.header.stamp = ros::Time::now();
  marker_data.ns = "map_namespace";
  marker_data.id = index;
  marker_data.type = visualization_msgs::Marker::CUBE;
  marker_data.action = visualization_msgs::Marker::ADD;
  //marker_data.lifetime = ros::Duration(1);
  marker_data.pose.position.x = x;
  marker_data.pose.position.y = y;
  marker_data.pose.position.z = z;
  marker_data.pose.orientation.x = 0;
  marker_data.pose.orientation.y = 0;
  marker_data.pose.orientation.z = 0;
  marker_data.pose.orientation.w = 1;
  marker_data.scale.x = 0.05;
  marker_data.scale.y = 0.05;
  marker_data.scale.z = 0.05;
  marker_data.color.a = 1.0;
  marker_data.color.r = r;
  marker_data.color.g = g;
  marker_data.color.b = b;


  return marker_data;

}*/
/*
void release_marker_map(int index)
{
  visualization_msgs::Marker marker_data;
  marker_data.header.frame_id = "base_link";
  marker_data.header.stamp = ros::Time();
  marker_data.ns = "map_namespace";
  marker_data.id = index;
  marker_data.type = visualization_msgs::Marker::CUBE;
  marker_data.action = visualization_msgs::Marker::DELETE;
  remove_markers.markers.push_back(marker_data);
}*/
void empty_grid_filter(GridMap& map, GridMap input_map)
{
  boost::recursive_mutex::scoped_lock scopedLockmap(rawMapMutex_);
  map=input_map;
  for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
      if (!map.isValid(*it, "elevation"))
      {
          Position hole_pos;
          Index idx;
          map.getPosition(*it,hole_pos);
          map.getIndex(hole_pos,idx);
//          for(SubmapIterator iter(map,idx,GRID_RESOLUTION);!iter.isPastEnd();++iter){

//          }

        //map.at("elevation",*it) =
      }

/*      Position position;
      Index idx;
      map.getPosition(*it,position);
      map.getIndex(position,idx);
      myMat.at<cv::Vec3b>(idx(1),idx(0))[0] = 255-roundToInt(255.0*map.at("roughness",*it));
      myMat.at<cv::Vec3b>(idx(1),idx(0))[1] = 255-roundToInt(255.0*map.at("slope",*it));
      myMat.at<cv::Vec3b>(idx(1),idx(0))[2] = 255-roundToInt(255.0*map.at("height",*it));

      roughMat.at<unsigned char>(idx(1),idx(0)) = 255-roundToInt(255.0*map.at("roughness",*it));
      slopeMat.at<unsigned char>(idx(1),idx(0)) = 255-roundToInt(255.0*map.at("slope",*it));
      heightMat.at<unsigned char>(idx(1),idx(0)) = 255-roundToInt(255.0*map.at("height",*it));*/
  }


  scopedLockmap.unlock();
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
void traversability_filter(GridMap& map,double w_roughness, double w_slope,double w_height,double t_critic, GridMap input_map)
{

  boost::recursive_mutex::scoped_lock scopedLockmap(rawMapMutex_);
  map=input_map;
  double t_factor = 0.0f;
  for (GridMapIterator iter(map); !iter.isPastEnd();++iter){
      if (!map.isValid(*iter, "roughness")) continue;
      if (!map.isValid(*iter, "slope")) continue;
      if (!map.isValid(*iter, "height")) continue;
      //if (!map.isValid(*iter,"elevation")) continue;
       t_factor =  w_roughness * map.at("roughness",*iter) + w_slope * map.at("slope",*iter) + w_height * map.at("height",*iter) - (w_roughness + w_slope + w_height) + 1;
       map.at("traverse", *iter) = t_factor;

              if (t_factor >= t_critic) {
            map.at("t_points",*iter) = map.at("elevation",*iter);
           }
           else {
            map.at("t_points",*iter) = NAN;
//             map.at("traverse", *iter) = 0.0;
           }

   //        if (slope > slopeMax) slopeMax = slope;
  }
  scopedLockmap.unlock();
}

/*
class WatershedSegmenter {
private:
cv::Mat markers;

public:
void setMarkers(const cv::Mat& markerImage) {
 markerImage.convertTo(markers,CV_32S);
  // 정수형 영상 변환
}

cv::Mat process(const cv::Mat &image) {
 cv::watershed(image,markers);
  // 워터쉐드 적용
 return markers;
 }

 cv::Mat getSegmentation() { // 영상 형태인 결과를 반환
 cv::Mat tmp;
 markers.convertTo(tmp,CV_8U);
  // 255 이상인 레이블을 갖는 모든 분할은 255인 값으로 할당
 return tmp;
 }

 cv::Mat getWatersheds() { // 영상 형태인 워터쉐드를 반환
 cv::Mat tmp;
 markers.convertTo(tmp,CV_8U,255,255);
 return tmp;
 }
};*/

int cost_map_data[200*200] = {0,};
int r_factor[200*200] = {0,};
int s_factor[200*200] = {0,};
int h_factor[200*200] = {0,};
int edge_factor[200*200] = {0,};

int roundToInt(double x) {
  if (x >= 0) return (int) (x + 0.5);
  return (int) (x - 0.5);
}
double norm_dist_cell(double x,double y, double cell_x, double cell_y)
{
  return sqrt( (x-cell_x)*(x-cell_x) + (y-cell_y)*(y-cell_y) );
}

void obstacle_classifier(GridMap& map, GridMap input_map)
{
  boost::recursive_mutex::scoped_lock scopedLockmap(rawMapMutex_);
  map=input_map;

  double dNaN = std::numeric_limits<double>::quiet_NaN();
  cv::Mat myMat = cv::Mat(200, 200, CV_8UC3, cv::Scalar(255,255,255));


  cv::Mat roughMat = cv::Mat(200,200,CV_8UC1,255);
  cv::Mat slopeMat = cv::Mat(200,200,CV_8UC1,255);
  cv::Mat heightMat = cv::Mat(200,200,CV_8UC1,255);

  for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
      if (!map.isValid(*it, "traverse")) continue;
      if (!map.isValid(*it, "roughness")) continue;
      if (!map.isValid(*it, "slope")) continue;
      if (!map.isValid(*it, "height")) continue;
      Position position;
      Index idx;
      map.getPosition(*it,position);
      map.getIndex(position,idx);
      myMat.at<cv::Vec3b>(idx(1),idx(0))[0] = 255-roundToInt(255.0*map.at("roughness",*it));
      myMat.at<cv::Vec3b>(idx(1),idx(0))[1] = 255-roundToInt(255.0*map.at("slope",*it));
      myMat.at<cv::Vec3b>(idx(1),idx(0))[2] = 255-roundToInt(255.0*map.at("height",*it));

      roughMat.at<unsigned char>(idx(1),idx(0)) = 255-roundToInt(255.0*map.at("roughness",*it));
      slopeMat.at<unsigned char>(idx(1),idx(0)) = 255-roundToInt(255.0*map.at("slope",*it));
      heightMat.at<unsigned char>(idx(1),idx(0)) = 255-roundToInt(255.0*map.at("height",*it));
  }

  namedWindow("raw input", CV_WINDOW_AUTOSIZE);
  imshow("raw input",myMat);

  namedWindow("rough",CV_WINDOW_AUTOSIZE);
  imshow("rough",roughMat);
  namedWindow("slope",CV_WINDOW_AUTOSIZE);
  imshow("slope",slopeMat);
  namedWindow("height",CV_WINDOW_AUTOSIZE);
  imshow("height",heightMat);

  // 닫힘 연산자 적
  cv::Mat element5(5, 5, CV_8U, cv::Scalar(1)); // 5,5 for lobby, 9,9 for slope
  cv::Mat closed;
  cv::morphologyEx(myMat, closed, cv::MORPH_CLOSE, element5);


  // 열림 연산자 적용
  cv::Mat opened;
  cv::morphologyEx(myMat, opened, cv::MORPH_OPEN, element5);

//  namedWindow("closed", CV_WINDOW_AUTOSIZE);
//  imshow("closed",closed);
 //cv::Mat closed = myMat;

  cv::Mat lab_closed;
  cv::cvtColor(closed,lab_closed,CV_BGR2Lab);
  int w = closed.rows, h = closed.cols;
  //int nr_superpixels = 100;
  //int nc = 20;

  int nr_superpixels = 90;
  int nc = 40;

  double step = sqrt((w * h) / (double) nr_superpixels);

  IplImage *lab_image = new IplImage(lab_closed);
  IplImage *closed_image = new IplImage(closed);
/*
  Mat egbisImage;
  int num_ccs;
  egbisImage = runEgbisOnMat(closed, 0.5, 500, 100, &num_ccs);
  namedWindow( "graph" , CV_WINDOW_AUTOSIZE );
  imshow( "graph" , egbisImage );
  */

  Slic slic;
  slic.generate_superpixels(lab_image, step, nc);
  slic.create_connectivity(lab_image);


  slic.colour_with_cluster_means(closed_image);
  //slic.display_contours(closed_image, CV_RGB(255,0,0));
  //slic.display_center_grid(closed_image,CV_RGB(0,0,255));
    int num = slic.get_center_num(closed_image);
    vector<int> region;
    vector<CvPoint> center_pixel;
    //vector<int> blue;
    //vector<int> green;
    //vector<int> red;
    center_pixel.clear();
    region.clear();
    CvPoint test;
    for(int i=0;i<num;i++)
      {
          test = slic.get_center_grid(closed_image,i);
          CvScalar s;
          s=cvGet2D(closed_image,test.x,test.y);

          int nB = closed.at<Vec3b>(test.x, test.y)[0];
          int nG = closed.at<Vec3b>(test.x, test.y)[1];
          int nR = closed.at<Vec3b>(test.x, test.y)[2];

      }


  cv::namedWindow("superpixel raw");
  cv::imshow("superpixel raw",closed);



  cv::Mat grayImage;
  cv::cvtColor(closed,grayImage,CV_RGB2GRAY);
  //cv::namedWindow("Gray");
  //cv::imshow("Gray", grayImage);
  cv::Mat contour;
  cv::Canny(grayImage,contour,120,250);
  cv::Mat contoursInv; // 반전 영상
  cv::threshold(contour, contoursInv, 128, 255, cv::THRESH_BINARY_INV);
  //cv::threshold(contour, contoursInv, 0, 128, cv::THRESH_BINARY_INV);
 cv::namedWindow("Canny Contours");
 cv::imshow("Canny Contours", contoursInv);


  //cv::waitKey(3);

  // graph based segmentation


  /*
  cv::Mat grayImage;
  cv::cvtColor(closed,grayImage,CV_RGB2GRAY);
  //cv::namedWindow("Gray");
  //cv::imshow("Gray", grayImage);
  cv::Mat contour;
  cv::Canny(grayImage,contour,20,250);
  cv::Mat contoursInv; // 반전 영상
  cv::threshold(contour, contoursInv, 128, 255, cv::THRESH_BINARY_INV);
  //cv::threshold(contour, contoursInv, 0, 128, cv::THRESH_BINARY_INV);
 cv::namedWindow("Canny Contours");
 cv::imshow("Canny Contours", contoursInv);
*/
 /*
 // contour extraction
 vector<vector<Point> > contours;
 vector<Vec4i> hierarchy;
 RNG rng(12345);
 findContours( contour, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
 Mat drawing = Mat::zeros( contour.size(), CV_8UC3 );
 cout << contours.size() << endl;
   for( int i = 0; i< contours.size(); i++ )
      {
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
      }
   namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
    imshow( "Contours", drawing );
 cv::waitKey(3);
*/
 // 채널 분리
 /*cv::Mat channel[3];
 split(opened,channel);

 cv::namedWindow("r");
 cv::namedWindow("g");
 cv::namedWindow("b");
 imshow("r",channel[0]);
 imshow("g",channel[1]);
 imshow("b",channel[2]);

 cv::namedWindow("original");
 imshow("original",myMat);

 cv::namedWindow("filtered");
 imshow("filtered",closed);

 cv::waitKey(3);*/
/*
 cv::Mat channel[3];
 split(closed,channel);

 cv::Mat contoursR;
 cv::Canny(channel[0],contoursR,0,100);
 cv::Mat contoursInvR; // 반전 영상
 cv::threshold(contoursR, contoursInvR, 0, 128, cv::THRESH_BINARY_INV);
 cv::namedWindow("Contours");
 cv::imshow("R Contours", contoursInvR);

 cv::Mat contoursG;
 cv::Canny(channel[1],contoursG,0,100);
 cv::Mat contoursInvG; // 반전 영상
 cv::threshold(contoursG, contoursInvG, 0, 128, cv::THRESH_BINARY_INV);
 cv::namedWindow("G Contours");
 cv::imshow("G Contours", contoursInvG);

 cv::Mat contoursB;
 cv::Canny(channel[2],contoursB,0,100);
 cv::Mat contoursInvB; // 반전 영상
 cv::threshold(contoursB, contoursInvB, 0, 128, cv::THRESH_BINARY_INV);
 cv::namedWindow("B Contours");
 cv::imshow("B Contours", contoursInvB);
 cv::waitKey(3);*/
/*
  cv::Mat grayImage;
  cv::cvtColor(closed,grayImage,CV_RGB2GRAY);
  cv::Mat binary;
  cv::threshold(grayImage,binary,100,255,cv::THRESH_BINARY_INV);
  // 이진 영상은 영상의 여러 부분에 속하는 흰색 화소를 많이 포함
 // 중요한 객체에 속하는 화소만 남기기 위해 영상을 여러 번 침식
  cv::namedWindow("gray Image");
  cv::imshow("gray Image", grayImage);
  cv::namedWindow("binary Image");
  cv::imshow("binary Image", binary);
  cv::waitKey(3);*/
/*
 cv::Mat fg;
 cv::erode(binary, fg, cv::Mat(), cv::Point(-1, -1), 6);
  // 잡음과 작은 객체 제거
 cv::namedWindow("Foreground Image");
 cv::imshow("Foreground Image", fg);
  // 배경인 숲에 속하는 몇 화소는 여전히 존재
  // 관심 객체에 대응하는 부분을 고려

 // 원 이진 영상의 큰 팽창인 배경의 화소를 선택
 cv::Mat bg;
 cv::dilate(binary,bg,cv::Mat(),cv::Point(-1,-1),6);
 cv::threshold(bg,bg,1,128,cv::THRESH_BINARY_INV);
  // 객체 없는 영상 화소 식별
 cv::namedWindow("Background Image");
 cv::imshow("Background Image",bg);
  // 결과인 검은 화소는 배경 화소와 일치
  // 255 레이블인 전경화소와 128 레이블인 배경화소를 마크
  // 팽창 시 128 값을 화소에 할당한 후 즉시 경계화 작업

 // 영상을 마커 영상과 조합
 cv::Mat markers(binary.size(),CV_8U,cv::Scalar(0));
 markers= fg+bg; // 마커 영상 생성
  // 영상 조합시 오버로드 된 operator+ 사용
 cv::namedWindow("Markers");
 cv::imshow("Markers",markers);
  // 워터쉐드 알고리즘에 입력하기 위해 사용하는 입력 영상

 // 영상 분할
 WatershedSegmenter segmenter; // 웨터쉐드 분할 객체 생성

 segmenter.setMarkers(markers);
 segmenter.process(opened);
  // 마커를 설정한 후 처리
 cv::namedWindow("Segmentation"); // 분할 결과 띄워 보기
  cv::imshow("Segmentation",segmenter.getSegmentation());

  cv::namedWindow("Watersheds"); // 워터쉐드 띄워 보기
  cv::imshow("Watersheds",segmenter.getWatersheds());
*/

  //cout<< "closed : " << closed.size() << "opend : " << opened.size() << endl;




  cv::namedWindow("Closed Image");
  cv::imshow("Closed Image", closed);
  cv::waitKey(3);

  int nBlue;
  int nGreen;
  int nRed;
  int l=0;


  for(int i = 0; i < closed.rows; i++)
  {
      for(int j = 0; j < closed.cols; j++)
      {
          nBlue = closed.at<Vec3b>(i, j)[0];
          nGreen = closed.at<Vec3b>(i, j)[1];
          nRed = closed.at<Vec3b>(i, j)[2];
          r_factor[l] = nBlue;
          s_factor[l] = nGreen;
          h_factor[l] = nRed;
          cost_map_data[l] = roundToInt((r_factor[l]+s_factor[l]+h_factor[l])/3);
          edge_factor[l] = (int)(contoursInv.at<uchar>(i,j));
          l++;
      }
  }

  int k=0;

  /*float z_min = 0.0;
  float z_max = 0.0;
  float z_thres = 0.2;
  float z_cost = 0.0;
  for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
      if(!map.isValid(*it,"elevation")){ continue;}
      float z_buf = map.at("elevation",*it);
      if(z_buf <= z_min){z_min = z_buf;}
      if(z_buf >= z_max){z_max = z_buf;}
  }*/

  for(GridMapIterator it(map); !it.isPastEnd(); ++it){

      if(cost_map_data[k] >= 255)
      {
          map.at("costmap",*it) = NAN;
          map.at("cost_elevation",*it) = NAN;
          map.at("plan",*it) = NAN;
      }
      else
      {
          map.at("costmap",*it) = cost_map_data[k];
          //cout << "cost : " << cost_map_data[k] << endl;
          map.at("cost_elevation",*it) = map.at("elevation",*it);
          map.at("plan",*it) = map.at("costmap",*it);
          /*if(map.isValid(*it,"cost_elevation"))
          { z_cost = map.at("cost_elevation",*it)/z_min;
          int height_cost = roundToInt(255*(1-z_cost));
          cout << "h cost : " << height_cost << endl;
          map.at("h_cost",*it) = height_cost;}*/


          //if(map.at("plan",*it) >= user_obstacle_threshold){} // flat plane
          //else {map.at("obstacle",*it) = map.at("plan",*it);} // diffrence plane

        }
      if(edge_factor[k]!=255){map.at("edge",*it) = 255-edge_factor[k];}
      k++;
    }  
  cout << "err2" << endl;
  for(GridMapIterator it(map);!it.isPastEnd(); ++it){
    if(!map.isValid(*it,"edge")){continue;}
    else{
          Eigen::Vector2d edge_point;
          map.getPosition(*it,edge_point);
          int cost_cnt = 0;
          int cost_circle_total = 0;
          for(CircleIterator submapIterator(map,edge_point,ROBOT_FOOTPRINT_SIZE);!submapIterator.isPastEnd();++submapIterator){
              if(!map.isValid(*submapIterator,"costmap")){
                  //cost_circle_total += 255;
                  //cost_cnt++;
                }
              else{
                  cost_circle_total += map.at("costmap",*submapIterator);
                  cost_cnt++;
                }

          }
          //cout << "edge footprint cost : " << cost_circle_total/cost_cnt << endl;
          map.at("h_cost",*it) = cost_circle_total/cost_cnt;
        }
    }

  /*for(GridMapIterator it(map);!it.isPastEnd(); ++it){
      Eigen::Vector2d edge_point;
      map.getPosition(*it,edge_point);
      for(CircleIterator submapIterator(map,edge_point,ROBOT_FOOTPRINT_SIZE);!submapIterator.isPastEnd();++submapIterator){

          map.at("boundary",*submapIterator) = map.at("h_cost",*it);

        }
    }*/
cout << "err3" << endl;
  for(GridMapIterator it(map);!it.isPastEnd(); ++it){
      if(!map.isValid(*it,"edge")){}
      else {
      Eigen::Vector2d center;
      map.getPosition(*it,center);
        for(CircleIterator submapIterator(map,center,ROBOT_FOOTPRINT_SIZE);!submapIterator.isPastEnd();++submapIterator){
            if(map.isValid(*submapIterator,"edge")){}
            else {
                Eigen::Vector2d cells;
                map.getPosition(*submapIterator,cells);
                double dist = norm_dist_cell(center(0),center(1),cells(0),cells(1));
                double weight = dist/ROBOT_FOOTPRINT_SIZE;
                map.at("boundary",*submapIterator) = map.at("h_cost",*it)*(1-exp(-1.0 * weight));//map.at("costmap",*it)*(1-exp(-1.0 * weight));
                 }
             }
            }
    }
  /*for(GridMapIterator it(map);!it.isPastEnd(); ++it){
      if(!map.isValid(*it,"boundary")){continue;}
      map.at("cost_elevation",*it) = map.at("boundary",*it);
    }
  for(GridMapIterator it(map);!it.isPastEnd(); ++it){
      if(!map.isValid(*it,"h_cost")){continue;}
      map.at("cost_elevation",*it) = map.at("h_cost",*it);
    }
  for(GridMapIterator it(map);!it.isPastEnd(); ++it){
      if(!map.isValid(*it,"costmap")){map.at("cost_elevation",*it) = NAN;}

    }*/

  for(GridMapIterator it(map);!it.isPastEnd(); ++it){
      if(!map.isValid(*it,"plan")){continue;}
      if(!map.isValid(*it,"boundary")){continue;}
        map.at("plan",*it) = map.at("plan",*it) + map.at("boundary",*it);
         //map.at("plan",*it) = map.at("elevation",*it);
        //map.at("plan",*it) = map.at("costmap",*it);

    }


  //slic.display_contours(closed_image, CV_RGB(255,0,0));
//  cv::namedWindow("test");
//  cv::imshow("test",closed);

/*
  int k=0;
  for(GridMapIterator it(map); !it.isPastEnd(); ++it){
      if(cost_map_data[k] <= 0.0001)
      {
          map.at("costmap",*it) = NAN;
          map.at("cost_elevation",*it) = NAN;
          map.at("plan",*it) = NAN;
      }
      else
      {
          map.at("costmap",*it) = 1-cost_map_data[k];          
          map.at("cost_elevation",*it) = map.at("elevation",*it);
          map.at("plan",*it) = map.at("costmap",*it);
          if(map.at("plan",*it) < 0.1){} // flat plane
          else {map.at("obstacle",*it) = map.at("plan",*it);} // diffrence plane
        }
      k++;
    }
  for(GridMapIterator it(map);!it.isPastEnd(); ++it){
      if(!map.isValid(*it,"obstacle")){}
      else {
      Eigen::Vector2d center;
      map.getPosition(*it,center);

        for(CircleIterator submapIterator(map,center,ROBOT_FOOTPRINT_SIZE);!submapIterator.isPastEnd();++submapIterator){
            if(map.isValid(*submapIterator,"obstacle")){}
            else {
                Eigen::Vector2d cells;
                map.getPosition(*submapIterator,cells);
                double dist = norm_dist_cell(center(0),center(1),cells(0),cells(1));
                double weight = dist/ROBOT_FOOTPRINT_SIZE;
                map.at("boundary",*submapIterator) = 1-exp(-1.0 * weight);
                 }
             }
            }
    }
  for(GridMapIterator it(map);!it.isPastEnd(); ++it){
      if(!map.isValid(*it,"plan")){continue;}
      if(map.isValid(*it,"boundary")){
          map.at("plan",*it) = map.at("boundary",*it);
        }
    }
*/
  scopedLockmap.unlock();
}
/*
void marker_map_generator(GridMap map)
{
  boost::recursive_mutex::scoped_lock scopedLockMarker(markerMutex_);

  visualization_msgs::Marker marker_data;
  marker_data.header.frame_id = "base_link";
  marker_data.type = visualization_msgs::Marker::CUBE;
  marker_data.ns = "map_namespace";

  visualization_msgs::MarkerArray mark;

  int remove_idx = 0;
  for(GridMapIterator it(map); !it.isPastEnd(); ++it){
      marker_data.header.stamp = ros::Time::now();
      marker_data.id = remove_idx;
      marker_data.action = visualization_msgs::Marker::DELETE;
      mark.markers.push_back(marker_data);
      remove_idx++;
    }
    markers_pub.publish(mark);
    mark.markers.clear();
    sleep(1);
  int pos_idx = 0;
  for(GridMapIterator it(map); !it.isPastEnd(); ++it){
      Position3 xyz;
      map.getPosition3("cost_elevation",*it,xyz);
      marker_data.header.stamp = ros::Time::now();
      marker_data.id = pos_idx;
      marker_data.action = visualization_msgs::Marker::ADD;
      marker_data.pose.position.x = xyz(0);
      marker_data.pose.position.y = xyz(1);
      marker_data.pose.position.z = xyz(2);
      marker_data.pose.orientation.x = 0;
      marker_data.pose.orientation.y = 0;
      marker_data.pose.orientation.z = 0;
      marker_data.pose.orientation.w = 1;
      marker_data.scale.x = 0.05;
      marker_data.scale.y = 0.05;
      marker_data.scale.z = 0.05;
      marker_data.color.a = 1.0;
      marker_data.color.r = (float)h_factor[pos_idx]/255.0;
      marker_data.color.g = (float)s_factor[pos_idx]/255.0;
      marker_data.color.b = (float)r_factor[pos_idx]/255.0;

      mark.markers.push_back(marker_data);
      pos_idx++;
    }
    markers_pub.publish(mark);
    mark.markers.clear();

  scopedLockMarker.unlock();
}
*/
int main(int argc, char** argv)
{

  // Initialize node and publisher.
  ros::init(argc, argv, "traversability");
  ros::NodeHandle nh;

  ros::Subscriber cloud_ = nh.subscribe<sensor_msgs::PointCloud2>("/gf_cloud",1000,gf_cloud_cb);
  ros::Publisher grid_pub = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  //markers_pub = nh.advertise<visualization_msgs::MarkerArray>("markers",1,true);

  GridMap grid_map({"elevation","normal_x","normal_y","normal_z","roughness","slope","height","traverse","costmap","cost_elevation","plan","boundary","obstacle","h_cost","edge","t_points"});
  grid_map.setGeometry(Length(GRID_X_SIZE,GRID_Y_SIZE), GRID_RESOLUTION,Position(GRID_X_OFFSET,GRID_Y_OFFSET));
  grid_map.setFrameId("base_link");

  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",grid_map.getLength().x(), grid_map.getLength().y(),grid_map.getSize()(0), grid_map.getSize()(1));

  ros::Rate rate(5.0);
  while (nh.ok()) {
      ros::Time tick = ros::Time::now();
      for(unsigned int i=0; i<cloud->size();++i){
        auto& point = cloud->points[i];
        Index index;
        Position position(point.x,point.y);
        if(!grid_map.getIndex(position, index)) continue;
        auto& elevation = grid_map.at("elevation",index);
        if(!grid_map.isValid(index)){ elevation = point.z;continue;}
      }

      grid_normal_estimation(grid_map,GRID_RESOLUTION*CIRCLE_SCALE,grid_map);
      slope_filter(grid_map,deg2rad(SLOPE_THRES),grid_map); // 45 degree
      roughness_filter(grid_map,GRID_RESOLUTION*CIRCLE_SCALE,0.3,grid_map); // 0.2 radius 0.1 std
      step_filter(grid_map,GRID_RESOLUTION*CIRCLE_SCALE,GRID_RESOLUTION*CIRCLE_SCALE,(CIRCLE_SCALE*2+1)*(CIRCLE_SCALE*2+1)-4,STEP_THRES,grid_map); // first 0.05 second 0.05 cell 5*5-4 height 0.25
      traversability_filter(grid_map,0.0f,0.5f,0.5,0.3,grid_map); // weight(roughness, slope, height) traversabilty threshold 0.5
//      obstacle_classifier(grid_map,grid_map);
      //if(!b_is_marker_pub)
      //{
      //    marker_map_generator(grid_map);
      //    b_is_marker_pub = true;
      //}
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
           //markers_pub.publish(map_markers);
           //markers_pub.publish(remove_markers);
        ros::Time tock = ros::Time::now();
        cout << "clock "<< tick-tock << endl;
    rate.sleep();
    ros::spinOnce();
 }
  return 0;
}
