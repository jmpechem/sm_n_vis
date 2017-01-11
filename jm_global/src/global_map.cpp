#include "jm_global/global_map.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr clouds (new pcl::PointCloud<pcl::PointXYZ>);

Global_Map_Builder::Global_Map_Builder()
  : grid_map_({"elevation","normal_x","normal_y","normal_z","height","slope","edge","global"})
{
    clouds->clear();
    grid_map_.setGeometry(Length(GRID_X_SIZE,GRID_Y_SIZE), GRID_RESOLUTION,Position(GRID_X_OFFSET,GRID_Y_OFFSET));
    grid_map_.setFrameId("base_link");
    grid_pub_ = nh_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    is_first = true;
}

void Global_Map_Builder::PCD2GridMap()
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/jimin/catkin_ws/src/jm_global/pcd_data_set/d2_ds.pcd", *clouds) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read pcd file\n");
    return ;
  }
   ROS_INFO("PCD file read success!!");

   pcl::PointCloud<pcl::PointXYZ>::Ptr assembly (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointXYZ temp;
   for(int i=0;i<clouds->size();i++)
   {
       temp.x = clouds->points[i].x;
       temp.y = clouds->points[i].y;
       temp.z = clouds->points[i].z;
       assembly->push_back(temp);
   }

   pcl::PointXYZ filter_min;
   pcl::PointXYZ filter_max;
   pcl::getMinMax3D(*assembly,filter_min,filter_max);
   pcl::PointCloud<pcl::PointXYZ> final_assembly;
   double z_thres = HEIGHT_LIMIT;
   for(int i=0;i<assembly->size();i++)
   {
           if ( abs(filter_min.z-assembly->points[i].z) <= z_thres)
           {
                   temp.x = assembly->points[i].x;
                   temp.y = assembly->points[i].y;
                   temp.z = assembly->points[i].z;
                   final_assembly.push_back(temp);
           }
   }

   for(unsigned int i=0; i<final_assembly.size();++i){
     auto& point = final_assembly.points[i];
     Index index;
     Position position(point.x,point.y);
     if(!grid_map_.getIndex(position, index)) continue;
     auto& elevation = grid_map_.at("elevation",index);
     if(!grid_map_.isValid(index)){ elevation = point.z;continue;}
   }
   ROS_INFO("Elevation Grid Map Created!!");
}

void Global_Map_Builder::Cloud2GridMap(const sensor_msgs::PointCloud2::ConstPtr &input)
{
    clouds->clear();
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*clouds);
    pcl::PointCloud<pcl::PointXYZ>::Ptr assembly (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ temp;
    for(int i=0;i<clouds->size();i++)
    {
        temp.x = clouds->points[i].x;
        temp.y = clouds->points[i].y;
        temp.z = clouds->points[i].z;
        assembly->push_back(temp);
    }
    pcl::PointXYZ filter_min;
    pcl::PointXYZ filter_max;
    pcl::getMinMax3D(*assembly,filter_min,filter_max);
    pcl::PointCloud<pcl::PointXYZ> final_assembly;
    double z_thres = HEIGHT_LIMIT;
    for(int i=0;i<assembly->size();i++)
    {
            if ( abs(filter_min.z-assembly->points[i].z) <= z_thres)
            {
                    temp.x = assembly->points[i].x;
                    temp.y = assembly->points[i].y;
                    temp.z = assembly->points[i].z;
                    final_assembly.push_back(temp);
            }
    }

    for(unsigned int i=0; i<final_assembly.size();++i){
      auto& point = final_assembly.points[i];
      Index index;
      Position position(point.x,point.y);
      if(!grid_map_.getIndex(position, index)) continue;
      auto& elevation = grid_map_.at("elevation",index);
      if(!grid_map_.isValid(index)){ elevation = point.z;continue;}
    }
    ROS_INFO("Elevation Grid Map Created!!");
}

void Global_Map_Builder::Empty_Grid_Filter(int mask_size,GridMap& output_map, GridMap input_map)
{
  boost::recursive_mutex::scoped_lock scopedLockmap(MapMutex_);
  output_map = input_map;

  for (GridMapIterator it(output_map); !it.isPastEnd(); ++it) {
      if (!output_map.isValid(*it, "elevation"))
      {
          Position hole_pos;
          Index hole_idx;
          output_map.getPosition(*it,hole_pos);
          output_map.getIndex(hole_pos,hole_idx);
          Eigen::Array2i grid_mask(mask_size, mask_size);
          double empty_grid_elevation = 0.0f;
          int    empty_grid_count = 0;
          int    hole_number = 0;
          for(SubmapIterator iter(output_map,hole_idx,grid_mask);!iter.isPastEnd();++iter){
              if(output_map.isValid(*iter,"elevation"))
              {
                  empty_grid_elevation  += output_map.at("elevation",*iter);
                  empty_grid_count++;
              }
              else
              {
                  hole_number++;
              }
          }
              if(hole_number<3)
              output_map.at("elevation",*it) = empty_grid_elevation/((double)empty_grid_count);
       }

  }
  scopedLockmap.unlock();
}

void Global_Map_Builder::Grid_Normal_Estimation(double radius,GridMap& output_map, GridMap input_map)
{
  boost::recursive_mutex::scoped_lock scopedLockmap(MapMutex_);
  output_map = input_map;


    Eigen::Vector3d surfaceNormalPositiveAxis_;
  surfaceNormalPositiveAxis_ = grid_map::Vector3::UnitZ();
  vector<string> surfaceNormalTypes;
  surfaceNormalTypes.push_back("normal_x");
  surfaceNormalTypes.push_back("normal_y");
  surfaceNormalTypes.push_back("normal_z");

  for(GridMapIterator iter(output_map);!iter.isPastEnd();++iter){
    if(!output_map.isValid(*iter,"elevation")) continue;
    if(output_map.isValid(*iter,surfaceNormalTypes)) continue; // normal extraction once...
    Length submapLength = Length::Ones() * (2.0 * radius);
    Position submapPosition;
    output_map.getPosition(*iter,submapPosition);
    Index submapTopLeftIndex, submapBufferSize, requestedIndexInSubmap;
    getSubmapInformation(submapTopLeftIndex, submapBufferSize, submapPosition, submapLength,
                             requestedIndexInSubmap, submapPosition, submapLength,
                             output_map.getLength(), output_map.getPosition(), output_map.getResolution(),
                             output_map.getSize(), output_map.getStartIndex());

    const int maxNumberOfCells = submapBufferSize.prod();
    Eigen::MatrixXd points(3, maxNumberOfCells);

    size_t nPoints = 0;
    size_t Edge_nPoints = 0;
    size_t Not_Edge_nPoints = 0;
    Position3 max_point;
    Position3 min_point;
    Position3 edge_point;
    bool min_max_first = true;
    Position3 start_point;
    output_map.getPosition3("elevation",submapTopLeftIndex,start_point);
       for (SubmapIterator submapIterator(output_map, submapTopLeftIndex, submapBufferSize); !submapIterator.isPastEnd(); ++submapIterator) {

         if (!output_map.isValid(*submapIterator,"elevation")) continue;               
         if(output_map.at("edge",*submapIterator) == 0)
           {
              Edge_nPoints++;
              output_map.getPosition3("elevation",*submapIterator,edge_point);
              if(min_max_first)
              {
                  max_point = edge_point;
                  min_point = edge_point;
                  min_max_first = false;
              }
              if(max_point(2) <= edge_point(2))
              {
                  max_point = edge_point;
              }
              if(min_point(2) >= edge_point(2))
              {
                  min_point = edge_point;
              }
           }
         else
           {
              Not_Edge_nPoints++;
           }
       }
       int min_cnt = 0;
       int max_cnt = 0;
       for (SubmapIterator submapIterator(output_map, submapTopLeftIndex, submapBufferSize); !submapIterator.isPastEnd(); ++submapIterator) {
         if (!output_map.isValid(*submapIterator,"elevation")) continue;
             Position3 point;
             output_map.getPosition3("elevation", *submapIterator, point);
             if( abs(min_point(2) - point(2)) < abs(max_point(2) - point(2)) )
             {
                min_cnt++;
             }
             else if( abs(min_point(2) - point(2)) >= abs(max_point(2) - point(2)) )
             {
                max_cnt++;
             }
       }

       if(Not_Edge_nPoints == submapBufferSize(0)*submapBufferSize(1))
       {
          for (SubmapIterator submapIterator(output_map, submapTopLeftIndex, submapBufferSize); !submapIterator.isPastEnd(); ++submapIterator) {
            if (!output_map.isValid(*submapIterator,"elevation")) continue;
                Position3 point;
                output_map.getPosition3("elevation", *submapIterator, point);
                points.col(nPoints) = point;
                nPoints++;
          }
       }
       else if(Edge_nPoints > 0)
       {
           for (SubmapIterator submapIterator(output_map, submapTopLeftIndex, submapBufferSize); !submapIterator.isPastEnd(); ++submapIterator) {
             if (!output_map.isValid(*submapIterator,"elevation")) continue;
                Position3 point;
                output_map.getPosition3("elevation", *submapIterator, point);
                if( abs(min_point(2) - point(2)) <= GRID_RESOLUTION && min_cnt > max_cnt)
                {
                    points.col(nPoints) = point;
                    nPoints++;
                }
                if( abs(max_point(2) - point(2)) <= GRID_RESOLUTION && min_cnt < max_cnt)
                {
                    points.col(nPoints) = point;
                    nPoints++;
                }
           }
       }
       else
       {
           for (SubmapIterator submapIterator(output_map, submapTopLeftIndex, submapBufferSize); !submapIterator.isPastEnd(); ++submapIterator) {
             if (!output_map.isValid(*submapIterator,"elevation")) continue;
                 Position3 point;
                 output_map.getPosition3("elevation", *submapIterator, point);
                 points.col(nPoints) = point;
                 nPoints++;
           }
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
              output_map.at("normal_x",*iter)=eigenvector.x();
              output_map.at("normal_y",*iter)=eigenvector.y();
              output_map.at("normal_z",*iter)=eigenvector.z();
  }
  scopedLockmap.unlock();
}

void Global_Map_Builder::Global_Slope(double gs_crit,GridMap& output_map, GridMap input_map)
{
  boost::recursive_mutex::scoped_lock scopedLockmap(MapMutex_);
  output_map = input_map;

  double slope, slopeMax = 0.0;
  for (GridMapIterator iter(output_map); !iter.isPastEnd();++iter){
      if (!output_map.isValid(*iter, "normal_z")) continue;
       slope = acos(output_map.at("normal_z", *iter));

       if (slope < gs_crit)
       {
             output_map.at("slope", *iter) = 1.0 - slope / gs_crit;
       }
       else
       {
             output_map.at("slope", *iter) = 0.0;
       }
       if (slope > slopeMax) slopeMax = slope;
    }
  scopedLockmap.unlock();
}
void Global_Map_Builder::Global_Height(double foot_center_z,GridMap& output_map, GridMap input_map)
{
  boost::recursive_mutex::scoped_lock scopedLockmap(MapMutex_);
  output_map = input_map;

  double height, heightMax = 0.0;
  for (GridMapIterator iter(output_map); !iter.isPastEnd();++iter){
      if (!output_map.isValid(*iter, "elevation")) continue;
       height = abs(foot_center_z-output_map.at("elevation",*iter));//acos(grid_map_.at("normal_z", *iter));

       if (height < HEIGHT_THRESHOLD) {
             output_map.at("height", *iter) = 1.0 - height / HEIGHT_THRESHOLD;
           }
           else {
             output_map.at("height", *iter) = 0.0;
           }

           if (height > heightMax) heightMax = height;
    }
  scopedLockmap.unlock();
}
/*void Global_Map_Builder::Global_Roughness(double gr_crit,double r_radius,GridMap& output_map, GridMap input_map)
{
  boost::recursive_mutex::scoped_lock scopedLockmap(MapMutex_);

  double roughnessmax = 0.0;

  for (GridMapIterator iterator(output_map);!iterator.isPastEnd(); ++iterator) {

      if (!output_map.isValid(*iterator, "normal_x")) continue;

          const int maxNumberOfCells = ceil(pow(2*r_radius/output_map.getResolution(),2));
          Eigen::MatrixXd points(3, maxNumberOfCells);

          // Requested position (center) of circle in map.
          Position center;
          output_map.getPosition(*iterator, center);

          // Gather surrounding data.
          size_t nPoints = 0;
          for (CircleIterator submapIterator(output_map, center, r_radius);
              !submapIterator.isPastEnd(); ++submapIterator) {
            if (!output_map.isValid(*submapIterator, "elevation")) continue;
            Eigen::Vector3d point;
            output_map.getPosition3("elevation", *submapIterator, point);
            points.col(nPoints) = point;
            nPoints++;
          }

          const Eigen::Vector3d mean = points.leftCols(nPoints).rowwise().sum() / nPoints;


          double normalX = output_map.at("normal_x", *iterator);
          double normalY = output_map.at("normal_y", *iterator);
          double normalZ = output_map.at("normal_z", *iterator);
          double planeParameter = mean.x()*normalX + mean.y()*normalY + mean.z()*normalZ;
          double sum = 0.0;
          for (int i = 0; i < nPoints; i++) {
            double dist = normalX*points(0,i) + normalY*points(1,i) + normalZ*points(2,i) - planeParameter;
            sum += pow(dist,2);
          }
          double roughness = sqrt(sum / (nPoints -1));

          if (roughness < gr_crit) {
            output_map.at("roughness", *iterator) = 1.0 - roughness / gr_crit;
          }
          else {
            output_map.at("roughness", *iterator) = 0.0;
          }

          if (roughness > roughnessmax) roughnessmax = roughness;
        }
  scopedLockmap.unlock();
}*/
int Global_Map_Builder::roundToInt(double x) {
  if (x >= 0) return (int) (x + 0.5);
  return (int) (x - 0.5);
}

void Global_Map_Builder::Edge_Extraction(GridMap& output_map, GridMap input_map)
{
  boost::recursive_mutex::scoped_lock scopedLockmap(MapMutex_);
  output_map = input_map;

  cv::Mat slopeMat = cv::Mat(300, 300, CV_8UC1, 255);
  cv::Mat heightMat = cv::Mat(300, 300, CV_8UC1, 255);

  for (GridMapIterator iter(output_map); !iter.isPastEnd();++iter)
  {
      if (!output_map.isValid(*iter, "height")) continue;
      Position position;
      Index idx;
      output_map.getPosition(*iter,position);
      output_map.getIndex(position,idx);
      heightMat.at<unsigned char>(idx(0),idx(1)) = roundToInt(255.0*output_map.at("height",*iter));
  }

  namedWindow("height input", CV_WINDOW_AUTOSIZE);
  imshow("height input",heightMat);

  cv::Mat element5(5, 5, CV_8U, cv::Scalar(1));
  cv::Mat closed;
  cv::morphologyEx(heightMat, closed, cv::MORPH_CLOSE, element5);


  // morphology filter open calculation
  cv::Mat opened;
  cv::morphologyEx(heightMat, opened, cv::MORPH_OPEN, element5);

  namedWindow("opened", CV_WINDOW_AUTOSIZE);
  imshow("opened",opened);

  // canny edge detector
 cv::Mat contours, contoursInv;
 cv::Canny(opened,contours,125,350);//125,350;
 cv::threshold(contours, contoursInv, 128, 255, cv::THRESH_BINARY_INV);

 cv::namedWindow("Canny Contours");
 cv::imshow("Canny Contours", contoursInv);

 int edge_img[300 * 300] = {0,};
 int k = 0;
 for(int i=0;i<contoursInv.cols;i++)
   {
     for(int j=0;j<contoursInv.rows;j++)
       {
         edge_img[k] = contoursInv.at<uchar>(j,i);
         k++;
       }
   }

 k=0;

 for (GridMapIterator iter(output_map); !iter.isPastEnd();++iter)
 {
     output_map.at("edge",*iter) = edge_img[k];
     k++;
 }

  cv::waitKey(3);
  scopedLockmap.unlock();
}

void Global_Map_Builder::Global_Cost(double alpha,GridMap& output_map, GridMap input_map)
{
  boost::recursive_mutex::scoped_lock scopedLockmap(MapMutex_);
  output_map = input_map;

  double gcost = 0.0;
  for (GridMapIterator iter(output_map); !iter.isPastEnd();++iter){
      if (!output_map.isValid(*iter, "elevation")) continue;
      if (!output_map.isValid(*iter, "height")) continue;
      if (!output_map.isValid(*iter, "slope")) continue;

       gcost = alpha*output_map.at("height",*iter)+(1-alpha)*output_map.at("slope",*iter);
       output_map.at("global", *iter) = gcost;

    }
  scopedLockmap.unlock();
}

void Global_Map_Builder::Update()
{
  if(clouds->empty())
    {
      PCD2GridMap();
    }
  if(is_first)
  {
      for(int i = 0; i<FILTER_UPDATE_NUM;i++)
      {
          Empty_Grid_Filter(3,grid_map_,grid_map_);
      }
      is_first = false;
  }

  Global_Height(-1.09,grid_map_,grid_map_);
  Edge_Extraction(grid_map_,grid_map_);  
  Grid_Normal_Estimation(GRID_RESOLUTION*CIRCLE_SCALE,grid_map_,grid_map_);
  Global_Slope(deg2rad(23),grid_map_,grid_map_);
  //Global_Roughness(0.02,GRID_RESOLUTION*CIRCLE_SCALE,grid_map_,grid_map_);

  Global_Cost(0.5,grid_map_,grid_map_);

  //boost::recursive_mutex::scoped_lock scopedLockmap(MapMutex_);
  pcl::PointCloud<pcl::PointXYZ>::Ptr grid_points(new pcl::PointCloud<pcl::PointXYZ>);
  for (GridMapIterator it(grid_map_); !it.isPastEnd(); ++it) {
    Position3 pos;
    grid_map_.getPosition3("elevation",*it,pos);
    pcl::PointXYZ pt(pos.x(),pos.y(),pos.z());
    grid_points->points.push_back(pt);
    }
    grid_points->width = grid_points->points.size();
    grid_points->height = 1;
    grid_points->is_dense = true;
    ros::Time time = ros::Time::now();
    grid_map_.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(grid_map_, message);
    grid_pub_.publish(message);
    ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
  // scopedLockmap.unlock();
}
