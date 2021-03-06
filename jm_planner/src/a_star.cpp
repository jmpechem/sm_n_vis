#include "jm_planner/stlastar.h"

#include <ros/ros.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

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

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>

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
#include <std_msgs/Int32MultiArray.h>

#include "jm_planner/tinysplinecpp.h"

using namespace std;
using namespace grid_map;
using namespace cv;

#define PI 3.14159265359
#define deg2rad(deg)  ((deg) * PI / 180.0)
#define rad2deg(rad)  ((deg) * 180.0 / PI)
#define MAX_X 10
#define MAX_Y 10


const int MAP_WIDTH = 200;
const int MAP_HEIGHT = 200;

int world_map[ MAP_WIDTH * MAP_HEIGHT ] = {1,};
std_msgs::Int32MultiArray root_arr;
/*int world_map[ MAP_WIDTH * MAP_HEIGHT ] =
{

	// 0001020304050607080910111213141516171819
	1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,   // 00
	1,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,1,   // 01
	1,9,9,1,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 02
	1,9,9,1,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 03
	1,9,1,1,1,1,9,9,1,9,1,9,1,1,1,1,9,9,1,1,   // 04
	1,9,1,1,9,1,1,1,1,9,1,1,1,1,9,1,1,1,1,1,   // 05
	1,9,9,9,9,1,1,1,1,1,1,9,9,9,9,1,1,1,1,1,   // 06
	1,9,9,9,9,9,9,9,9,1,1,1,9,9,9,9,9,9,9,1,   // 07
	1,9,1,1,1,1,1,1,1,1,1,9,1,1,1,1,1,1,1,1,   // 08
	1,9,1,9,9,9,9,9,9,9,1,1,9,9,9,9,9,9,9,1,   // 09
	1,9,1,1,1,1,9,1,1,9,1,1,1,1,1,1,1,1,1,1,   // 10
	1,9,9,9,9,9,1,9,1,9,1,9,9,9,9,9,1,1,1,1,   // 11
	1,9,1,9,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 12
	1,9,1,9,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 13
	1,9,1,1,1,1,9,9,1,9,1,9,1,1,1,1,9,9,1,1,   // 14
	1,9,1,1,9,1,1,1,1,9,1,1,1,1,9,1,1,1,1,1,   // 15
	1,9,9,9,9,1,1,1,1,1,1,9,9,9,9,1,1,1,1,1,   // 16
	1,1,9,9,9,9,9,9,9,1,1,1,9,9,9,1,9,9,9,9,   // 17
	1,9,1,1,1,1,1,1,1,1,1,9,1,1,1,1,1,1,1,1,   // 18
	1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,   // 19

};
*/
int roundToInt(double x) {
  if (x >= 0) return (int) (x + 0.5);
  return (int) (x - 0.5);
}
int GetMap( int x, int y )
{
	if( x < 0 || x >= MAP_WIDTH || y < 0 || y >= MAP_HEIGHT)
	{
		return 255;
	}

	return world_map[(y*MAP_WIDTH)+x];
}

class MapSearchNode
{
public:
	int x;
	int y;	
	int z;
	MapSearchNode() { x = y = z= 0; }
	MapSearchNode( int px, int py ) { x=px; y=py; }
	float GoalDistanceEstimate( MapSearchNode &nodeGoal );
	bool IsGoal( MapSearchNode &nodeGoal );
	bool GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
	float GetCost( MapSearchNode &successor );
	bool IsSameState( MapSearchNode &rhs );
	void PrintNodeInfo(); 
};

bool MapSearchNode::IsSameState( MapSearchNode &rhs )
{
	if( (x == rhs.x) && (y == rhs.y) ){ return true;}
	else{ return false;}
}

void MapSearchNode::PrintNodeInfo()
{
	char str[100];
	sprintf( str, "Node position : (%d,%d)\n", x,y );
	cout << str;
}
float MapSearchNode::GoalDistanceEstimate( MapSearchNode &nodeGoal )
{	
	float dist = 0.0f;
	dist = sqrt((float)((x - nodeGoal.x)*(x - nodeGoal.x)+(y - nodeGoal.y)*(y - nodeGoal.y)));
	return dist;
}

bool MapSearchNode::IsGoal( MapSearchNode &nodeGoal )
{

	if( (x == nodeGoal.x) && (y == nodeGoal.y) ){ return true;  }
	return false;
}
bool MapSearchNode::GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node )
{

	int parent_x = -1; 
	int parent_y = -1; 
	if( parent_node )
	{
		parent_x = parent_node->x;
		parent_y = parent_node->y;
	}
	MapSearchNode NewNode;
	if((GetMap( x-1, y ) < 255) && !((parent_x == x-1) && (parent_y == y)))
	{
		NewNode = MapSearchNode( x-1, y );
		astarsearch->AddSuccessor( NewNode );
	}	

	if((GetMap( x, y-1 ) < 255) && !((parent_x == x) && (parent_y == y-1)))
	{
		NewNode = MapSearchNode( x, y-1 );
		astarsearch->AddSuccessor( NewNode );
	}	
	if((GetMap( x+1, y ) < 255) && !((parent_x == x+1) && (parent_y == y)))
	{
		NewNode = MapSearchNode( x+1, y );
		astarsearch->AddSuccessor( NewNode );
	}	
	if((GetMap( x, y+1 ) < 255) && !((parent_x == x) && (parent_y == y+1)))
	{
		NewNode = MapSearchNode( x, y+1 );
		astarsearch->AddSuccessor( NewNode );
	}	

	return true;
}
float MapSearchNode::GetCost( MapSearchNode &successor )
{
	return (float) GetMap( x, y );

}

GridMap map_data;
bool publish_path = false;
bool do_local_planning = false;
bool first_time_running = true;
nav_msgs::Path a_star_path;
nav_msgs::Path b_spline_path;
std::vector<geometry_msgs::PoseStamped> a_star_plan;
std::vector<geometry_msgs::PoseStamped> b_spline_plan;
geometry_msgs::PoseStamped astar_root_pose;
geometry_msgs::PoseStamped user_goal_pose;
geometry_msgs::PoseStamped b_spline_pose;
float norm_dist(float grid_x,float grid_y,float user_x,float user_y)
{

  return sqrt((float)((grid_x - user_x)*(grid_x - user_x)+(grid_y - user_y)*(grid_y - user_y)));

}

void map_cb(const grid_map_msgs::GridMap& map_input){

  grid_map::GridMapRosConverter::fromMessage(map_input,map_data);
  /*if(first_time_running)
  {
      map_data.add("plan");
      first_time_running = false;
  }*/

  if(do_local_planning)
  {
      ros::Time tick = ros::Time::now();

      publish_path = true;
      ROS_INFO("map size : %d x %d",map_data.getSize()(0),map_data.getSize()(1));
      float dist_min = 0.0f;
      float dmin = 0.0f;
      Index goal_idx;
      int i=0;
      double grid_max = 0;
      double max_buf = 0;
      double grid_min = 0;
      double min_buf = 0;
      for (GridMapIterator it(map_data); !it.isPastEnd(); ++it){
          if(!map_data.isValid(*it,"global")) continue;
          max_buf = map_data.at("global",*it);
           if(max_buf >= grid_max)
             {
               grid_max = max_buf;
             }
           min_buf = map_data.at("global",*it);
           if(min_buf <= grid_min && min_buf != -1)
             {
               grid_min = min_buf;
             }
        }
        cout << "max : " << grid_max << " min : " << grid_min << endl;
      for (GridMapIterator it(map_data); !it.isPastEnd(); ++it) {          
        if (!map_data.isValid(*it, "global")){
              world_map[i] = 255;
            }
        else{

             Eigen::Vector2d center;
             map_data.getPosition(*it,center);

                //z_cost = (0.35/0.2)*map_data.at("cost_elevation",*it);

                world_map[i] = (int)(255*map_data.at("global",*it));
 //               cout << "world cost : " << world_map[i] << endl;
                if(world_map[i] >= 255){world_map[i]=255;}
                else if(world_map[i] <= 0 ){world_map[i] = 0;}
              Position test_pos;
              test_pos(0) = user_goal_pose.pose.position.x;
              test_pos(1) = user_goal_pose.pose.position.y;
              map_data.getIndex(test_pos,goal_idx);
            }
            i++;
      }
      cout << "id target : " << goal_idx(0) << "   id target : " << goal_idx(1) << endl;
//user_goal_pose
      vector<int> x_node_grid;
      vector<int> y_node_grid;

      cout << "A* Search implementation\n(C)2016 Ji min Lee\n";
      AStarSearch<MapSearchNode> astarsearch;

         unsigned int SearchCount = 0;

         const unsigned int NumSearches = 1;

         while(SearchCount < NumSearches)
         {

                 // Create a start state
                 MapSearchNode nodeStart;
                 nodeStart.x = 190;//195;//rand()%MAP_WIDTH;
                 nodeStart.y = 100;//100;//rand()%MAP_HEIGHT;

                 // Define the goal state
                 MapSearchNode nodeEnd;
                 nodeEnd.x = goal_idx(0);//rand()%MAP_WIDTH;
                 nodeEnd.y = goal_idx(1);//rand()%MAP_HEIGHT;
                 //nodeEnd.x = 140;//rand()%MAP_WIDTH;
                 //nodeEnd.y = 82;//rand()%MAP_HEIGHT;

                 // Set Start and goal states

                 astarsearch.SetStartAndGoalStates( nodeStart, nodeEnd );

                 unsigned int SearchState;
                 unsigned int SearchSteps = 0;

                 do
                 {
                         SearchState = astarsearch.SearchStep();
                         SearchSteps++;
                 }
                 while( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );
                 if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED )
                 {
                         cout << "Search found goal state\n";
                         MapSearchNode *node = astarsearch.GetSolutionStart();
                         int steps = 0;
                         node->PrintNodeInfo();

                         for( ;; )
                         {
                                 node = astarsearch.GetSolutionNext();
                                 if( !node ){ break;}
                                 node->PrintNodeInfo();
                                 x_node_grid.push_back(node->x);
                                 y_node_grid.push_back(node->y);
                                 steps ++;
                         };

                         cout << "Solution steps " << steps << endl;
                         astarsearch.FreeSolutionNodes();
                 }
                 else if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED )
                 {
                         cout << "Search terminated. Did not find goal state\n";
                 }
                 cout << "SearchSteps : " << SearchSteps << "\n";
                 SearchCount ++;
                 astarsearch.EnsureMemoryFreed();
         }

   cout << "x path : " << x_node_grid.size()  << endl;
   cout << "y path : " << y_node_grid.size()  << endl;
   root_arr.data.clear();
   for(int i=0;i<x_node_grid.size();i++)
     {
       root_arr.data.push_back(x_node_grid.at(i));
       root_arr.data.push_back(y_node_grid.at(i));
     }

  /* for(GridMapIterator it(map_data); !it.isPastEnd(); ++it)
     {
        map_data.at("root",*it) = 255;
     }

   for(size_t i=0; i<x_node_grid.size(); i++)
     {
       Index idx;
       idx(0) = x_node_grid.at(i);
       idx(1) = y_node_grid.at(i);
       //for(GridMapIterator it(map_data); !it.isPastEnd(); ++it)
       //  {

            map_data.at("root",idx) = 0;
       //  }

       //cout << "x,y : " << x_node_grid.at(i) << y_node_grid.at(i) << endl;
     }*/
   Index plan_idx;
   Position astar_grid_pos;

   astar_root_pose.header.frame_id = "base_link";
   astar_root_pose.pose.orientation.x = 0.0;
   astar_root_pose.pose.orientation.y = 0.0;
   astar_root_pose.pose.orientation.z = 0.0;
   astar_root_pose.pose.orientation.w = 0.0;
   astar_root_pose.pose.position.z = 0.0;
   a_star_plan.clear();
   for(int j=0;j<x_node_grid.size(); j++){
       plan_idx(0) = x_node_grid.at(j);
       plan_idx(1) = y_node_grid.at(j);
       map_data.at("global",plan_idx) = 255;
       map_data.getPosition(plan_idx,astar_grid_pos);

       astar_root_pose.pose.position.x = astar_grid_pos(0);
       astar_root_pose.pose.position.y = astar_grid_pos(1);
       a_star_plan.push_back(astar_root_pose);
   }
   a_star_path.poses.clear();
   a_star_path.poses.resize(a_star_plan.size());



   Index spline_plan_idx;
   Position spline_grid_pos;
   int spline_deg = 3;
   int spline_dimension = 2;
   int spline_control_point = x_node_grid.size();

   int divide_control_point = spline_control_point/5;

   ts::BSpline spline(spline_deg,spline_dimension,divide_control_point,TS_CLAMPED);
   std::vector<float> ctrlp = spline.ctrlp();
   b_spline_plan.clear();
   for(int i=0;i<divide_control_point;i++){

       if(i==(divide_control_point-1))
         {
           spline_plan_idx(0) = x_node_grid.back();
           spline_plan_idx(1) = y_node_grid.back();
         }
       else
         {
           spline_plan_idx(0) = x_node_grid.at(5*i);
           spline_plan_idx(1) = y_node_grid.at(5*i);
         }

       map_data.at("global",spline_plan_idx) = 255;
       map_data.getPosition(spline_plan_idx,spline_grid_pos);

       ctrlp[2*i] = (float)spline_grid_pos(0);
       ctrlp[1+2*i] = (float)spline_grid_pos(1);

     }

   spline.setCtrlp(ctrlp);
   std::vector<float> spline_result;
   float spline_resolution = 0.0f;
   b_spline_pose.header.frame_id = "base_link";


   for(int i=0;i<1000;i++)
     {
       spline_result = spline.evaluate(spline_resolution).result();
       spline_resolution = i*0.001;
       b_spline_pose.pose.position.z = 0.0;
       b_spline_pose.pose.orientation.w = 1.0;
       b_spline_pose.pose.orientation.x = 0.0;
       b_spline_pose.pose.orientation.y = 0.0;
       b_spline_pose.pose.orientation.z = 0.0;
       b_spline_pose.pose.position.x = (double)spline_result[0];
       b_spline_pose.pose.position.y = (double)spline_result[1];
       b_spline_plan.push_back(b_spline_pose);
       spline_result.clear();
     }
      spline_result.clear();
      ctrlp.clear();
       b_spline_path.poses.clear();
       b_spline_path.poses.resize(b_spline_plan.size());

       for(int i=0;i<b_spline_plan.size();i++){
           b_spline_path.poses.push_back(b_spline_plan.at(i));
         }

   /*ctrlp[0]  = -1.75f; // x0
   ctrlp[1]  = -1.0f;  // y0
   ctrlp[2]  = -1.5f;  // x1
   ctrlp[3]  = -0.5f;  // y1
   ctrlp[4]  = -1.5f;  // x2
   ctrlp[5]  =  0.0f;  // y2
   ctrlp[6]  = -1.25f; // x3
   ctrlp[7]  =  0.5f;  // y3
   ctrlp[8]  = -0.75f; // x4
   ctrlp[9]  =  0.75f; // y4
   ctrlp[10] =  0.0f;  // x5
   ctrlp[11] =  0.5f;  // y5
   ctrlp[12] =  0.5f;  // x6
   ctrlp[13] =  0.0f;  // y6
   spline.setCtrlp(ctrlp);
   std::vector<float> result = spline.evaluate(1.0f).result();
   std::cout << "x = " << result[0] << ", y = " << result[1] << " curv size : " << result.size() << std::endl;*/

   for(int a=0; a<a_star_plan.size();a++){
   a_star_path.poses.push_back(a_star_plan.at(a));
     }

      do_local_planning = false;
      ros::Time tock = ros::Time::now();
      cout << "time " << tick - tock << endl;
  }
}

void goal_pose_stamped_cb(const geometry_msgs::PoseStamped::ConstPtr& goal_pose_stamped)
{
   user_goal_pose.header.frame_id = goal_pose_stamped->header.frame_id;
   user_goal_pose.header.stamp = goal_pose_stamped->header.stamp;
   user_goal_pose.pose.position.x = goal_pose_stamped->pose.position.x;
   user_goal_pose.pose.position.y = goal_pose_stamped->pose.position.y;
   user_goal_pose.pose.position.z = goal_pose_stamped->pose.position.z;
   user_goal_pose.pose.orientation.x = goal_pose_stamped->pose.orientation.x;
   user_goal_pose.pose.orientation.y = goal_pose_stamped->pose.orientation.y;
   user_goal_pose.pose.orientation.z = goal_pose_stamped->pose.orientation.z;
   user_goal_pose.pose.orientation.w = goal_pose_stamped->pose.orientation.w;
   do_local_planning = true;
}

int main(int argc, char** argv)
{  
  ros::init(argc, argv, "a_star");
  ros::NodeHandle nh;
  ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("plan_test", 1, true);
  ros::Publisher pub_root = nh.advertise<nav_msgs::Path>("astar_root",1,true);
  ros::Publisher pub_spline_root = nh.advertise<nav_msgs::Path>("b_spline_root",1,true);
  ros::Publisher pub_footprint = nh.advertise<geometry_msgs::PolygonStamped>("robot_footprint",1,true);
  ros::Publisher pub_root_grid = nh.advertise<std_msgs::Int32MultiArray>("root_grid_array",100);

  ros::Subscriber sub_map = nh.subscribe("grid_map",1,map_cb);
  ros::Subscriber sub_goal = nh.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal",1,goal_pose_stamped_cb);

 /* GridMap map({"ele","planner"});
  map.setFrameId("base_link");
  //map.setGeometry(Length(MAX_X,MAX_Y), 0.03,Position(5,0));
  map.setGeometry(Length(20,20), 1,Position(5,0));


     //ROS_INFO("map size : %d x %d",map.getSize()(0),map.getSize()(1));

     vector<int> x_node_grid;
     vector<int> y_node_grid;

     cout << "A* Search implementation\n(C)2016 Ji min Lee\n";
     AStarSearch<MapSearchNode> astarsearch;

	unsigned int SearchCount = 0;

	const unsigned int NumSearches = 1;

	while(SearchCount < NumSearches)
	{

		// Create a start state
		MapSearchNode nodeStart;
		nodeStart.x = 3;//rand()%MAP_WIDTH;
		nodeStart.y = 2;//rand()%MAP_HEIGHT;

		// Define the goal state
		MapSearchNode nodeEnd;
		nodeEnd.x = 2;//rand()%MAP_WIDTH;
		nodeEnd.y = 12;//rand()%MAP_HEIGHT;

		// Set Start and goal states

		astarsearch.SetStartAndGoalStates( nodeStart, nodeEnd );

		unsigned int SearchState;
		unsigned int SearchSteps = 0;

		do
		{
			SearchState = astarsearch.SearchStep();
			SearchSteps++;
		}
		while( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );
		if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED )
		{
			cout << "Search found goal state\n";
			MapSearchNode *node = astarsearch.GetSolutionStart();
			int steps = 0;
			node->PrintNodeInfo();

			for( ;; )
			{
				node = astarsearch.GetSolutionNext();
				if( !node ){ break;}
				node->PrintNodeInfo();
				x_node_grid.push_back(node->x);
				y_node_grid.push_back(node->y);
				steps ++;
			};

			cout << "Solution steps " << steps << endl;
			astarsearch.FreeSolutionNodes();
		}
		else if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED ) 
		{
			cout << "Search terminated. Did not find goal state\n";
		}
		cout << "SearchSteps : " << SearchSteps << "\n";
		SearchCount ++;
		astarsearch.EnsureMemoryFreed();
	}

  cout << "x path : " << x_node_grid.size()  << endl;
  cout << "y path : " << y_node_grid.size()  << endl;*/
  ros::Rate rate(30.0);
  while (nh.ok()) {

      /*for(int j=0;j<x_node_grid.size();j++){
          world_map[(20*y_node_grid.at(j))+x_node_grid.at(j)] = 6;
      }
      ROS_INFO("Err1");
      ros::Time time = ros::Time::now();
      int i=0;
      for(int j=0;i<MAP_WIDTH * MAP_HEIGHT;i++){
         // ROS_INFO("%f",world_map[j]);
        }

      for (GridMapIterator it(map_data); !it.isPastEnd(); ++it) {
           Position position;
           //map_data.getPosition(*it, position);
           //map_data.at("elevation", *it) = -0.04 + 0.2 * std::sin(3.0 * 30 + 5.0 * position.y()) * position.x();
           map_data.at("planner", *it) = world_map[i];
           i++;
         }
      ROS_INFO("Err2");
         map_data.setTimestamp(time.toNSec());
         grid_map_msgs::GridMap message;
         GridMapRosConverter::toMessage(map_data, message);
         publisher.publish(message);*/
      ros::Time time = ros::Time::now();
      map_data.setTimestamp(time.toNSec());
      grid_map_msgs::GridMap message;
      GridMapRosConverter::toMessage(map_data, message);
      publisher.publish(message);

      geometry_msgs::Point32 p32;
      geometry_msgs::PolygonStamped pstamped;
      p32.x = 0.35;
      p32.y = -0.35;
      p32.z = 0.0;
      pstamped.polygon.points.push_back(p32);
      p32.x = 0.35;
      p32.y = 0.35;
      p32.z = 0.0;
      pstamped.polygon.points.push_back(p32);
      p32.x = -0.35;
      p32.y = 0.35;
      p32.z = 0.0;
      pstamped.polygon.points.push_back(p32);
      p32.x = -0.35;
      p32.y = -0.35;
      p32.z = 0.0;
      pstamped.polygon.points.push_back(p32);

      pstamped.header.frame_id = "base_link";
      pstamped.header.stamp = ros::Time::now();

      pub_footprint.publish(pstamped);

      if(publish_path)
      {

          a_star_path.header.frame_id = "base_link";
          a_star_path.header.stamp = ros::Time::now();
          pub_root.publish(a_star_path);
          b_spline_path.header.frame_id = "base_link";
          b_spline_path.header.stamp = ros::Time::now();
          pub_spline_root.publish(b_spline_path);
          pub_root_grid.publish(root_arr);
        }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
