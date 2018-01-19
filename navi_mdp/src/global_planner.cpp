#include "ros/ros.h"
#include "MDPmanager.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PointStamped.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense>
#include <sstream>
#include <boost/thread/thread.hpp>
#include "srBSpline.h"

using namespace Eigen;

bool boolSolve=false;
MapParam   mapParam;
static int  Receive_count=0;
int x_size=Grid_Num_X;
int y_size=Grid_Num_Y;
double unit_step=0.25;
double center_offset=0.3;
vector<int>    localoccupancy(Grid_Num_X*Grid_Num_Y,0);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "MDP_planner");
  
  MDPManager problemmanager; 
  
  //problemmanager.setRosObj(&nodeObj);
   problemmanager.setPMapParam(&mapParam);

  // ros::Rate r(5);
  ros::Subscriber Point_sub;          //subscriber clicked point
  ros::Subscriber CurPos_sub;         //subscriber tracking current base position
  ros::Subscriber Localmap_sub;       //subscriber detecting lcal map with laser scan
  ros::Subscriber staticmap_sub;
  ros::Subscriber global_pos_sub;

  ros::NodeHandle n;
  
  Point_sub     = n.subscribe<geometry_msgs::PointStamped>("/clicked_point", 10, &MDPManager::ClikedpointCallback,&problemmanager);
  //Localmap_sub  = n.subscribe<nav_msgs::OccupancyGrid>("/local_map_navigation_map/local_map", 30, &MDPManager::Local_mapCallback,&problemmanager); 
  staticmap_sub = n.subscribe<nav_msgs::OccupancyGrid>("/static_obstacle_map_ref", 30, &MDPManager::static_mapCallback,&problemmanager); 
  // Basepos_sub   = n.subscribe<nav_msgs::Odometry>("/hsrb/odom", 10, &MDPManager::base_pose_callback,&problemmanager);
  global_pos_sub= n.subscribe<geometry_msgs::PoseStamped>("/global_pose", 10, &MDPManager::global_pose_callback,&problemmanager);

  ros::Rate loop_rate(20);

  while (ros::ok())
  {
 
     //problemmanager.MDPsolPublish();
     if(problemmanager.m_boolSolve)
     {
        printf("Begin to solve\n");
        problemmanager.pathPublish();
        problemmanager.MDPsolve(); 
        //problemmanager.printPath(); 
        problemmanager.generatePath();
        
        printf("End solve\n");
        problemmanager.m_boolSolve=false;
     }

  	 ros::spinOnce();
     loop_rate.sleep();  
  }

  ros::spin();

  return 0;
}




