#include "ros/ros.h"
#include "MDPmanager.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Dense>
#include <sstream>
#include <boost/thread/thread.hpp>
#include "srBSpline.h"

using namespace Eigen;


bool boolSolve=false;
MapParam   dynamicmapParam(14,14,0.5);
static int  Receive_count=0;


void CmdIntCallback(const std_msgs::Int8::ConstPtr& msg)
{


}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_planner");
  
  MDPManager dynamicManager; 
  
  //problemmanager.setRosObj(&nodeObj);
   dynamicManager.setPMapParam(&dynamicmapParam);

  // ros::Rate r(5);
  ros::Subscriber Point_sub;          //subscriber clicked point
  ros::Subscriber dynamicmap_sub;
  ros::Subscriber Basepos_sub;
  ros::Subscriber human_marker_sub;
  ros::Subscriber human_cmd_sub;
  ros::Subscriber global_pos_sub;
  ros::NodeHandle n;
  
  Point_sub     = n.subscribe<geometry_msgs::PointStamped>("/clicked_point", 10, &MDPManager::ClikedpointCallback,&dynamicManager);
  human_cmd_sub = n.subscribe<std_msgs::Int8>("/Int_cmd_trackhuman", 50, &MDPManager::Human_target_cmdCallback,&dynamicManager);
  human_marker_sub= n.subscribe<visualization_msgs::Marker>("/human_target", 50, &MDPManager::Human_MarkerCallback,&dynamicManager);
  dynamicmap_sub =n.subscribe<nav_msgs::OccupancyGrid>("/dynamic_obstacle_map_ref", 30, &MDPManager::dynamic_mapCallback,&dynamicManager); 
  Basepos_sub   = n.subscribe<nav_msgs::Odometry>("/hsrb/odom", 10, &MDPManager::base_pose_callback,&dynamicManager);
  global_pos_sub= n.subscribe<geometry_msgs::PoseStamped>("/global_pose", 10, &MDPManager::global_pose_callback,&dynamicManager);
  ros::Rate loop_rate(20);



  nav_msgs::Odometry::ConstPtr Omsg = ros::topic::waitForMessage<nav_msgs::Odometry>("/hsrb/odom");
  dynamicManager.CurVector[0]= Omsg->pose.pose.position.x;
  dynamicManager.CurVector[1]= Omsg->pose.pose.position.y;

  while (ros::ok())
  {
      //problemmanager.MDPsolPublish();
      if(dynamicManager.m_boolSolve)
      {
         ROS_INFO("Start to dynamic mdp");
         dynamicManager.MDPsolve(); 
         dynamicManager.generate_dynamicPath();
         dynamicManager.m_boolSolve=false;
         dynamicManager.booltrackHuman=false;
         dynamicManager.dyn_path_num++;
      }
      else
      {
        if(dynamicManager.dyn_path_num>0)
          dynamicManager.publishpaths();
      }

  	 ros::spinOnce();
     loop_rate.sleep();  
  }

  ros::spin();

  return 0;
}




