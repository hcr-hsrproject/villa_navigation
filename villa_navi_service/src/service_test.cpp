#include "ros/ros.h"
#include "navi_service_node.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <sstream>
#include <signal.h>
#include <boost/thread/thread.hpp>
#include <villa_navi_service/GoTargetPos.h>
using namespace Eigen;

bool g_caught_sigint=false;

void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "villa_navi_srvice");

  villa_navi_srv villa_navi_srvice;
    
  ros::Subscriber global_pos_sub;
  ros::Subscriber edge_leg_sub;
  
  ros::NodeHandle n;
         
  villa_navi_srvice.setNavTarget_pub=n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/move/goal",50,true);
  villa_navi_srvice.Gaze_point_pub= n.advertise<geometry_msgs::Point>("/gazed_point_fixing_node/target_point", 50, true);
  villa_navi_srvice.Gaze_activate_pub= n.advertise<std_msgs::Bool>("/gazed_point_fixing_node/activate", 50, true);
  
  // global_pos_sub= n.subscribe<geometry_msgs::PoseStamped>("/global_pose", 10, &villa_navi_srv::global_pose_callback,&villa_navi_srvice);
  // edge_leg_sub=n.subscribe<geometry_msgs::PoseArray>("/edge_leg_detector", 10, &villa_navi_srv::edge_leg_callback,&villa_navi_srvice);
  ros::Rate loop_rate(50);

  ros::ServiceServer service = n.advertiseService("/navi_go_base",  &villa_navi_srv::goTarget,&villa_navi_srvice);
  
  signal(SIGINT, sig_handler);
  double ros_rate = 3.0;
  ros::Rate r(ros_rate);

  while (ros::ok())
  {
	   
     // villa_navi_srvice.Publish_nav_target();
     ros::spinOnce();
     r.sleep();
  }

  ros::spin();

  return 0;
}




