#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <sstream>
#include <boost/thread/thread.hpp>
#include <villa_navi_service/GoTargetPos.h>
#include <XmlRpcValue.h>

using namespace Eigen;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "navi_srvice_client");

  ros::NodeHandle n;
         
  double door_x_map = 0.0;
  double door_y_map = 0.0;

  XmlRpc::XmlRpcValue goal_point;
  n.getParam("villa_navi_service/goal_locations", goal_point);
  ROS_ASSERT(goal_point.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  ROS_INFO_STREAM("goal point in the map : " << goal_point);

  //alternatives
  //n.getParam("villa_navi_service/goal_locations/x", door_x_map);
  //n.getParam("villa_navi_service/goal_locations/y", door_y_map);
  //
  double x_map = goal_point["x"];
  double y_map = goal_point["y"];
  double t_map = goal_point["t"];

  std::cout<<"get parameter succeed: x : "<<x_map <<std::endl;
  std::cout<<"get parameter succeed:y :"<< y_map <<std::endl;

  ros::Rate loop_rate(50);

  ros::ServiceClient service_client = n.serviceClient<villa_navi_service::GoTargetPos>("/navi_go_base");

  villa_navi_service::GoTargetPos navi_srv;
  navi_srv.request.x_from_map=x_map;
  navi_srv.request.y_from_map=y_map;
  navi_srv.request.theta_from_map=0;

  service_client.call(navi_srv);
	   

  return 0;
}




