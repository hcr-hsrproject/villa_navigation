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
#include <yaml-cpp/yaml.h>

using namespace Eigen;

class waypoint_manager{

public:
  explicit waypoint_manager(ros::NodeHandle n_){
  
  //ros::NodeHandle n;

  m_client = n_.serviceClient<villa_navi_service::GoTargetPos>("/navi_go_base");
  waypoint_sub = n_.subscribe<std_msgs::String>("/way_point", 10, &waypoint_manager::waypoint_callback,this);
  int_sub = n_.subscribe<std_msgs::Int8>("/way_point_int", 10, &waypoint_manager::waypoint_int_callback,this);

  parseparameters(n_);
  
  }
  ~waypoint_manager(){}

  ros::ServiceClient m_client; 
  ros::Subscriber waypoint_sub;
  ros::Subscriber int_sub;
  std::map< std::string, std::vector<double> > goal_maps;

  void waypoint_int_callback(const std_msgs::Int8::ConstPtr& msg){
  
      ROS_INFO("waypoint_int_callback");
  
  }
  void waypoint_callback(const std_msgs::String::ConstPtr& msg)
  {
      double x_map =0.0;
      double y_map =0.0;
     
      ROS_INFO("waypoint_callback");
      //printf("Receive point %.3lf , %.3lf \n",x_map,y_map);
      std::string goal_loc = msg->data;
      
      std::cout<<"received goal locations: " << goal_loc << std::endl;

      auto search = goal_maps.find(goal_loc.c_str());
      //std::cout<< search->second<<std::endl;

      if((search->second).size()>0)
      {
          //std::cout<<"recieved_goal: " <<  search->first << ", positions: "
          //<<(search->second)[0]<<","<<(search->second)[1]<<","<<(search->second)[2]<<std::endl;
          x_map=(search->second)[0];
          y_map=(search->second)[1];
      }
      //std::vector<double> target;
      //target = ;


      villa_navi_service::GoTargetPos navi_srv;

      //double x_map =2.0;
      //double y_map =2.0;
      printf("Receive point %.3lf , %.3lf \n",x_map,y_map);
      navi_srv.request.x_from_map=x_map;
      navi_srv.request.y_from_map=y_map;
      navi_srv.request.theta_from_map=0;
      m_client.call(navi_srv);

  }

  void LoadPositionfromConfig(ros::NodeHandle n, std::string input_locations)
  {

      XmlRpc::XmlRpcValue input_loc;
      std::string param_name = "villa_navi_service/" + input_locations;
      n.getParam(param_name, input_loc);
      std::vector<double> tmp_pos(3,0.0);
      for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = input_loc.begin(); it != input_loc.end(); ++it) {
          ROS_INFO_STREAM("Loded "<< input_locations << "-" << (std::string)(it->first) << ",  " << input_loc[it->first]);

          tmp_pos[0]=static_cast<double>(input_loc["x"]);
          tmp_pos[1]=static_cast<double>(input_loc["y"]);
          tmp_pos[2]=static_cast<double>(input_loc["t"]);
      }

      goal_maps[input_locations]=tmp_pos;
  }

  void parseparameters(ros::NodeHandle n)
  {
      //XmlRpc::XmlRpcValue ref_frame;
      std::string target_frame;
      n.getParam("villa_navi_service/ref_frame", target_frame);
      //ROS_ASSERT(ref_frame.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      ROS_INFO_STREAM("reference frame: " << target_frame);

      LoadPositionfromConfig(n,"door");
      LoadPositionfromConfig(n,"goal");
      LoadPositionfromConfig(n,"living_room");
      LoadPositionfromConfig(n,"home_pos");
      LoadPositionfromConfig(n,"corridor");

      //print goalmaps
      std::map< std::string, std::vector<double> >::iterator goal_it = goal_maps.begin();
      for(goal_it ; goal_it!=goal_maps.end();goal_it++)
      {
          std::cout<<"goal: " <<  goal_it->first << ", positions: "
              <<(goal_it->second)[0]<<","<<(goal_it->second)[1]<<","<<(goal_it->second)[2]<<std::endl;
      }

  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_client");

  ros::NodeHandle n;
  waypoint_manager manager(n);

  ros::Rate loop_rate(50);
  double ros_rate = 3.0;
  ros::Rate r(ros_rate);

  while (ros::ok())
  {	   
      ros::spinOnce();
      r.sleep();
  }

  ros::spin();


  return 0;
}




