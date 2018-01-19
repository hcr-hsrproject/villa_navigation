#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/ControllerState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
//mk-----------------
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>
//mk-----------------

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;

//mk-----------------
static int path_indx=0;
static int pathSize=2;
std::vector<double> base_trajectory(3,0.0);
std::vector<double> x_;
std::vector<double> y_;

std::vector<double> hsrb_base_pose(2,0.0);


bool BoolMove=false;

void hsrb_odomCallback(const nav_msgs::Odometry ::ConstPtr& msg)
{
    ROS_INFO("odom callback msg");

}
//mk-----------------


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");

  //subscribe only once for topic
  nav_msgs::Path::ConstPtr msg = ros::topic::waitForMessage<nav_msgs::Path>("/mdp_path");
  pathSize=msg->poses.size();
  x_.resize(pathSize,0.0);
  y_.resize(pathSize,0.0);

  //save path from rosnode subscribe    
  for(int k(0);k<pathSize;k++)
  {
    x_[k]=msg->poses[k].pose.position.x;
    y_[k]=msg->poses[k].pose.position.y;
    printf("mdp path x-coord : %lf, y-coord : %lf \n", x_[k],y_[k]);
   }

  nav_msgs::Odometry::ConstPtr odom_msg = ros::topic::waitForMessage<nav_msgs::Odometry>("hsrb/odom");
  hsrb_base_pose[0]=odom_msg->pose.pose.position.x;
  hsrb_base_pose[1]=odom_msg->pose.pose.position.y;

  // initialize action client
  Client cli("/hsrb/omni_base_controller/follow_joint_trajectory", true);

  // wait for the action server to establish connection
  cli.waitForServer();
  
  // make sure the controller is running
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<controller_manager_msgs::ListControllers>("/hsrb/controller_manager/list_controllers");
  controller_manager_msgs::ListControllers list_controllers;
  bool running = false;
  while (running == false) {
    ros::Duration(0.1).sleep();
    if (client.call(list_controllers)) {
      for (unsigned int i = 0; i < list_controllers.response.controller.size(); i++) {
        controller_manager_msgs::ControllerState c = list_controllers.response.controller[i];
        if (c.name == "omni_base_controller" && c.state == "running") {
          running = true;
        }
      }
    }
  }

  
  //MK-customizing subsriber
  //ros::Subscriber mdppath_sub   = n.subscribe<nav_msgs::Path>("mdp_path_2", 100, mdp_pathCallback); 
  //ros::Subscriber mdppath_sub   = n.subscribe<nav_msgs::Odometry>("hsrb/odom", 100, hsrb_odomCallback); 
 
  ros::spinOnce();

  // fill ROS message
  control_msgs::FollowJointTrajectoryGoal goal;

  goal.trajectory.joint_names.push_back("odom_x");
  goal.trajectory.joint_names.push_back("odom_y");
  goal.trajectory.joint_names.push_back("odom_t");

  goal.trajectory.points.resize(pathSize);
  for(int z=0;z<pathSize;z++)
  {
    goal.trajectory.points[z].positions.resize(3);
    goal.trajectory.points[z].positions[0] =hsrb_base_pose[0]+x_[z];
    goal.trajectory.points[z].positions[1] =hsrb_base_pose[1]+y_[z];
    goal.trajectory.points[z].positions[2] = 0.0;
    goal.trajectory.points[z].velocities.resize(3);
    for (size_t i = 0; i < 3; ++i) {
      goal.trajectory.points[z].velocities[i] = 0.0;
    }
    double time_duration=4*z+4;
    goal.trajectory.points[z].time_from_start = ros::Duration(time_duration);

  }
  // send message to the action server
  cli.sendGoal(goal);

  // wait for the action server to complete the order
  cli.waitForResult(ros::Duration(10.0));


  ros::Rate loop_rate(20);

  while (ros::ok())
  {
    
   


    ros::spinOnce();  
     loop_rate.sleep();  
  }

  ros::spin();
 
  return 0;
}
