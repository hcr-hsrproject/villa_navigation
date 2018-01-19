// #include <ros/ros.h>
// #include <actionlib/client/simple_action_client.h>
// #include <controller_manager_msgs/ListControllers.h>
// #include <controller_manager_msgs/ControllerState.h>
// #include <control_msgs/FollowJointTrajectoryAction.h>
// #include <control_msgs/FollowJointTrajectoryGoal.h>
// //mk-----------------
// #include <std_msgs/Float32.h>
// #include <nav_msgs/Path.h>
// #include <nav_msgs/Odometry.h>
// #include <geometry_msgs/Pose2D.h>
// #include <geometry_msgs/PointStamped.h>
// #include <vector>
// //mk-----------------

// typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;

// //mk-----------------
// static int path_indx=0;
// static int pathSize=2;
// std::vector<double> base_trajectory(3,0.0);
// std::vector<double> x_;
// std::vector<double> y_;
// std::vector<double> t_;
// std::vector<double> hsrb_base_pose(3,0.0);
// std::vector<double> fixed_hsrb_base_pose(3,0.0);
// double desired_heading=0.0;

// bool BoolUpadated=false;


// void mdp_pathCallback(const nav_msgs::Path::ConstPtr& msg)
// {
//     //ROS_INFO("mdp_path callback msg");

//     pathSize=msg->poses.size();

//     x_.resize(pathSize,0.0);
//     y_.resize(pathSize,0.0);
//     t_.resize(pathSize,0.0);

//      //save path from rosnode subscribe    
//    for(int k(0);k<pathSize;k++)
//    {
//     x_[k]=msg->poses[k].pose.position.x;
//     y_[k]=msg->poses[k].pose.position.y;
//     t_[k]=msg->poses[k].pose.orientation.z;
//     // printf("mdp path x-coord : %lf, y-coord : %lf \n", x_[k],y_[k]);
//    }
//    BoolUpadated=true;
// }


// void ClickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
// {
//     //  BoolUpadated=true;
//     // for(int i(0);i<3;i++)
//     // {
//     //   fixed_hsrb_base_pose[i]= hsrb_base_pose[i];
//     // }

// }

// void hsrb_odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
// {
//    // ROS_INFO("odom callback msg");

//      hsrb_base_pose[0]=msg->pose.pose.position.x;
//      hsrb_base_pose[1]=msg->pose.pose.position.y;
//      hsrb_base_pose[2]=msg->pose.pose.orientation.z;
// }

// void Robotheading_Callback(const std_msgs::Float32::ConstPtr& msg)
// {
//   ROS_INFO("desired heading received : %.3lf", msg->data);
//     desired_heading=msg->data;

// }


// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "test");

//   // initialize action client
//   Client cli("/hsrb/omni_base_controller/follow_joint_trajectory", true);
//   // wait for the action server to establish connection
//   cli.waitForServer();
//   // make sure the controller is running
//   ros::NodeHandle n;
//   ros::ServiceClient client = n.serviceClient<controller_manager_msgs::ListControllers>("/hsrb/controller_manager/list_controllers");
//   controller_manager_msgs::ListControllers list_controllers;
//   bool running = false;
//   while (running == false) {
//     ros::Duration(0.1).sleep();
//     if (client.call(list_controllers)) {
//       for (unsigned int i = 0; i < list_controllers.response.controller.size(); i++) {
//         controller_manager_msgs::ControllerState c = list_controllers.response.controller[i];
//         if (c.name == "omni_base_controller" && c.state == "running") {
//           running = true;
//         }
//       }
//     }
//   }
//   //ros::Publisher controlmsg_Pub= n.advertise<control_msgs/FollowJointTrajectoryGoal>("MDP/costmap", 10);
//   //MK-customizing subsriber
//   ros::Subscriber mdppath_sub   = n.subscribe<nav_msgs::Path>("mdp_path_dynamic", 100, mdp_pathCallback); 
//   ros::Subscriber odompath_sub   = n.subscribe<nav_msgs::Odometry>("hsrb/odom", 100, hsrb_odomCallback); 
//   ros::Subscriber clickedpoint_sub   = n.subscribe<geometry_msgs::PointStamped>("/clicked_point", 100, ClickedPointCallback); 
//   ros::Subscriber robot_heading_sub = n.subscribe<std_msgs::Float32>("mdp_HeadingV",100,Robotheading_Callback);

//   std_msgs::Float32::ConstPtr Hmsg = ros::topic::waitForMessage<std_msgs::Float32>("mdp_HeadingV");
//   desired_heading=Hmsg->data;


//   //ros::spinOnce();
//   // fill ROS message 
//   ros::Rate loop_rate(20);

//   x_.resize(3,0.0);
//   y_.resize(3,0.0);

//   //Initialize fixed_hsrb_base_pos
//   for(int i(0);i<3;i++)
//   {
//     fixed_hsrb_base_pose[i]= hsrb_base_pose[i];
//   }

//   while (ros::ok())
//   {
//      control_msgs::FollowJointTrajectoryGoal goal;

//       if(BoolUpadated)
//       {
//          for(int i(0);i<3;i++)
//          {
//             fixed_hsrb_base_pose[i]= hsrb_base_pose[i];
//          }

//         goal.trajectory.joint_names.push_back("odom_x");
//         goal.trajectory.joint_names.push_back("odom_y");
//         goal.trajectory.joint_names.push_back("odom_t");

//         goal.trajectory.points.resize(pathSize);
        
//         for(int z=0;z<pathSize;z++){
//           goal.trajectory.points[z].positions.resize(3);
//           goal.trajectory.points[z].positions[0] = x_[z];
//           goal.trajectory.points[z].positions[1] = y_[z];
//           goal.trajectory.points[z].positions[2] = desired_heading;
//           //goal.trajectory.points[z].orientation[2] = t_[z];
//           goal.trajectory.points[z].velocities.resize(3);
    
//         for (size_t i = 0; i < 3; ++i) {
//           goal.trajectory.points[z].velocities[i] = 0.0;
//         }
//         double time_duration=z*10.0;
//         goal.trajectory.points[z].time_from_start = ros::Duration(time_duration);

//       }
//       // send message to the action server
//       cli.sendGoal(goal);

//       // wait for the action server to complete the order
//       cli.waitForResult(ros::Duration(20.0));  

//         BoolUpadated=false;  


//       }

//       // printf("Pathsize : %d",pathSize);
     


//      ros::spinOnce();  
//      loop_rate.sleep();  
//   }

//   ros::spin();
 
//   return 0;
// }
