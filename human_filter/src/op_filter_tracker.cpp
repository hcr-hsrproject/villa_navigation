/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

#include "people_tracking_filter/op_filter_tracker.h"
#include "people_tracking_filter/tracker_particle.h"
#include "people_tracking_filter/tracker_kalman.h"
#include "people_tracking_filter/state_pos_vel.h"
#include "people_tracking_filter/rgb.h"
#include <people_msgs/PositionMeasurement.h>

using namespace std;
using namespace tf;
using namespace BFL;
using namespace message_filters;

static const double       sequencer_delay            = 0.5; //TODO: this is probably too big, it was 0.8
static const unsigned int sequencer_internal_buffer  = 100;
static const unsigned int sequencer_subscribe_buffer = 10;
static const unsigned int num_particles_tracker      = 1000;
static const double       tracker_init_dist          = 4.0;

bool IsNotInitilized = true;

// constructor
op_filter_manager::op_filter_manager(ros::NodeHandle nh)
  : nh_(nh),
    robot_state_(),
    filter_counter_(0),
    edge_leg_iter(0),
    pub_iters(0),
    OnceTarget(false)
{
  // initialize
  // advertise filter output
  Leg_boxes_pub=nh_.advertise<visualization_msgs::MarkerArray>("/filtered_leg_target", 10);
  Leg_poses_pub=nh_.advertise<geometry_msgs::PoseArray>("/filtered_leg_poses", 10);
  human_target_pub=nh_.advertise<visualization_msgs::Marker>("/leg_target", 10);
  filtered_human_target_pub=nh_.advertise<visualization_msgs::Marker>("/filtered_target", 10);
  people_measurement_pub_ = nh_.advertise<people_msgs::PositionMeasurement>("people_tracker_measurements", 10);
  static_belief_map_pub=nh_.advertise<nav_msgs::OccupancyGrid>("/camera_region_map", 10, true);
  setNavTarget_pub=nh_.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/move/goal",50,true);

  openpose_sub=nh_.subscribe<geometry_msgs::PoseArray>("/openpose_pose_array", 10, &op_filter_manager::openpose_pose_callback,this);
  chair_sub=nh_.subscribe<visualization_msgs::MarkerArray>("/chair_boxes", 10, &op_filter_manager::chair_yolo_callback,this);
  //people_yolo_sub=nh_.subscribe<visualization_msgs::MarkerArray>("/human_boxes", 10, &op_filter_manager::human_yolo_callback,this);
  //edge_leg_sub=nh_.subscribe<geometry_msgs::PoseArray>("/edge_leg_detector", 10, &op_filter_manager::edge_leg_callback,this);
  filter_act_sub=nh_.subscribe<std_msgs::Int8>("/filter_act_cmd", 10, &op_filter_manager::filter_act_callback,this);
  globalpose_sub=nh_.subscribe<geometry_msgs::PoseStamped>("/global_pose",10,&op_filter_manager::global_pose_callback,this);
  keyboard_sub=nh_.subscribe<keyboard::Key>("/keyboard/keydown",10, &op_filter_manager::keyboard_callback,this);
  Scaled_static_map_sub=nh_.subscribe<nav_msgs::OccupancyGrid>("/scaled_static_map", 10, &op_filter_manager::scaled_static_map_callback,this);
  filter_result_sub=nh_.subscribe<people_msgs::PositionMeasurement>("people_tracker_filter", 10,&op_filter_manager::filter_result_callback,this);
  wrist_trigger_sub=nh_.subscribe<std_msgs::Int8>("/cmd_trackhuman", 10,&op_filter_manager::wrist_trigger_callback,this);

    
  m_service_client = nh_.serviceClient<villa_navi_service::GoTargetPos>("/navi_go_base");
  // One_People_pos_pub=nh_.advertise<people_msgs::PositionMeasurement>("/people_tracker_measurements", 0 );
 
  global_pose.resize(3,0.0);
  leg_target.resize(2,0.0);
  Head_Pos.resize(2,0.0);
  filtered_leg_target.resize(2,0.0);
  NN_laser_target.resize(2,0.0);
  last_nav_target.resize(2,0.0);

  //camera region
  static_belief_map.info.width=30;
  static_belief_map.info.height= 30;
  static_belief_map.info.resolution=0.5;
  static_belief_map.info.origin.position.x=-7.5;
  static_belief_map.info.origin.position.y=-7.5;
  belief_size=static_belief_map.info.width*static_belief_map.info.height;
  static_belief_map.data.resize(static_belief_map.info.width*static_belief_map.info.height);
  int belief_map_size=static_belief_map.info.width*static_belief_map.info.height;
  for(int k(0);k<belief_map_size;k++)
    static_belief_map.data[k]=0.01;

}

// destructor
op_filter_manager::~op_filter_manager()
{
  // delete sequencer

  // delete all trackers
};



void op_filter_manager::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{

  Head_Pos[0]=msg->position[9];     //pan
  Head_Pos[1]=msg->position[10];      //tilt
 

}

void op_filter_manager::filter_result_callback(const people_msgs::PositionMeasurement::ConstPtr& msg)
{

    ROS_INFO("filter_results callback");
    filtered_leg_target[0]=msg->pos.x;
    filtered_leg_target[1]=msg->pos.y;
}

void op_filter_manager::edge_leg_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  int num_leg_detected = msg->poses.size(); 
  std::vector<double> tempVec(2,0.0);
  
  Cur_leg_human.clear();
  
      for(int i(0);i<num_leg_detected;i++)
      {
        geometry_msgs::Vector3Stamped gV, tV;

          gV.vector.x = msg->poses[i].position.x;
          gV.vector.y = msg->poses[i].position.y;
          gV.vector.z = 1.0;

          gV.header.stamp = ros::Time();
          gV.header.frame_id = "/base_range_sensor_link";
          listener.transformVector("/map", gV, tV);

          tempVec[0]=tV.vector.x+global_pose[0];
          tempVec[1]=tV.vector.y+global_pose[1];

          //distance check between two candidates!
          bool IsFarEnough = true;
          bool IsNearfromRobot= false;
          for(int j(0);j<Cur_leg_human.size();j++)
          {
              if(Comparetwopoistions(tempVec,Cur_leg_human[j],0.4))
                  IsFarEnough=false;
          }

          if(Comparetwopoistions(global_pose,tempVec,LASER_Dist_person))
              IsNearfromRobot=true;

          //std::cout<<"here 2"<<std::endl;
          //add only if candidate pose ins far from 0.75 from previous candidates
          if(IsFarEnough && IsNearfromRobot)
          { 
              Cur_leg_human.push_back(tempVec);
          }
         
      }

      // std::cout<<"here 3"<<std::endl;
      if(Cur_leg_human.size()>0)
      { 
        int NearestLegIdx=FindNearesetLegIdx();
          
        std::vector<double> temp_leg_target(2,0.0); 
        temp_leg_target[0]=Cur_leg_human[NearestLegIdx][0];
        temp_leg_target[1]=Cur_leg_human[NearestLegIdx][1];

        if(Comparetwopoistions(temp_leg_target,filtered_leg_target,1.2))
        {
             std::cout<<"update target - person is in a range"<<std::endl;
            NN_laser_target[0]=temp_leg_target[0];
            NN_laser_target[1]=temp_leg_target[1];
        }

      }

}

void op_filter_manager::publish_cameraregion()
{
   getCameraregion();
   static_belief_map.header.stamp =  ros::Time::now();
   static_belief_map.header.frame_id = "map"; 
   static_belief_map_pub.publish(static_belief_map);

}


void op_filter_manager::getCameraregion()
{

  double global_robot_x= global_pose[0];
  double global_robot_y= global_pose[1];
  double global_robot_theta = global_pose[2]+Head_Pos[0];

  visiblie_idx_set.clear();

  global_robot_theta=0.0;
  //Iteration for belief grid
  for(int i(0);i<static_belief_map.info.width;i++)
    for(int j(0);j<static_belief_map.info.height;j++)
  {
    int belief_map_idx=j*static_belief_map.info.height+i;

    // double map_ogirin_x = static_belief_map.info.origin.position.x+global_robot_x;
    // double map_ogirin_y = static_belief_map.info.origin.position.y+global_robot_y;

    double map_ogirin_x = static_belief_map.info.origin.position.x;
    double map_ogirin_y = static_belief_map.info.origin.position.y;


    double trans_vector_x=(i+0.5)*static_belief_map.info.resolution;
    double trans_vector_y=(j+0.5)*static_belief_map.info.resolution;

    double rot_trans_vector_x = cos(global_robot_theta)*trans_vector_x-sin(global_robot_theta)*trans_vector_y;
    double rot_trans_vector_y = sin(global_robot_theta)*trans_vector_x+cos(global_robot_theta)*trans_vector_y;

    double belief_global_x=map_ogirin_x+rot_trans_vector_x;
    double belief_global_y=map_ogirin_y+rot_trans_vector_y;

    //solve
    bool line1_result =getlinevalue(1,belief_global_x,belief_global_y);
    bool line2_result =getlinevalue(2,belief_global_x,belief_global_y);


    if( line1_result && line2_result )
    {
      static_belief_map.data[belief_map_idx]=30;  
      visiblie_idx_set.push_back(belief_map_idx);         //save cell_id 
    }
    else
      static_belief_map.data[belief_map_idx]=0.0; 
  }



}


bool op_filter_manager::getlinevalue(int line_type,double input_x, double input_y)
{

  double global_robot_theta = global_pose[2]+Head_Pos[0];
  // double global_robot_theta =Camera_angle;
  double theta_1=-FOVW*MATH_PI/180+global_robot_theta;
  double theta_2= FOVW*MATH_PI/180+global_robot_theta;
  
  double m_1=tan(theta_1);
  double m_2=tan(theta_2);

  int isspecial=0;

  if(theta_1<-MATH_PI/2.0 && theta_2 >-MATH_PI/2.0)
  {
    double temp=m_2;
    isspecial=1;
  }
  else if(theta_2> MATH_PI/2.0 && (theta_1 <MATH_PI/2.0))
  {
    isspecial=2;
  }
  else if (theta_1<-MATH_PI/2.0 && theta_2 <-MATH_PI/2.0)
  {
    isspecial=5;
  }
  else if(theta_2< -MATH_PI/2.0)
  {
    isspecial=3;
  }

  else if(theta_1>MATH_PI/2.0 && theta_2> MATH_PI/2.0)
  {
    isspecial=4;  
  }


   // std::cout<<"camera region section : "<<isspecial<<std::endl;
  
  double m=0.0;
  double coeff_sign=1.0;

  double global_robot_x= global_pose[0];
  double global_robot_y= global_pose[1];

  double res =0.0;

  switch(line_type){
  case 1:
      m=m_1;
      coeff_sign=-1.0;

      if(isspecial==0)
          coeff_sign=-1.0;
      else if(isspecial==1)
        coeff_sign=1.0;
      else if(isspecial==2)
        coeff_sign=-1.0;  
      else if(isspecial==4)
        coeff_sign=1.0; 
      else if(isspecial==5)
        coeff_sign=1.0; 

      break;
  case 2:
      m=m_2;
      coeff_sign=-1.0;
      if(isspecial==1)
        coeff_sign=1.0; 
      else if(isspecial==0)
        coeff_sign=1.0; 
      else if(isspecial==3)
        coeff_sign=1.0;
           

      break;
  default:
    std::cout<<"Wrong line type"<<std::endl;
      m=m_1;
    }

  res= m*input_x-m*global_robot_x+global_robot_y-input_y;

  if(res*coeff_sign>0 || res==0)
    return true;
  else
    return false;

}




void op_filter_manager::scaled_static_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("map Received");
    std::cout <<"static_Width: " << msg->info.width << std::endl;
    std::cout <<"static_Height: " << msg->info.height << std::endl;
    std::cout << "static_X origin:" << msg->info.origin.position.x << std::endl;
    std::cout << "static_Y origin:" << msg->info.origin.position.y << std::endl;
    std::cout <<"static_Resolution: " << msg->info.resolution << std::endl;   

    // Copy Data;
    Scaled_map = (*msg);
    Scaled_map.header.stamp =  ros::Time::now();
    Scaled_map.header.frame_id = "map";
    std::cout<<"Here"<<std::endl; 
    // for(int i(0);i<msg->data.size();i++)
    //   std::cout<<msg->data[i]<<endl;

}


void op_filter_manager::wrist_trigger_callback(const std_msgs::Int8::ConstPtr& msg)
{
  int ReceivedNum= (int) msg->data;
  if(ReceivedNum==1)    //if wrist is triggered
  {
    if(pose_people.size()>0)
     {
          leg_target.resize(2,0.0);
          leg_target[0]=pose_people[0][0];
          leg_target[1]=pose_people[0][1];
          OnceTarget=true;
          ROS_INFO("set Target");
          std::cout<<"set target : "<<leg_target[0]<<" , "<<leg_target[1]<<std::endl;
     }
  }
}

void op_filter_manager::keyboard_callback(const keyboard::Key::ConstPtr& msg)
{

  ROS_INFO("Received Keyboard");
  printf("(key board)\n");
  int ReceivedNum= (int) msg->code;
  std::cout<<msg->code<<std::endl;
  if(ReceivedNum==116)    //if keyboard input is "t"
  {
    if(pose_people.size()>0)
     {
          leg_target.resize(2,0.0);
          leg_target[0]=pose_people[0][0];
          leg_target[1]=pose_people[0][1];
          OnceTarget=true;
          ROS_INFO("set Target");
          std::cout<<"set target : "<<leg_target[0]<<" , "<<leg_target[1]<<std::endl;
          // ROS_INFO("Filter : Set Target pos X : %.3lf, y : %.3lf", cur_yolo_people[0],cur_yolo_people[1]);
       }

  }
}


// void op_filter_manager::trigger_callback(const keyboard::Key::ConstPtr& msg)
// {

//   ROS_INFO("Received Keyboard");
//   printf("(key board)\n");
//   int ReceivedNum= (int) msg->code;
//   std::cout<<msg->code<<std::endl;
//   if(ReceivedNum==116)    //if keyboard input is "t"
//   {
//     if(cur_yolo_people.size()>0)
//      {
//           leg_target.resize(2,0.0);
//           leg_target[0]=cur_yolo_people[0][0];
//           leg_target[1]=cur_yolo_people[0][1];
//           OnceTarget=true;
//           ROS_INFO("set Target");
//           std::cout<<"set target : "<<leg_target[0]<<" , "<<leg_target[1]<<std::endl;
//           // ROS_INFO("Filter : Set Target pos X : %.3lf, y : %.3lf", cur_yolo_people[0],cur_yolo_people[1]);
//        }

//   }
// }
bool op_filter_manager::check_chair(float x_pos,float y_pos)
{
    //return true if input positions(x_pos,ypos) are close to the chair positions
   vector<double> tempVec(2,0.0);
   tempVec[0]=x_pos;
   tempVec[1]=y_pos;
   
  //return true if it is occupied with obstacles
  if (cur_yolo_chair.size()>0)
  {  
    for(int i(0);i<cur_yolo_chair.size();i++)
    {
      if(Comparetwopoistions(cur_yolo_chair[i],tempVec,0.22))
         return true;
    }
  }
  else{
  
  return false;
  }

  return false;
}

bool op_filter_manager::check_staticObs(float x_pos,float y_pos)
{
  
  //return true if it is occupied with obstacles
  if (Scaled_map.data.size()>0)
  {   int obs_idx=globalcoord_To_SScaled_map_index(x_pos,y_pos);
    

    if(Scaled_map.data[obs_idx]>0)
        return true;
    else
      return false;
  }

}


bool op_filter_manager::check_cameraregion(float x_pos,float y_pos)
{

  if(abs(x_pos)<7.0 && abs(y_pos)<7.0)
  {
  //return true if it is occupied with obstacles
  if (static_belief_map.data.size()>0)
  {   
    int obs_idx=CoordinateTransform_Global2_staticMap(x_pos,y_pos);
    
    if(obs_idx<static_belief_map.data.size()){
      if(static_belief_map.data[obs_idx]>10.0)
        return true;
      else
        return false;
    }
  }

  }

  return true;
}

void op_filter_manager::call_navi_srv()
{

  //m_service_client = n.serviceClient<villa_navi_service::GoTargetPos>("/navi_go_base");
  if(!Comparetwopoistions(last_nav_target, filtered_leg_target, 0.5))
  {
      villa_navi_service::GoTargetPos navi_srv;
      navi_srv.request.x_from_map=filtered_leg_target[0]-0.3;
      navi_srv.request.y_from_map=filtered_leg_target[1];
      navi_srv.request.theta_from_map=0.2;

      m_service_client.call(navi_srv);

      last_nav_target[0]=filtered_leg_target[0]-0.3;
      last_nav_target[1]=filtered_leg_target[1];
  }

  //villa_navi_service::GoTargetPos navi_srv;
  //navi_srv.request.x_from_map=filtered_leg_target[0]-0.3;
  //navi_srv.request.y_from_map=filtered_leg_target[1];
  //navi_srv.request.theta_from_map=0.2;

  //m_service_client.call(navi_srv);

  //last_nav_target[0]=filtered_leg_target[0]-0.3;
  //last_nav_target[1]=filtered_leg_target[1];

}


void op_filter_manager::Publish_nav_target()
{
  
  if(pub_iters>4000){

    if(OnceTarget){

      if(Comparetwopoistions(global_pose, filtered_leg_target, 0.4))
            return;

      move_base_msgs::MoveBaseActionGoal Navmsgs;
      Navmsgs.header.stamp =  ros::Time::now();
     
      //ROS_INFO("time");
      std::cout<<Navmsgs.header.stamp <<std::endl;
     //Navmsgs.header.frame_id = "map"; 
      Navmsgs.goal.target_pose.header.frame_id = "map";

      Navmsgs.goal.target_pose.pose.position.x=filtered_leg_target[0]-0.3;
      Navmsgs.goal.target_pose.pose.position.y=filtered_leg_target[1];
      Navmsgs.goal.target_pose.pose.position.z=0.5;

      std::vector<double> GoalVector;
      GoalVector.resize(2,0.0);
      GoalVector[0]=filtered_leg_target[0]-0.2;
      GoalVector[1]=filtered_leg_target[1];


      GoalVector[0]=GoalVector[0]-global_pose[0];
      GoalVector[1]=GoalVector[1]-global_pose[1];

      double temp_yaw =atan(GoalVector[1]/GoalVector[0]);
      temp_yaw=temp_yaw-global_pose[2];
      double temp_roll=0.0;
      double temp_pitch=0.0;
      // poses.orientation = tf::transformations.quaternion_from_euler(0.0, 0.0, temp_yaw);
      
      geometry_msgs::Quaternion q;

      double t0 = cos(temp_yaw * 0.5);
      double t1 = sin(temp_yaw * 0.5);
      double t2 = cos(temp_roll * 0.5);
      double t3 = sin(temp_roll * 0.5);
      double t4 = cos(temp_pitch * 0.5);
      double t5 = sin(temp_pitch * 0.5);
      q.w = t0 * t2 * t4 + t1 * t3 * t5;
      q.x = t0 * t3 * t4 - t1 * t2 * t5;
      q.y = t0 * t2 * t5 + t1 * t3 * t4;
      q.z = t1 * t2 * t4 - t0 * t3 * t5;

      // tf::StampedTransform tfs;
      // tf::Quaternion head_orientation =tf::createQuaternionFromRPY(0.0, 0.0, temp_yaw);
      // Navmsgs.goal.target_pose.pose.orientation = head_orientation;

       Navmsgs.goal.target_pose.pose.orientation.x=q.x;
       Navmsgs.goal.target_pose.pose.orientation.y=q.y;
       Navmsgs.goal.target_pose.pose.orientation.z=q.z;
       Navmsgs.goal.target_pose.pose.orientation.w=q.w;

       setNavTarget_pub.publish(Navmsgs);
       ROS_INFO("navgation published");

      pub_iters=0;
     }

  }


  pub_iters++;

}






int op_filter_manager::globalcoord_To_SScaled_map_index(float x_pos,float y_pos)
{
   std::vector<float> cur_coord(2,0.0);
  
   //for case of using static map
  float reference_origin_x =-4;
  float reference_origin_y =-4;
  float Grid_STEP=0.5;
  int num_grid=24;

  //for case of using static map
  // double reference_origin_x =-3.5;
  // double reference_origin_y =-3.5;
  float  temp_x  = x_pos-reference_origin_x;
  float  temp_y = y_pos-reference_origin_y;

  cur_coord[0]= (int) (temp_x/Grid_STEP);
  cur_coord[1]= (int)(temp_y/Grid_STEP);


  int robot_pos_id=num_grid*cur_coord[1]+cur_coord[0];
  //ROS_INFO("Robot pos ID : %d \n", robot_pos_id);

  return robot_pos_id;

}


void op_filter_manager::publish_target()
{

    visualization_msgs::Marker marker_human;
    marker_human.header.frame_id = "/map"; 
    marker_human.header.stamp = ros::Time::now();
    marker_human.id = 0;
    uint32_t shape = visualization_msgs::Marker::SPHERE;
    marker_human.type = shape;

//filtered_leg_target
  //publish marker
    marker_human.pose.position.x = leg_target[0];
    marker_human.pose.position.y = leg_target[1];
    //marker_human.pose.position.x = NN_laser_target[0];
    //marker_human.pose.position.y = NN_laser_target[1];
    marker_human.pose.position.z = 1;

    marker_human.pose.orientation.x = 0.0;
    marker_human.pose.orientation.y = 0.0;
    marker_human.pose.orientation.z = 0.0;
    marker_human.pose.orientation.w = 1.0;

      double temp_dist,temp_dist2,temp_dist3;
      temp_dist  =0.5;
      temp_dist2 =0.5;
      temp_dist3 =0.5;

      //ROS_INFO("temp dist : %.3lf, temp dist2 : %.3lf, temp dist3 : %.3lf",temp_dist,temp_dist2,temp_dist3);
      marker_human.scale.x = std::abs(temp_dist);
      marker_human.scale.y = std::abs(temp_dist2);
      marker_human.scale.z = std::abs(temp_dist3);

      marker_human.color.r = 0.0;
      marker_human.color.g = 0.2;
      marker_human.color.b = 0.8;
      marker_human.color.a = 0.85;

    human_target_pub.publish(marker_human);
    // human_target_Intcmd_pub.publish(track_cmd);  



}


void op_filter_manager::publish_filtered_target()
{
    visualization_msgs::Marker marker_human;
    marker_human.header.frame_id = "/map"; 
    marker_human.header.stamp = ros::Time::now();
    marker_human.id = 0;
    uint32_t shape = visualization_msgs::Marker::SPHERE;
    marker_human.type = shape;


  //publish marker
    marker_human.pose.position.x = filtered_leg_target[0];
      marker_human.pose.position.y = filtered_leg_target[1];
      marker_human.pose.position.z = 1;

      marker_human.pose.orientation.x = 0.0;
      marker_human.pose.orientation.y = 0.0;
      marker_human.pose.orientation.z = 0.0;
      marker_human.pose.orientation.w = 1.0;

      double temp_dist,temp_dist2,temp_dist3;
      temp_dist  =0.5;
      temp_dist2 =0.5;
      temp_dist3 =0.5;

      //ROS_INFO("temp dist : %.3lf, temp dist2 : %.3lf, temp dist3 : %.3lf",temp_dist,temp_dist2,temp_dist3);
      marker_human.scale.x = std::abs(temp_dist);
      marker_human.scale.y = std::abs(temp_dist2);
      marker_human.scale.z = std::abs(temp_dist3);

      marker_human.color.r = 0.0;
      marker_human.color.g = 0.7;
      marker_human.color.b = 0.2;
      marker_human.color.a = 0.85;

    filtered_human_target_pub.publish(marker_human);
    // human_target_Intcmd_pub.publish(track_cmd);  
}


int op_filter_manager::FindNearestPoseIdx()
{
    //target should be already set
    std::vector<double> Distanceset;
    // Distanceset.resize(Cur_detected_human.size(),0.0);
    Distanceset.resize(pose_people.size(),0.0);
    
    double minDistance=200.0;
    int    minDistance_Idx=0;

      for(int i(0);i<pose_people.size();i++)
      {
        // Distanceset[i]=getDistance(Cur_detected_human[i][0],Cur_detected_human[i][1]);
        Distanceset[i]=getDistance_from_Vec(global_pose,pose_people[i][0],pose_people[i][1]);
        
        if(minDistance>Distanceset[i])
          {
            minDistance=Distanceset[i];
            minDistance_Idx=i;
          }
      }
  
    return minDistance_Idx;

}


int op_filter_manager::FindNearesetLegIdx()
{
    //target should be already set
    std::vector<double> Distanceset;
    Distanceset.resize(Cur_leg_human.size(),0.0);
    
    double minDistance=200.0;
    int    minDistance_Idx=0;

      for(int i(0);i<Cur_leg_human.size();i++)
      {
        Distanceset[i]=getDistance_from_Vec(filtered_leg_target,Cur_leg_human[i][0],Cur_leg_human[i][1]);
        
        if(minDistance>Distanceset[i])
          {
            minDistance=Distanceset[i];
            minDistance_Idx=i;
          }
      }
  
    return minDistance_Idx;

}

double op_filter_manager::getDistance_from_Vec(std::vector<double> origin, double _x, double _y)
{
  double temp=0.0;

  temp=(origin[0]-_x)*(origin[0]-_x);
  temp+=(origin[1]-_y)*(origin[1]-_y);
  temp=sqrt(temp);

  return temp;

}


void op_filter_manager::filter_act_callback(const std_msgs::Int8::ConstPtr& msg)
{

    if(msg->data==1)
    {
      if(cur_yolo_people.size()>0)
        {
          leg_target.resize(2,0.0);
          leg_target[0]=cur_yolo_people[0][0];
          leg_target[1]=cur_yolo_people[0][1];
          // ROS_INFO("set Target");
          std::cout<<leg_target[0]<<", "<<leg_target[1]<<std::endl;
          // ROS_INFO("Filter : Set Target pos X : %.3lf, y : %.3lf", cur_yolo_people[0],cur_yolo_people[1]);
        }
    }

}

void op_filter_manager::global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
   global_pose[0]=msg->pose.position.x;
   global_pose[1]=msg->pose.position.y;

   tf::StampedTransform baselinktransform;
   listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(2.0));
   listener.lookupTransform("map", "base_link", ros::Time(0), baselinktransform);
   double yaw_tf =   tf::getYaw(baselinktransform.getRotation()); 

    global_pose[2]=yaw_tf;
}

void op_filter_manager::chair_yolo_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    std::cout<<"chair recieved"<<std::endl;
    num_of_detected_chair=msg->markers.size();

    if(num_of_detected_chair>0)
       cur_yolo_chair.resize(num_of_detected_chair);
    else
    {
      return;
    }

    for(int i(0);i<num_of_detected_chair;i++)
    {
      geometry_msgs::Vector3Stamped gV, tV;

      gV.vector.x = msg->markers[i].pose.position.x;
      gV.vector.y = msg->markers[i].pose.position.y;
      gV.vector.z = msg->markers[i].pose.position.z;

      // std::cout<<"x :"<<_x<<"_y:"<<_y<<"_z:"<<_z<<std::endl;
      tf::StampedTransform maptransform;
      listener.waitForTransform("head_rgbd_sensor_rgb_frame", "map", ros::Time(0), ros::Duration(1.0));
              
      gV.header.stamp = ros::Time();
      gV.header.frame_id = "/head_rgbd_sensor_rgb_frame";
      listener.transformVector(std::string("/map"), gV, tV);
              
      cur_yolo_chair[i].resize(2,0.0);
      cur_yolo_chair[i][0]=tV.vector.x+global_pose[0];
      cur_yolo_chair[i][1]=tV.vector.y+global_pose[1];
   }
}

void op_filter_manager::openpose_pose_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
        //openpose comes w.r.t map frame
        //ROS_INFO("openposes callback: poses size : %d ", msg->poses.size());
        //human_op_poses_array = *msg;
        std::vector<double> temp_target(2,0.0); 
        int num_of_detected_human= msg->poses.size(); 
        if(num_of_detected_human>0)
        {
            pose_people.resize(num_of_detected_human);
            for(int i(0);i<num_of_detected_human;i++)
            {
                pose_people[i].resize(2,0.0);
                pose_people[i][0]=msg->poses[i].position.x;
                pose_people[i][1]=msg->poses[i].position.y;
            }

            int NearestPoseIdx=FindNearestPoseIdx();

            std::vector<double> temp_target(2,0.0); 
            temp_target[0]=pose_people[NearestPoseIdx][0];
            temp_target[1]=pose_people[NearestPoseIdx][1];

            if(OnceTarget){

                people_msgs::PositionMeasurement pos;
                pos.header.stamp = ros::Time();
                pos.header.frame_id = "/map";
                pos.name = "leg_laser";
                pos.object_id ="person 0";
                pos.pos.x = temp_target[0];
                pos.pos.y = temp_target[1];
                pos.pos.z = 1.0;
                pos.reliability = 0.85;
                pos.covariance[0] = pow(0.01 / pos.reliability, 2.0);
                pos.covariance[1] = 0.0;
                pos.covariance[2] = 0.0;
                pos.covariance[3] = 0.0;
                pos.covariance[4] = pow(0.01 / pos.reliability, 2.0);
                pos.covariance[5] = 0.0;
                pos.covariance[6] = 0.0;
                pos.covariance[7] = 0.0;
                pos.covariance[8] = 10000.0;
                pos.initialization = 0;

                people_measurement_pub_.publish(pos);
            }

            leg_target[0]=temp_target[0];
            leg_target[1]=temp_target[1];


        }  
        else if(Cur_leg_human.size()>0){

         //std::vector<double> temp_target(2,0.0); 
         //temp_target[0]=NN_laser_target[0];
         //temp_target[1]=NN_laser_target[1];

         //people_msgs::PositionMeasurement pos;
            //pos.header.stamp = ros::Time();
            //pos.header.frame_id = "/map";
            //pos.name = "leg_laser";
            //pos.object_id ="person 0";
            //pos.pos.x = temp_target[0];
            //pos.pos.y = temp_target[1];
            //pos.pos.z = 1.0;
            //pos.reliability = 0.85;
            //pos.covariance[0] = pow(0.01 / pos.reliability, 2.0);
            //pos.covariance[1] = 0.0;
            //pos.covariance[2] = 0.0;
            //pos.covariance[3] = 0.0;
            //pos.covariance[4] = pow(0.01 / pos.reliability, 2.0);
            //pos.covariance[5] = 0.0;
            //pos.covariance[6] = 0.0;
            //pos.covariance[7] = 0.0;
            //pos.covariance[8] = 10000.0;
            //pos.initialization = 0;

            //people_measurement_pub_.publish(pos);

            //leg_target[0]=temp_target[0];
            //leg_target[1]=temp_target[1];


        }
        else
        {
            return;
        }


      //if(pose_people.size()>0)
      //{ 

         //if(Comparetwopoistions(temp_target,leg_target,1.2))
         //{
              //std::cout<<"update target - person is in a range"<<std::endl;
             //leg_target[0]=temp_target[0];
             //leg_target[1]=temp_target[1];
         //}

        //if(OnceTarget){
        
            //people_msgs::PositionMeasurement pos;
            //pos.header.stamp = ros::Time();
            //pos.header.frame_id = "/map";
            //pos.name = "leg_laser";
            //pos.object_id ="person 0";
            //pos.pos.x = temp_target[0];
            //pos.pos.y = temp_target[1];
            //pos.pos.z = 1.0;
            //pos.reliability = 0.85;
            //pos.covariance[0] = pow(0.01 / pos.reliability, 2.0);
            //pos.covariance[1] = 0.0;
            //pos.covariance[2] = 0.0;
            //pos.covariance[3] = 0.0;
            //pos.covariance[4] = pow(0.01 / pos.reliability, 2.0);
            //pos.covariance[5] = 0.0;
            //pos.covariance[6] = 0.0;
            //pos.covariance[7] = 0.0;
            //pos.covariance[8] = 10000.0;
            //pos.initialization = 0;

            //people_measurement_pub_.publish(pos);

            //leg_target[0]=temp_target[0];
            //leg_target[1]=temp_target[1];

      //}



}



void op_filter_manager::human_yolo_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{

    num_of_detected_human=msg->markers.size();

    if(num_of_detected_human>0)
       cur_yolo_people.resize(num_of_detected_human);
    else
    {
      return;
    }

    for(int i(0);i<num_of_detected_human;i++)
    {
      //geometry_msgs::Vector3Stamped gV, tV;

      //gV.vector.x = msg->markers[i].pose.position.x;
      //gV.vector.y = msg->markers[i].pose.position.y;
      //gV.vector.z = msg->markers[i].pose.position.z;

       //std::cout<<"x :"<<_x<<"_y:"<<_y<<"_z:"<<_z<<std::endl;
      //tf::StampedTransform maptransform;
      //listener.waitForTransform("head_rgbd_sensor_rgb_frame", "map", ros::Time(0), ros::Duration(1.0));
              
      //gV.header.stamp = ros::Time();
      //gV.header.frame_id = "/head_rgbd_sensor_rgb_frame";
      //listener.transformVector(std::string("/map"), gV, tV);
      
      //cur_yolo_people[i].resize(2,0.0);
      //cur_yolo_people[i][0]=tV.vector.x+global_pose[0];
      //cur_yolo_people[i][1]=tV.vector.y+global_pose[1];

       double time_sec = 0.0;
       geometry_msgs::PoseStamped poseInOriginCoords;
       geometry_msgs::PoseStamped poseInTargetCoords;
       //poseInTargetCoords.header.frame_id = target_frame;
       poseInOriginCoords.header.frame_id = "head_rgbd_sensor_rgb_frame";
       poseInOriginCoords.header.stamp= ros::Time::now();
       poseInOriginCoords.pose = msg->markers[i].pose;
      
       try{
           ROS_DEBUG("Transforming received position into map frame");
           //listener.waitForTransform(poseInTargetCoords.header.frame_id, BASE_LINK, poseInTargetCoords.header.stamp, ros::Duration(3.0));
           listener.waitForTransform("head_rgbd_sensor_rgb_frame", "map", ros::Time(0), ros::Duration(1.0));
           listener.transformPose("map", ros::Time(0), poseInOriginCoords, poseInOriginCoords.header.frame_id, poseInTargetCoords);

       }catch(tf::TransformException ex){
       
           ROS_WARN("Failed transform: %s", ex.what());
           continue;
       }

       cur_yolo_people[i].resize(2,0.0);
       cur_yolo_people[i][0]=poseInTargetCoords.pose.position.x;
       cur_yolo_people[i][1]=poseInTargetCoords.pose.position.y;

   }
    //ROS_INFO("yolo size");

   // printf("size yolo : %d \n",cur_yolo_people.size());
}

// callback for messages
void op_filter_manager::callbackRcv(const people_msgs::PositionMeasurement::ConstPtr& msg)
{
  


}


void op_filter_manager::publish_leg_boxes()
{


  human_leg_boxes_array.markers.clear();
  human_leg_poses_array.poses.clear();

  if(Cur_leg_human.size()>0)
  {
    for(int i(0);i<Cur_leg_human.size();i++)
    {
        //geometry_msgsr
        geometry_msgs::Pose pose_human_leg;

        pose_human_leg.position.x=Cur_leg_human[i][0];
        pose_human_leg.position.y=Cur_leg_human[i][1];
        pose_human_leg.position.z=0.5;
        pose_human_leg.orientation.x=0.0;
        pose_human_leg.orientation.y=0.0;
        pose_human_leg.orientation.z=0.0;
        pose_human_leg.orientation.w=1.0;
        human_leg_poses_array.poses.push_back(pose_human_leg);
        //human_leg_poses_array.header.stamp=ros::Time::now();

        //visualization_marker
        visualization_msgs::Marker marker_human_leg;

        marker_human_leg.header.frame_id = "/map"; 
        marker_human_leg.header.stamp = ros::Time::now();
        marker_human_leg.ns = "/human_leg_boxes";
        marker_human_leg.id = i;

        uint32_t shape = visualization_msgs::Marker::SPHERE;
        marker_human_leg.type = shape;

        marker_human_leg.pose.position.x = Cur_leg_human[i][0];
        marker_human_leg.pose.position.y = Cur_leg_human[i][1];
        marker_human_leg.pose.position.z = 1;

        marker_human_leg.pose.orientation.x = 0.0;
        marker_human_leg.pose.orientation.y = 0.0;
        marker_human_leg.pose.orientation.z = 0.0;
        marker_human_leg.pose.orientation.w = 1.0;

        double temp_dist,temp_dist2,temp_dist3;
        temp_dist  =0.5;
        temp_dist2 =0.5;
        temp_dist3 =0.5;

        //ROS_INFO("temp dist : %.3lf, temp dist2 : %.3lf, temp dist3 : %.3lf",temp_dist,temp_dist2,temp_dist3);
        marker_human_leg.scale.x = std::abs(temp_dist);
        marker_human_leg.scale.y = std::abs(temp_dist2);
        marker_human_leg.scale.z = std::abs(temp_dist3);

        marker_human_leg.color.r = 0.83;
        marker_human_leg.color.g = 0.6;
        marker_human_leg.color.b = 0.5;
        marker_human_leg.color.a = 0.85;

        human_leg_boxes_array.markers.push_back(marker_human_leg);
      }

      Leg_boxes_pub.publish(human_leg_boxes_array);
      human_leg_poses_array.header.stamp=ros::Time::now();
      human_leg_poses_array.header.frame_id="map";
      Leg_poses_pub.publish(human_leg_poses_array);
  }
}

int op_filter_manager::CoordinateTransform_Global2_staticMap(float global_x, float global_y)
{
  double reference_origin_x=static_belief_map.info.origin.position.x;
  double reference_origin_y=static_belief_map.info.origin.position.y;

  //Find the coordinate w.r.t map origin
  double  temp_x  = global_x - reference_origin_x;
  double  temp_y  = global_y - reference_origin_y;

  //Find the map cell idx for x, y
  std::vector<int> human_coord(2,0);
  human_coord[0]= (int) (temp_x/static_belief_map.info.resolution);
  human_coord[1]= (int) (temp_y/static_belief_map.info.resolution);

  //Find the map index from cell x, y
  int static_map_idx= human_coord[0]+static_belief_map.info.width*human_coord[1];

  // std::cout<<"map_idx : "<<static_map_idx<<std::endl;
  return static_map_idx;
   
}

bool op_filter_manager::Comparetwopoistions(std::vector<double> pos,std::vector<double> pos2, double criterion)
{
  //return true if there are in criterion distance 
  double temp_dist=0.0;
  for(int i(0);i<2;i++) 
  {
    temp_dist+=pow((pos[i]-pos2[i]),2);
  }

  temp_dist=sqrt(temp_dist);

  if(temp_dist<criterion)
    return true;
  

  return false;

}

// filter loop
void op_filter_manager::spin()
{
  ROS_INFO("People tracking manager started.");

  while (ros::ok())
  {
    publish_leg_boxes();
    publish_cameraregion();
    publish_target();
    publish_filtered_target();
    //call_navi_srv();
    Publish_nav_target();
    // ------ LOCKED ------
    boost::mutex::scoped_lock lock(filter_mutex_);
    lock.unlock();
    // ------ LOCKED ------

    // sleep
    usleep(1e6 / freq_);

    ros::spinOnce();
  }
};


// ----------
// -- MAIN --
// ----------
int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "op_filter_tracker");
  ros::NodeHandle(nh);

  // create tracker node
  op_filter_manager op_filter_node(nh);

  // wait for filter to finish
  op_filter_node.spin();
  // Clean up

  return 0;
}
