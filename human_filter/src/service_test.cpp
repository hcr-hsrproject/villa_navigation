/*********************************************************************
Human_filter_service
sending kalman filter to 
*********************************************************************
/* Author: Minkyu Kim : steveminq@gmail.com*/

#include "people_tracking_filter/service_test.h"
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
Edgeleg_manager_srv::Edgeleg_manager_srv(ros::NodeHandle nh)
  : nh_(nh),
    robot_state_(),
    filter_counter_(0),
    edge_leg_iter(0),
    pub_iters(0),
    OnceTarget(false)
{
  // initialize
  // advertise filter output

  //markerarrary for candidates of human marker
  Leg_boxes_pub=nh_.advertise<visualization_msgs::MarkerArray>("/filtered_leg_target", 10);
  
  human_target_pub=nh_.advertise<visualization_msgs::Marker>("/leg_target", 10);
  //filtered reuslts of laser detection
  filtered_human_target_pub=nh_.advertise<visualization_msgs::Marker>("/filtered_target", 10);
  //measurement information which is sending to kalman filter 
  people_measurement_pub_ = nh_.advertise<people_msgs::PositionMeasurement>("people_tracker_measurements", 10);
  //camera region 
  static_belief_map_pub=nh_.advertise<nav_msgs::OccupancyGrid>("/camera_region_map", 10, true);
  //navigation target
  setNavTarget_pub=nh_.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/move/goal",50,true);
  //GazePoint target
  Gaze_point_pub= nh_.advertise<geometry_msgs::Point>("/gazed_point_fixing_node/target_point", 50, true);

  //subscribe for yolo detection
  people_yolo_sub=nh_.subscribe<visualization_msgs::MarkerArray>("/human_boxes", 10, &Edgeleg_manager_srv::human_yolo_callback,this);
  //subscribe for laser detection
  edge_leg_sub=nh_.subscribe<geometry_msgs::PoseArray>("/edge_leg_detector", 10, &Edgeleg_manager_srv::edge_leg_callback,this);
  //activation filtering
  filter_act_sub=nh_.subscribe<std_msgs::Int8>("/filter_act_cmd", 10, &Edgeleg_manager_srv::filter_act_callback,this);
  //global pose call back - w.r.t map frame
  globalpose_sub=nh_.subscribe<geometry_msgs::PoseStamped>("/global_pose",10,&Edgeleg_manager_srv::global_pose_callback,this);
  //keyboard callback
  keyboard_sub=nh_.subscribe<keyboard::Key>("/keyboard/keydown",10, &Edgeleg_manager_srv::keyboard_callback,this);
  //scaled_map
  Scaled_static_map_sub=nh_.subscribe<nav_msgs::OccupancyGrid>("/scaled_static_map", 10, &Edgeleg_manager_srv::scaled_static_map_callback,this);
  //filtering result from kalman filter
  filter_result_sub=nh_.subscribe<people_msgs::PositionMeasurement>("people_tracker_filter", 10,&Edgeleg_manager_srv::filter_result_callback,this);
  //wrist_trigger_subscriber
  wrist_trigger_sub=nh_.subscribe<std_msgs::Int8>("/cmd_trackhuman", 10,&Edgeleg_manager_srv::wrist_trigger_callback,this);
    
  // Create service for activation following
  m_service = nh_.advertiseService("/following_human",  &Edgeleg_manager_srv::followingLoop,this);
 
  last_global_pose.resize(3,0.0);
  global_pose.resize(3,0.0);
  leg_target.resize(2,0.0);
  Head_Pos.resize(2,0.0);
  filtered_leg_target.resize(2,0.0);

  //camera region map setting
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
Edgeleg_manager_srv::~Edgeleg_manager_srv()
{
  // delete sequencer

  // delete all trackers
};


void Edgeleg_manager_srv::check_IsrobotMoving()
{
  //This function should be actviate only when robot is following person
  //We have to chek if robot is freezing or not
  //If we loose people, we have to reset based on yolo detection
  //robot_last position
  //robot_current_position
  //getdistance between last_position, current position
  //This should be combined with human interaction

  // last_globasl_pose_t

  
}



void Edgeleg_manager_srv::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{

  Head_Pos[0]=msg->position[9];     //pan
  Head_Pos[1]=msg->position[10];      //tilt
 

}

void Edgeleg_manager_srv::filter_result_callback(const people_msgs::PositionMeasurement::ConstPtr& msg)
{
      filtered_leg_target[0]=msg->pos.x;
      filtered_leg_target[1]=msg->pos.y;
}

void Edgeleg_manager_srv::edge_leg_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  // Function for subscribe laser leg detection results
  // Receive all the possilble candiates of leg detection
  //  1) Remove when map occupancy is filled with obstacles 
  //  2) Remove when map position is too far from robot % is in the range of camera
  //  3)
  //   

  int num_leg_detected = msg->poses.size(); 
  std::vector<double> tempVec(2,0.0);
  Cur_leg_human.clear();
  
  for(int i(0);i<num_leg_detected;i++)
    {
      geometry_msgs::Vector3Stamped gV, tV;

      gV.vector.x = msg->poses[i].position.x;
      gV.vector.y = msg->poses[i].position.y;
      gV.vector.z = 1.0;

      while (ros::ok()){

      try{
            listener.waitForTransform("base_range_sensor_link", "map", ros::Time(0), ros::Duration(3.0));
            gV.header.stamp = ros::Time();
            gV.header.frame_id = "/base_range_sensor_link";
            listener.transformVector(std::string("/map"), gV, tV);
          }
          //keep trying until we get the transform
          catch (tf::TransformException ex){
            ROS_ERROR_THROTTLE(1, "%s",ex.what());
            ROS_WARN_THROTTLE(1,"    Waiting for transform from base_range_sensor_link to map frame. Trying again");
            continue;
          }
      } 

          //chane coordination from "base_range_sensor_link" to "map" 
          tempVec[0]=tV.vector.x+global_pose[0];
          tempVec[1]=tV.vector.y+global_pose[1];

          //check_staticObs - (1)
          if(check_staticObs(tempVec[0],tempVec[1]))
            continue;
          //check_camera region - (2)
          if(!check_cameraregion(tempVec[0],tempVec[1]))
            continue;
         
          //ceck the distance between two candidates!
          bool IsFarEnough = true;
          bool IsNearfromRobot= false;
        
          for(int j(0);j<Cur_leg_human.size();j++)
          {
             if(Comparetwopoistions(tempVec,Cur_leg_human[j],0.65))
                 IsFarEnough=false;
          }
            
          if(Comparetwopoistions(global_pose,tempVec,LASER_Dist_person))
             IsNearfromRobot=true;

          // std::cout<<"here 2"<<std::endl;
          //add only if candidate pose ins far from 0.75 from previous candidates
          if(IsFarEnough && IsNearfromRobot)
          { 
           Cur_leg_human.push_back(tempVec);
          }
         
      }


      // std::cout<<"here 3"<<std::endl;
      if(Cur_leg_human.size()>0)
      {
        //find the nearest laser leg candidates from the target
        int NearestLegIdx=FindNearesetLegIdx();
          
        std::vector<double> temp_leg_target(2,0.0); 
        temp_leg_target[0]=Cur_leg_human[NearestLegIdx][0];
        temp_leg_target[1]=Cur_leg_human[NearestLegIdx][1];

        //Check the distance between the target and candidates
        if(Comparetwopoistions(temp_leg_target,leg_target,0.7))
        {
            // std::cout<<"update target - person is in a range"<<std::endl;
          leg_target[0]=temp_leg_target[0];
          leg_target[1]=temp_leg_target[1];
        }

        //Making a messege for kalman filtering (sending ros msg after setting the target)  
        if(OnceTarget){
          people_msgs::PositionMeasurement pos;
          pos.header.stamp = ros::Time();
          pos.header.frame_id = "/map";
          pos.name = "leg_laser";
          pos.object_id ="person 0";
          pos.pos.x = leg_target[0];
          pos.pos.y = leg_target[1];
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

    }
}

void Edgeleg_manager_srv::publish_cameraregion()
{
   getCameraregion();
   static_belief_map.header.stamp =  ros::Time::now();
   static_belief_map.header.frame_id = "map"; 
   static_belief_map_pub.publish(static_belief_map);
}


void Edgeleg_manager_srv::getCameraregion()
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


bool Edgeleg_manager_srv::getlinevalue(int line_type,double input_x, double input_y)
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

void Edgeleg_manager_srv::wrist_trigger_callback(const std_msgs::Int8::ConstPtr& msg)
{

  ROS_INFO("Received trigger ");
  int ReceivedNum= (int) msg->data;
  if(ReceivedNum==1)    //if keyboard input is "t"
  {
    if(cur_yolo_people.size()>0)
     {
          leg_target.resize(2,0.0);
          leg_target[0]=cur_yolo_people[0][0];
          leg_target[1]=cur_yolo_people[0][1];
          OnceTarget=true;
          ROS_INFO("set Target");
          std::cout<<"set target : "<<leg_target[0]<<" , "<<leg_target[1]<<std::endl;
     }
  }
}


void Edgeleg_manager_srv::scaled_static_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
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

void Edgeleg_manager_srv::keyboard_callback(const keyboard::Key::ConstPtr& msg)
{

  ROS_INFO("Received Keyboard");
  printf("(key board)\n");
  int ReceivedNum= (int) msg->code;
  std::cout<<msg->code<<std::endl;
  if(ReceivedNum==116)    //if keyboard input is "t"
  {
    if(cur_yolo_people.size()>0)
     {
          leg_target.resize(2,0.0);
          leg_target[0]=cur_yolo_people[0][0];
          leg_target[1]=cur_yolo_people[0][1];
          OnceTarget=true;
          ROS_INFO("set Target");
          std::cout<<"set target : "<<leg_target[0]<<" , "<<leg_target[1]<<std::endl;
          // ROS_INFO("Filter : Set Target pos X : %.3lf, y : %.3lf", cur_yolo_people[0],cur_yolo_people[1]);
      }

  }
}

bool Edgeleg_manager_srv::check_staticObs(float x_pos,float y_pos)
{
  
  //if there is no static obstacle, we can set the target
  //return true if it is occupied with obstacles
  if (Scaled_map.data.size()>0)
  { 
    int obs_idx=globalcoord_To_SScaled_map_index(x_pos,y_pos);
    if(Scaled_map.data[obs_idx]>0)
        return true;
    else
      return false;
  }

  //Fixme : try to add return ?
  //Do we use static map ? 
  //this function can be ignored when we are not using mdp_navi which made scaled static map
}


bool Edgeleg_manager_srv::check_cameraregion(float x_pos,float y_pos)
{
  //function for checking camear region

  if(abs(x_pos)<MAP_MAX_X && abs(y_pos)<MAP_MAX_Y)
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


void Edgeleg_manager_srv::Publish_nav_target()
{
  //Discussion : Is it more natural that robot should see the view point target?

  if(pub_iters>3500){
    
    if(OnceTarget){
      move_base_msgs::MoveBaseActionGoal Navmsgs;
      Navmsgs.header.stamp =  ros::Time::now();
     
      ROS_INFO("time");
      std::cout<<Navmsgs.header.stamp <<std::endl;
     //Navmsgs.header.frame_id = "map"; 
      Navmsgs.goal.target_pose.header.frame_id = "map";

       Navmsgs.goal.target_pose.pose.position.x=filtered_leg_target[0]-0.3;
       Navmsgs.goal.target_pose.pose.position.y=filtered_leg_target[1];
       Navmsgs.goal.target_pose.pose.position.z=0.5;

      std::vector<double> GoalVector;
      GoalVector.resize(2,0.0);
      GoalVector[0]=filtered_leg_target[0]-0.3;
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


void Edgeleg_manager_srv::setViewpointTarget(const std::vector<double> pos)
{
  
  geometry_msgs::Point GazePoint_msg;
  if(pos[0]==0.0 && pos[1]==0.0)
  {
      GazePoint_msg.x=2.0;
      GazePoint_msg.y=0.0;  
  }
  else
  {
      geometry_msgs::Vector3Stamped gV, tV;
      gV.vector.x = leg_target[0];
      gV.vector.y = leg_target[1];
      gV.vector.z = 1.0;

      gV.header.stamp = ros::Time();
      gV.header.frame_id = "/map";
      listener.transformVector("/base_range_sensor_link", gV, tV);

      std::vector<double> tempVec(3,0.0);
      tempVec[0]=tV.vector.x;
      tempVec[1]=tV.vector.y;
      tempVec[2]=tV.vector.z;

      GazePoint_msg.x=tempVec[0];
      GazePoint_msg.y=tempVec[1];
      GazePoint_msg.z=tempVec[2];
  }

  GazePoint_msg.z=1.0;
  Gaze_point_pub.publish(GazePoint_msg);




}











int Edgeleg_manager_srv::globalcoord_To_SScaled_map_index(float x_pos,float y_pos)
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


void Edgeleg_manager_srv::publish_target()
{

    visualization_msgs::Marker marker_human;
    marker_human.header.frame_id = "/map"; 
    marker_human.header.stamp = ros::Time::now();
    marker_human.id = 0;
    uint32_t shape = visualization_msgs::Marker::SPHERE;
    marker_human.type = shape;


  //publish marker
    marker_human.pose.position.x = leg_target[0];
      marker_human.pose.position.y = leg_target[1];
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

void Edgeleg_manager_srv::publish_filtered_target()
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

int Edgeleg_manager_srv::FindNearesetLegIdx()
{
    //target should be already set
    std::vector<double> Distanceset;
    // Distanceset.resize(Cur_detected_human.size(),0.0);
    Distanceset.resize(Cur_leg_human.size(),0.0);
    
    double minDistance=200.0;
    int    minDistance_Idx=0;

      for(int i(0);i<Cur_leg_human.size();i++)
      {
        // Distanceset[i]=getDistance(Cur_detected_human[i][0],Cur_detected_human[i][1]);
        Distanceset[i]=getDistance_from_Vec(leg_target,Cur_leg_human[i][0],Cur_leg_human[i][1]);
        
        if(minDistance>Distanceset[i])
          {
            minDistance=Distanceset[i];
            minDistance_Idx=i;
          }
      }
  
    return minDistance_Idx;

}

double Edgeleg_manager_srv::getDistance_from_Vec(std::vector<double> origin, double _x, double _y)
{
  double temp=0.0;

  temp=(origin[0]-_x)*(origin[0]-_x);
  temp+=(origin[1]-_y)*(origin[1]-_y);
  temp=sqrt(temp);

  return temp;

}


void Edgeleg_manager_srv::filter_act_callback(const std_msgs::Int8::ConstPtr& msg)
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

double Edgeleg_manager_srv::get_Distance_3d(std::vector<double> pose1,std::vector<double> pose2)
{
  double temp_dist=0.0;
  int pose1_size=pose1.size();
  int pose2_size=pose1.size();
  if(pose1_size==pose2_size)
  {
    temp_dist=0.0;
    for(int i(0);i<3;i++) 
    {
      temp_dist+=pow((pose1[i]-pose2[i]),2);
    }
  }

  temp_dist=sqrt(temp_dist);

  return temp_dist;
}

void Edgeleg_manager_srv::global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
   //Save current time
   double current_time = 0.0;
   //ros::Time();
  
   global_pose[0]=msg->pose.position.x;
   global_pose[1]=msg->pose.position.y;

   tf::StampedTransform baselinktransform;
   while (ros::ok()){

        try{
        // Look up transform
            listener.lookupTransform("map", "base_link", ros::Time(0), baselinktransform);
            double yaw_tf =   tf::getYaw(baselinktransform.getRotation());
            global_pose[2]=yaw_tf; 
        }
        //keep trying until we get the transform
        catch (tf::TransformException ex){
          ROS_ERROR_THROTTLE(1, "%s",ex.what());
          ROS_WARN_THROTTLE(1,"    Waiting for transform from map to base_link frame. Trying again");
          continue;
        }
  }

  ///////////////////////////////////flight////////////////////////
  //Check the moving distance and they reset the target
  if(OnceTarget)
  {
      double move_distance = get_Distance_3d(last_global_pose,global_pose);
      if(move_distance<0.01)   
      { 
        //when robot has not moved since last callback
        double time_not_move= current_time-last_global_pose_t;
        if(time_not_move>10.0)
          {
            ROS_INFO("Robot has not moved for 15 seconds");
            resetTargetbasedYolo();
          }

      }
      else
      {
        //robot is moved
        //Fixme
        //last_global_pose_t=static_cast<double> ros::Time().now();  
        last_global_pose_t=140;
      }

  }
  
  for(int i(0);i<3;i++)
    last_global_pose[i] = global_pose[i];
  ///////////////////////////////////flight////////////////////////
}


void Edgeleg_manager_srv::resetTargetbasedYolo()
{
  
  if(cur_yolo_people.size()>0)
  {

        leg_target.resize(2,0.0);
        leg_target[0]=cur_yolo_people[0][0];
        leg_target[1]=cur_yolo_people[0][1];
        OnceTarget=true;
        ROS_INFO("reset Target");
        std::cout<<"set target : "<<leg_target[0]<<" , "<<leg_target[1]<<std::endl;
          // ROS_INFO("Filter : Set Target pos X : %.3lf, y : %.3lf", cur_yolo_people[0],cur_yolo_people[1]);
  }

  //based_yolodetection


}

void Edgeleg_manager_srv::human_yolo_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
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
      geometry_msgs::Vector3Stamped gV, tV;

      gV.vector.x = msg->markers[i].pose.position.x;
      gV.vector.y = msg->markers[i].pose.position.y;
      gV.vector.z = msg->markers[i].pose.position.z;

      // std::cout<<"x :"<<_x<<"_y:"<<_y<<"_z:"<<_z<<std::endl;
      tf::StampedTransform maptransform;
      while (ros::ok()){

        try{
            listener.waitForTransform("head_rgbd_sensor_rgb_frame", "map", ros::Time(0), ros::Duration(1.0));
          gV.header.stamp = ros::Time();
          gV.header.frame_id = "/head_rgbd_sensor_rgb_frame";
          listener.transformVector(std::string("/map"), gV, tV);
        }
        //keep trying until we get the transform
        catch (tf::TransformException ex){
          ROS_ERROR_THROTTLE(1, "%s",ex.what());
          ROS_WARN_THROTTLE(1,"    Waiting for transform from head_rgbd_sensor_rgb_frame to map frame. Trying again");
          continue;
        }
  } 

              
      cur_yolo_people[i].resize(2,0.0);
      cur_yolo_people[i][0]=tV.vector.x+global_pose[0];
      cur_yolo_people[i][1]=tV.vector.y+global_pose[1];
   }
   // ROS_INFO("yolo size");

   // printf("size yolo : %d \n",cur_yolo_people.size());
}

// callback for messages
void Edgeleg_manager_srv::callbackRcv(const people_msgs::PositionMeasurement::ConstPtr& msg)
{
  


}


void Edgeleg_manager_srv::publish_leg_boxes()
{


  human_leg_boxes_array.markers.clear();
  if(Cur_leg_human.size()>0)
  {
    for(int i(0);i<Cur_leg_human.size();i++)
    {
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
  }
}

int Edgeleg_manager_srv::CoordinateTransform_Global2_staticMap(float global_x, float global_y)
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

bool Edgeleg_manager_srv::Comparetwopoistions(std::vector<double> pos,std::vector<double> pos2, double criterion)
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

bool Edgeleg_manager_srv::followingLoop(human_filter::set_target_to_follow::Request &req, human_filter::set_target_to_follow::Response &res)
{

     if(cur_yolo_people.size()>0)
     {
          leg_target.resize(2,0.0);
          leg_target[0]=cur_yolo_people[0][0];
          leg_target[1]=cur_yolo_people[0][1];
          OnceTarget=true;
          ROS_INFO("set Target");
          std::cout<<"set target : "<<leg_target[0]<<" , "<<leg_target[1]<<std::endl;
          // ROS_INFO("Filter : Set Target pos X : %.3lf, y : %.3lf", cur_yolo_people[0],cur_yolo_people[1]);
       }
    else{

      std::cout<<"target is not set"<<std::endl;
    }

    res.track_target=true;
    return res.track_target;
}


// filter loop
void Edgeleg_manager_srv::spin()
{
  ROS_INFO("People tracking manager started.");

  while (ros::ok())
  {

    publish_leg_boxes();
    publish_cameraregion();
    publish_target();
    publish_filtered_target();
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
  ros::init(argc, argv, "edge_lef_filter");
  ros::NodeHandle(nh);
  // create tracker node
  Edgeleg_manager_srv edge_filter_node(nh);

   ROS_INFO("People tracking manager started.");
   edge_filter_node.spin();
  //  while (ros::ok())
  // {
  //    // ROS_INFO("I am in the loop");
  //   edge_filter_node.publish_leg_boxes();
  //   edge_filter_node.publish_cameraregion();
  //   edge_filter_node.publish_target();
  //   edge_filter_node.publish_filtered_target();
  //   edge_filter_node.Publish_nav_target();
  //   // ------ LOCKED ------
  //   // boost::mutex::scoped_lock lock(filter_mutex_);
  //   // lock.unlock();
  //   // ------ LOCKED ------

  //   // sleep
  //   // usleep(1e6 / freq_);

  //   ros::spinOnce();
  // }

  // wait for filter to finish
   //ros::spin();
  // Clean up

  return 0;
}
