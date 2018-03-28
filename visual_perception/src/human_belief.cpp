#include "human_belief.h"

using namespace std;
using namespace tf;
using namespace message_filters;

static const double       sequencer_delay            = 0.5; //TODO: this is probably too big, it was 0.8
static const unsigned int sequencer_internal_buffer  = 100;
static const unsigned int sequencer_subscribe_buffer = 10;
static const unsigned int num_particles_tracker      = 1000;
static const double       tracker_init_dist          = 4.0;

bool IsNotInitilized = true;

// constructor
belief_manager::belief_manager(ros::NodeHandle nh)
  : nh_(nh),
    robot_state_()
{
  // initialize
  camera_visible_region_pub=nh_.advertise<nav_msgs::OccupancyGrid>("/camera_region_map", 10, true);
  belief_pub=nh_.advertise<nav_msgs::OccupancyGrid>("/human_belief_map", 10, true);

  joint_state_sub =nh_.subscribe<sensor_msgs::JointState>("/hsrb/joint_states", 10, &belief_manager::joint_states_callback,this);
  people_yolo_sub=nh_.subscribe<visualization_msgs::MarkerArray>("/human_boxes", 10, &belief_manager::human_yolo_callback,this);
  people_poses_sub =nh_.subscribe<geometry_msgs::PoseArray>("/human_poses", 10, &belief_manager::human_poses_callback,this);
  globalpose_sub=nh_.subscribe<geometry_msgs::PoseStamped>("/global_pose",10,&belief_manager::global_pose_callback,this);
    
  global_pose.resize(3,0.0);
  Head_Pos.resize(2,0.0);

  //camera region
  camera_visible_region.info.width=30;
  camera_visible_region.info.height= 30;
  camera_visible_region.info.resolution=0.5;
  camera_visible_region.info.origin.position.x=-7.5;
  camera_visible_region.info.origin.position.y=-7.5;
  camera_visible_region.data.resize(camera_visible_region.info.width*camera_visible_region.info.height,0.0);

  //Initialize human_belief_map
  Human_Belief_Scan_map.info.width=30;
  Human_Belief_Scan_map.info.height= 30;
  Human_Belief_Scan_map.info.resolution=0.5;
  Human_Belief_Scan_map.info.origin.position.x=-7.5;
  Human_Belief_Scan_map.info.origin.position.y=-7.5;
  int belief_size=Human_Belief_Scan_map.info.width*Human_Belief_Scan_map.info.height;
  Human_Belief_Scan_map.data.resize(Human_Belief_Scan_map.info.width*Human_Belief_Scan_map.info.height);
  // int belief_map_size=static_belief_map.info.width*static_belief_map.info.height;
  for(int k(0);k<belief_size;k++)
      Human_Belief_Scan_map.data[k]=0.01;

}

// destructor
belief_manager::~belief_manager()
{
  // delete sequencer
  // delete all trackers
};


void belief_manager::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{

    //ROS_INFO("joint states callback");
  Head_Pos[0]=msg->position[9];     //pan
  Head_Pos[1]=msg->position[10];      //tilt
 

}

void belief_manager::publish_cameraregion()
{

   getCameraregion();
   camera_visible_region.header.stamp =  ros::Time::now();
   camera_visible_region.header.frame_id = "map"; 
   camera_visible_region_pub.publish(camera_visible_region);

}

void belief_manager::publish_human_belief()
{

    typedef std::map<int, float>::iterator map_iter;
    for(map_iter iterator = map_index_of_human_cells_to_prob.begin(); iterator !=  map_index_of_human_cells_to_prob.end(); iterator++) {

        int human_map_idx = iterator->first;
        Human_Belief_Scan_map.data[human_map_idx]=(iterator->second) * 100.0;
    }

   Human_Belief_Scan_map.header.stamp =  ros::Time::now();
   Human_Belief_Scan_map.header.frame_id = "map"; 
   belief_pub.publish(Human_Belief_Scan_map);


}

void belief_manager::getCameraregion()
{

  double global_robot_x= global_pose[0];
  double global_robot_y= global_pose[1];
  double global_robot_theta = global_pose[2]+Head_Pos[0];

  visiblie_idx_set.clear();
  global_robot_theta=0.0;
  //Iteration for belief grid
  for(int i(0);i<camera_visible_region.info.width;i++)
    for(int j(0);j<camera_visible_region.info.height;j++)
  {
    int belief_map_idx=j*camera_visible_region.info.height+i;

    // double map_ogirin_x = camera_visible_region.info.origin.position.x+global_robot_x;
    // double map_ogirin_y = camera_visible_region.info.origin.position.y+global_robot_y;

    double map_ogirin_x = camera_visible_region.info.origin.position.x;
    double map_ogirin_y = camera_visible_region.info.origin.position.y;


    double trans_vector_x=(i+0.5)*camera_visible_region.info.resolution;
    double trans_vector_y=(j+0.5)*camera_visible_region.info.resolution;

    double rot_trans_vector_x = cos(global_robot_theta)*trans_vector_x-sin(global_robot_theta)*trans_vector_y;
    double rot_trans_vector_y = sin(global_robot_theta)*trans_vector_x+cos(global_robot_theta)*trans_vector_y;

    double belief_global_x=map_ogirin_x+rot_trans_vector_x;
    double belief_global_y=map_ogirin_y+rot_trans_vector_y;

    //solve
    bool line1_result =getlinevalue(1,belief_global_x,belief_global_y);
    bool line2_result =getlinevalue(2,belief_global_x,belief_global_y);


    if( line1_result && line2_result )
    {
      camera_visible_region.data[belief_map_idx]=30;  
      visiblie_idx_set.push_back(belief_map_idx);         //save cell_id 
    }
    else
      camera_visible_region.data[belief_map_idx]=0.0; 
  }



}


bool belief_manager::getlinevalue(int line_type,double input_x, double input_y)
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

void belief_manager::dyn_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    // Copy Data;
	// update dynamic map info
    dynamic_belief_map = *msg;
	//dynamic_belief_map.info.width  = msg->info.width;
	//dynamic_belief_map.info.height = msg->info.height;
	//dynamic_belief_map.info.resolution = msg->info.resolution;
	//dynamic_belief_map.info.origin.position.x = msg->info.origin.position.x;
	//dynamic_belief_map.info.origin.position.y =msg->info.origin.position.y;
	//dynamic_belief_map.data.resize(dynamic_belief_map.info.width*dynamic_belief_map.info.width);
	
    int dyn_occ_size = msg->data.size();
     m_dyn_occupancy.resize(dyn_occ_size);
	 for(int i(0);i<msg->data.size();i++)
	 {
	 	m_dyn_occupancy[i]=msg->data[i];
	 }

}



void belief_manager::scaled_dynamic_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{



}

void belief_manager::scaled_static_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
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
 
}



bool belief_manager::check_staticObs(float x_pos,float y_pos)
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


bool belief_manager::check_cameraregion(float x_pos,float y_pos)
{

  if(abs(x_pos)<7.0 && abs(y_pos)<7.0)
  {
  //return true if it is occupied with obstacles
  if (camera_visible_region.data.size()>0)
  {   
    int obs_idx=CoordinateTransform_Global2_staticMap(x_pos,y_pos);
    
    if(obs_idx<camera_visible_region.data.size()){
      if(camera_visible_region.data[obs_idx]>10.0)
        return true;
      else
        return false;
    }
  }

  }

  return true;
}



int belief_manager::globalcoord_To_SScaled_map_index(float x_pos,float y_pos)
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


double belief_manager::getDistance_from_Vec(std::vector<double> origin, double _x, double _y)
{
  double temp=0.0;

  temp=(origin[0]-_x)*(origin[0]-_x);
  temp+=(origin[1]-_y)*(origin[1]-_y);
  temp=sqrt(temp);

  return temp;

}

void belief_manager::global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

   global_pose[0]=msg->pose.position.x;
   global_pose[1]=msg->pose.position.y;

   tf::StampedTransform baselinktransform;
   listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(2.0));
   listener.lookupTransform("map", "base_link", ros::Time(0), baselinktransform);
   double yaw_tf =   tf::getYaw(baselinktransform.getRotation()); 

    global_pose[2]=yaw_tf;

}


void belief_manager::human_poses_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{


    ROS_INFO("human poses callback");


}


void belief_manager::human_yolo_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    //clear human_occ_cells_updated from last callback
    index_of_human_occ_cells_updated_recently.clear();
    //human_occupied_idx.clear();

    //check number of detected_humans
    num_of_detected_human=msg->markers.size();

    if(num_of_detected_human>0)
       cur_yolo_people.resize(num_of_detected_human);
    else
    {
        update_human_occ_belief(NO_HUMANS_DETECTED);
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
      listener.waitForTransform("head_rgbd_sensor_rgb_frame", "map", ros::Time(0), ros::Duration(1.0));
              
      gV.header.stamp = ros::Time();
      gV.header.frame_id = "/head_rgbd_sensor_rgb_frame";
      listener.transformVector(std::string("/map"), gV, tV);
              
      double cur_people_x=tV.vector.x+global_pose[0];
      double cur_people_y=tV.vector.y+global_pose[1];

      cur_yolo_people[i].resize(2,0.0);
      cur_yolo_people[i][0]=cur_people_x;
      cur_yolo_people[i][1]=cur_people_y;


      int human_mapidx=CoordinateTransform_Global2_beliefMap(cur_people_x,cur_people_y);
      index_of_human_occ_cells_updated_recently.push_back(human_mapidx);
      Human_Belief_Scan_map.data[human_mapidx] = 70;

      if (map_index_of_human_cells_to_prob.count(human_mapidx) == 1){
          // Encountered the same cells. Update probability:
          // P(H|S) = P(S|H)P(H) / P(S)
          float prior = map_index_of_human_cells_to_prob[human_mapidx]; // P(H)
          float P_S = P_S_given_H*prior + P_S_given_Hc*(1-prior);
          float posterior = (P_S_given_H)*prior / P_S;
          map_index_of_human_cells_to_prob[human_mapidx] = posterior;

      }else{
          map_index_of_human_cells_to_prob[human_mapidx] = 0.1;
      }		
      //human_occupied_idx.push_back(human_mapidx);

   }

    update_human_occ_belief(HUMANS_DETECTED);

   // printf("size yolo : %d \n",cur_yolo_people.size());
}
void belief_manager::update_human_occ_belief(int update_type){

    std::map<int, int> map_index_recently_updated;
    if (update_type == (int) HUMANS_DETECTED){
        for(int i = 0; i < index_of_human_occ_cells_updated_recently.size(); i++){
            int index = index_of_human_occ_cells_updated_recently[i];
            map_index_recently_updated[index] = index;
        }
    }

    // Extract_world_indices_from_visible_camera_region(float depth, float width, float res)
    // For each cell, check if is labeled as human.
    // If cell is recently updated, continue;
    // If labeled as human, update probability of cell regions
    std::vector<int> indices_to_assign_as_free;

    for(int i(0);i< visiblie_idx_set.size();i++)
    {
        int cell_idx= visiblie_idx_set[i];
        if (map_index_recently_updated.count(cell_idx) == 1){
            continue;
        }
        if (Human_Belief_Scan_map.data[cell_idx]>0){
            // Update probability			
            float prior = map_index_of_human_cells_to_prob[cell_idx]; // P(H)
            float P_S = P_Sc_given_H*(prior) + P_Sc_given_Hc*(1-prior);
            float posterior = prior*0.4;
            map_index_of_human_cells_to_prob[cell_idx] =posterior;
            //map_index_of_human_cells_to_prob[cell_idx] =0.05;
            std::cout << "index : "<<cell_idx<< ", Prob: " << posterior << std::endl;
            if (posterior < PROB_THRESH){
                indices_to_assign_as_free.push_back(cell_idx);
            }

        }
        //else{
        
            //map_index_of_human_cells_to_prob[cell_idx] =0.05;
        
        //}

        for(size_t i = 0; i < indices_to_assign_as_free.size(); i++){
            int index_to_erase =  indices_to_assign_as_free[i];
            map_index_of_human_cells_to_prob.erase(index_to_erase);
            //Human_Belief_Scan_map.data[index_to_erase]=0.00;
        }
    
    }

    //if(num_of_detected_human==0)
    //{
         //for(int j(0);j<Human_Belief_Scan_map.data.size();j++)
         //std::cout<<"no human from yolo"<<std::endl;
        //for(int i(0);i< visiblie_idx_set.size();i++)
            //Human_Belief_Scan_map.data[visiblie_idx_set[i]]=0.0;
    //}
    //else{
            //int belief_map_index=visiblie_idx_set[i];

            //if( Human_Belief_Scan_map.data[belief_map_index]>0)	//If we detected human
            //{
                //prior =(float) Human_Belief_Scan_map.data[belief_map_index]/100.0; // P(H)
                 //float P_S = P_Sc_given_H*(prior) + P_Sc_given_Hc*(1-prior);
                //posterior = prior*0.99;
                //Human_Belief_Scan_map.data[belief_map_index] = posterior*100.0;
            //}
            //else
            //{
                //Human_Belief_Scan_map.data[belief_map_index]=0.0;

            //}
        //}
    //}





/*
    typedef std::map<int, int>::iterator it_type;
    for(it_type iterator = camera_visible_world_indices.begin(); iterator !=  camera_visible_world_indices.end(); iterator++) {
        int cell_index = iterator->first;
        if (map_index_recently_updated.count(cell_index) == 1){
            continue;
        }
        if (data[cell_index].cell_type == HUMAN_OCCUPIED){
            // Update probability			
            float prior = map_index_of_human_cells_to_prob[cell_index]; // P(H)
            float P_S = P_Sc_given_H*(prior) + P_Sc_given_Hc*(1-prior);
            float posterior = prior*0.4;
            map_index_of_human_cells_to_prob[cell_index] = posterior;

            //std::cout << "Prob: " << posterior << std::endl;

            if (posterior < PROB_THRESH){
                indices_to_assign_as_free.push_back(cell_index);
            }

        }

    }

*/





}


void belief_manager::put_human_occ_map_yolo()
{
	//int num_size = human_occupied_idx.size();
     //std::cout<<"human_occupied_size"<<num_size<<std::endl;
	//if(num_size>0)
	//{
		//if(Human_Belief_Scan_map.data.size()>0){
			//for(int i(0);i<human_occupied_idx.size();i++){
				 //Human_Belief_Scan_map.data[human_occupied_idx[i]]=80.0;
                 //put_human_surrounding_beliefmap(human_occupied_idx[i]);
			//}
		//}
	//}
	//else
	//{


	//}
}



// callback for messages
int belief_manager::CoordinateTransform_Global2_staticMap(float global_x, float global_y)
{
  double reference_origin_x=camera_visible_region.info.origin.position.x;
  double reference_origin_y=camera_visible_region.info.origin.position.y;

  //Find the coordinate w.r.t map origin
  double  temp_x  = global_x - reference_origin_x;
  double  temp_y  = global_y - reference_origin_y;

  //Find the map cell idx for x, y
  std::vector<int> human_coord(2,0);
  human_coord[0]= (int) (temp_x/camera_visible_region.info.resolution);
  human_coord[1]= (int) (temp_y/camera_visible_region.info.resolution);

  //Find the map index from cell x, y
  int static_map_idx= human_coord[0]+camera_visible_region.info.width*human_coord[1];

  // std::cout<<"map_idx : "<<static_map_idx<<std::endl;
  return static_map_idx;
   
}


int belief_manager::CoordinateTransform_Global2_beliefMap(double global_x, double global_y)
{

	double reference_origin_x=Human_Belief_Scan_map.info.origin.position.x;
	double reference_origin_y=Human_Belief_Scan_map.info.origin.position.y;

	//Find the coordinate w.r.t map origin
	double  temp_x  = global_x - reference_origin_x;
	double  temp_y  = global_y - reference_origin_y;

	//Find the map cell idx for x, y
	std::vector<int> human_coord(2,0);
 	human_coord[0]= (int) (temp_x/Human_Belief_Scan_map.info.resolution);
 	human_coord[1]= (int) (temp_y/Human_Belief_Scan_map.info.resolution);

 	//Find the map index from cell x, y
 	int static_map_idx= human_coord[0]+Human_Belief_Scan_map.info.width*human_coord[1];

 	return static_map_idx;

}




bool belief_manager::Comparetwopoistions(std::vector<double> pos,std::vector<double> pos2, double criterion)
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
void belief_manager::spin()
{
  ROS_INFO("People tracking manager started.");

  while (ros::ok())
  {
    publish_cameraregion();
    publish_human_belief();

    // ------ LOCKED ------
    boost::mutex::scoped_lock lock(filter_mutex_);
    lock.unlock();
    // ------ LOCKED ------

    // sleep
    ros::Duration(0.4).sleep();

    ros::spinOnce();
  }
};

// ----------
// -- MAIN --
// ----------
int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "human_belief_manager");
  ros::NodeHandle(nh);

  // create human_tracker node
  belief_manager human_belief_node(nh);
  // wait for filter to finish
  human_belief_node.spin();
  // Clean up

  return 0;
}
