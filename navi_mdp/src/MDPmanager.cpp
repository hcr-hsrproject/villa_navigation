#include "MDPmanager.h"



MapParam::MapParam()
{
    map_step=Grid_STEP;
   	m_cell_x_width=0.0;
	m_cell_y_width=0.0;
	Num_grid_X=Grid_Num_X;
	Num_grid_Y=Grid_Num_Y;
	MapSize=Grid_Num_X*Grid_Num_Y;
	Cell_Info.resize(MapSize, 0);
	OCC_Info.resize(MapSize, 0);
	boolDynamic=false;
}

MapParam::MapParam(int width_,int height_,double res_)
{
	setWidth(width_);
	setHeight(height_);
	setResolution(res_);

	m_cell_x_width=0.0;
	m_cell_y_width=0.0;
	MapSize=width_*height_;
	Cell_Info.resize(MapSize, 0);
	OCC_Info.resize(MapSize, 0);
	boolDynamic=true;
}

MapParam::~MapParam()
{

}
void MapParam::setWidth (int Width_)
{
	Num_grid_X=Width_;
}

void MapParam::setHeight(int height_)
{
	Num_grid_Y=height_;
}

void MapParam::setResolution(double res_)
{
	map_step=res_;
}

void MapParam::set_Cell_Info(vector<int> _inputCellInfo)
{
	Cell_Info.resize(_inputCellInfo.size());
	
	for(int i(0);i<_inputCellInfo.size();i++)
		Cell_Info[i]=_inputCellInfo[i];

}

void MapParam::set_State_Type(vector<int> _State_Type)
{
	for(int i(0);i<_State_Type.size();i++)
	State_Type[i]=_State_Type[i];

}

void MapParam::set_State_Distance(vector<float> _State_Distance)
{
	for(int i(0);i<_State_Distance.size();i++)
	State_Distance[i]=_State_Distance[i];

}
void MapParam::set_NearestHuman_V(vector<float> _NearestHuman_V)
{
		for(int i(0);i<_NearestHuman_V.size();i++)
	NearestHuman_V[i]=_NearestHuman_V[i];

}

void MapParam::set_RobotHeading_V(vector<float> _RobotHeading_V)
{
		for(int i(0);i<_RobotHeading_V.size();i++)
	RobotHeading_V[i]=_RobotHeading_V[i];
}

void MapParam::set_OCC_Info(vector<int> _inputOCCInfo)
{

	OCC_Info.resize(_inputOCCInfo.size());
	
	for(int i(0);i<_inputOCCInfo.size();i++)
		OCC_Info[i]=_inputOCCInfo[i];
}

void MapParam::set_Robot_Info(vector<int> _inputRobotInfo)
{
	Robot_localpos.resize(_inputRobotInfo.size());
	for(int i(0);i<_inputRobotInfo.size();i++)
		Robot_localpos[i]=_inputRobotInfo[i];
}


MDPManager::MDPManager(MapParam* _pMapParam):maxiter(Maxiteration),Action_dim(8),gamma(1),Ra(ra),publishnum(0),ReceiveData(0),m_boolSolve(false),dyn_path_num(0)
{
	pMapParam=_pMapParam;
	Init();
}

void MDPManager::setPMapParam(MapParam* _pMapParam)
{

	pMapParam=_pMapParam;
	Init();

}

MDPManager::~MDPManager()
{
	if(pMapParam!=NULL)
	{
		delete pMapParam;
		pMapParam=NULL;
	}
}

//Callback function for map
// void MDPManager::local_mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
// {
//    //  ROS_INFO("int msg");
//    // // ROS_INFO("Cur costmap cmd :%d ",msg->data);
//    //  std::cout<<"Receive localmap"<<std::endl;
//    m_localoccupancy.resize(msg->data.size());

//    for(int i(0);i<msg->data.size();i++){
//         m_localoccupancy[i]=(msg->data)[i];
//    }

//    ReceiveData++;
//    updateMap();

//    return;
// }

void MDPManager::base_pose_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
   CurVector[0]= global_pose[0];
   CurVector[1]= global_pose[1];
   CurVector[2]= global_pose[2];

}

void MDPManager::Init()
{
	cout<<"Initialize"<<endl;
	Local_X_start=0;
 	Local_Y_start=0;
 	ReceiveData=0;
 	human_callback_count=0;
 	
 	X_mapSize=pMapParam->Num_grid_X;
 	Y_mapSize=pMapParam->Num_grid_Y;
 	Num_Grids=pMapParam->MapSize;
 	cout<<" Grid - X :"<<X_mapSize<<", - Y : "<<Y_mapSize<<endl;
 	cout<<" NumofGrids : "<<Num_Grids<<endl;

 	Policies.resize(pMapParam->MapSize, '0');
 	Rewards.resize(pMapParam->MapSize, Ra); 
 	U.resize(pMapParam->MapSize, 0.0); 
 	Up.resize(pMapParam->MapSize, 0.0); 
 	MdpSols.resize(pMapParam->MapSize, 0);
 	PolicyNum.resize(pMapParam->MapSize, 0);

 	m_Start.resize(2,0);	
    m_Goal.resize(2,0);							
 	m_Robot.resize(2,0);
 	Human_Goal_Coord.resize(2,0);
 	human_global.resize(2,0.0);
 	m_Start[0]=Start_X;
 	m_Start[1]=Start_Y;
 	cout<<" Start pos - X :"<<Start_X<<", - Y : "<<Start_Y<<endl;

 	//Current Pos 
 	m_Robot[0]=Start_X;
 	m_Robot[1]=Start_Y;

 	m_Goal[0]=Goal_X;
 	m_Goal[1]=Goal_Y;
 	cout<<" Goal pos - X :"<<Goal_X<<", - Y : "<<Goal_Y<<endl;

 	 Rewards[Coord2CellNum(m_Goal)]=100;
 	Policies[Coord2CellNum(m_Goal)]='+';


 	for(int i(0);i<m_static_obs.size();i++){
 		Rewards[m_static_obs[i]]=0.0;
 		Policies[m_static_obs[i]]='#';
  	}

 	//setActionVector
 	ActionCC.resize(Action_dim);
 	for(int i(0);i<Action_dim;i++)
 		ActionCC[i].resize(2);

 // 	ActionCC[0][0]= 1;   ActionCC[0][1]= 0;
	// ActionCC[1][0]= 0;   ActionCC[1][1]= 1;
	// ActionCC[2][0]= -1;  ActionCC[2][1]= 0;
	// ActionCC[3][0]=0;    ActionCC[3][1]= -1;
 	ActionCC[0][0]= 1;   ActionCC[0][1]= 0;
	ActionCC[1][0]= 1;   ActionCC[1][1]= 1;
	ActionCC[2][0]= 0;   ActionCC[2][1]= 1;
	ActionCC[3][0]=-1;   ActionCC[3][1]= 1;
	ActionCC[4][0]=-1;   ActionCC[4][1]= 0;
	ActionCC[5][0]=-1;   ActionCC[5][1]=-1;
	ActionCC[6][0]= 0;   ActionCC[6][1]=-1;
	ActionCC[7][0]= 1;   ActionCC[7][1]=-1;

	Prob_good = 0.95;
	Prob_bad = (1-Prob_good)/2.0;

	 Map_orig_Vector.resize(2,0.0);
	 Map_orig_Vector[0]= 3.5;
   	 Map_orig_Vector[1]=-3.5;


   	 global_pose.resize(3,0.0);
	 CurVector.resize(3,0.0);
	 GoalVector.resize(2,0.0);
	 HeadingVector.resize(2,0.0);
	 cur_coord.resize(2,0);
	 Goal_Coord.resize(2,0);
	 MapCoord.resize(2,0);

	 m_desired_heading=0.0;

	 
	 m_localoccupancy.resize(Grid_Num_X*Grid_Num_Y);
	 m_dynamic_occupancy.resize(Num_Grids);


	//Declare publisher
	 // obsmap_Pub= m_node.advertise<std_msgs::Int32MultiArray>("MDP/costmap", 10);
	 // Path_Pub= m_node.advertise<std_msgs::Int32MultiArray>("MDP/path", 10);
	 SplinePath_pub=  m_node.advertise<nav_msgs::Path>("mdp_path", 10, true);
	 SplinePath_pub2=  m_node.advertise<nav_msgs::Path>("mdp_path_dynamic", 10, true);
	 UnitGoalVec_pub = m_node.advertise<std_msgs::Float32MultiArray>("/CBA_unit_goal", 10, true);
	 Scaled_static_map_pub=m_node.advertise<nav_msgs::OccupancyGrid>("/scaled_static_map", 10, true);
	 Scaled_static_map_path_pub=m_node.advertise<nav_msgs::OccupancyGrid>("/scaled_static_map_path", 10, true);
	 Scaled_dynamic_map_pub=m_node.advertise<nav_msgs::OccupancyGrid>("/scaled_dynamic_map", 10, true);
	 Scaled_dynamic_map_path_pub=m_node.advertise<nav_msgs::OccupancyGrid>("/scaled_dynamic_map_path", 10, true);
	 MDPSol_pub = m_node.advertise<std_msgs::Int32MultiArray>("MDP/Solution", 10);
	 RobotHeading_pub= m_node.advertise<geometry_msgs::Pose>("mdp_HeadingV", 10);

	Scaled_static_map_path.info.width=Grid_Num_X;
	Scaled_static_map_path.info.height= Grid_Num_Y;
	Scaled_static_map_path.info.resolution=0.5;
	Scaled_static_map_path.info.origin.position.x=-4;
	Scaled_static_map_path.info.origin.position.y=-4;
	Scaled_static_map_path.data.resize(Scaled_static_map_path.info.width*Scaled_static_map_path.info.height);


   	Scaled_dynamic_map_path.info.width=14;
	Scaled_dynamic_map_path.info.height= 14;
	Scaled_dynamic_map_path.info.resolution=0.5;
	Scaled_dynamic_map_path.info.origin.position.x=CurVector[0]-DYN_OFFSET_X-0.5*Scaled_dynamic_map.info.resolution;
	Scaled_dynamic_map_path.info.origin.position.y=CurVector[1]-DYN_OFFSET_Y-0.5*Scaled_dynamic_map.info.resolution;;
	// Scaled_dynamic_map_path.info.origin.position.x=CurVector[0]-0.5*Scaled_dynamic_map_path.info.width*0.5;
	// Scaled_dynamic_map_path.info.origin.position.y=CurVector[1]-0.5*Scaled_dynamic_map_path.info.height*0.5;
	Scaled_dynamic_map_path.data.resize(Scaled_dynamic_map_path.info.width*Scaled_dynamic_map_path.info.height);
	booltrackHuman=false;
	dyn_path_num=0;

	 ROS_INFO("here2");

}

//function which relates to get origin w.r.t map (mdp)3232
void MDPManager::CoordinateTransform_Rviz_Grid_Start(double _x, double _y,int map_type=0)
{
	 cur_coord.resize(2,0);

	 //for case of using static map
	double reference_origin_x;
	double reference_origin_y;

	if(map_type==0) //static
	{
		ROS_INFO("static_start");
		reference_origin_x=Scaled_static_map.info.origin.position.x;
		reference_origin_y=Scaled_static_map.info.origin.position.y;
	}
	else
	{
		ROS_INFO("dynamic_start");
		reference_origin_x=Scaled_dynamic_map.info.origin.position.x;
		reference_origin_y=Scaled_dynamic_map.info.origin.position.y;
	}

	//for case of using static map
	// double reference_origin_x =-3.5;
	// double reference_origin_y =-3.5;
	
	double  temp_x  = _x-reference_origin_x;
	double  temp_y = _y-reference_origin_y;


	cur_coord[0]= (int)(temp_x/pMapParam->map_step);
 	cur_coord[1]= (int)(temp_y/pMapParam->map_step);


 	 return;
}

//function which relates to get origin w.r.t map (mdp)
void MDPManager::CoordinateTransform_Rviz_Grid_Goal(double _x, double _y,int map_type=0)
{
	 Goal_Coord.resize(2,0);
	
	double reference_origin_x;
	double reference_origin_y;

	if(map_type==0) //static
	{
		ROS_INFO("static_goal_setting\n");
		reference_origin_x=Scaled_static_map.info.origin.position.x;
		reference_origin_y=Scaled_static_map.info.origin.position.y;
	}
	else	//dynamic_window
	{
		ROS_INFO("dynamic_goal_setting\n");
		reference_origin_x=Scaled_dynamic_map.info.origin.position.x;
		reference_origin_y=Scaled_dynamic_map.info.origin.position.y;
	}


	double  temp_x  = _x-reference_origin_x;
	double  temp_y = _y-reference_origin_y;


 	Goal_Coord[0]= (int) (temp_x/pMapParam->map_step);
 	Goal_Coord[1]= (int)(temp_y/pMapParam->map_step);

 	ROS_INFO("Goal_Coord : x : %d , y : %d ", Goal_Coord[0],Goal_Coord[1]);

 	 return;

}


void MDPManager::CoordinateTransform_Rviz_Grid_Human(double _x, double _y,int map_type=0)
{
	 Human_Goal_Coord.resize(2,0);
	
	double reference_origin_x;
	double reference_origin_y;

	if(map_type==0) //static
	{
		
		reference_origin_x=Scaled_static_map.info.origin.position.x;
		reference_origin_y=Scaled_static_map.info.origin.position.y;
	}
	else	//dynamic_window
	{
		
		reference_origin_x=Scaled_dynamic_map.info.origin.position.x;
		reference_origin_y=Scaled_dynamic_map.info.origin.position.y;
	}


	double  temp_x  = _x-reference_origin_x;
	double  temp_y = _y-reference_origin_y;


 	Human_Goal_Coord[0]= (int) (temp_x/pMapParam->map_step);
 	Human_Goal_Coord[1]= (int)(temp_y/pMapParam->map_step);

 	// ROS_INFO("Goal_Coord : x : %d , y : %d ", Goal_Coord[0],Goal_Coord[1]);

 	 return;

}

void MDPManager::Human_target_cmdCallback(const std_msgs::Int8::ConstPtr& msg)
{

	if(msg->data==1)
	{
		booltrackHuman=true;
	}
	else{
		booltrackHuman=false;

	}
}

bool MDPManager::IsinDynamicMap(float global_x, float global_y)
{
	float margin =0.1;
	float map_start_x=Scaled_dynamic_map.info.origin.position.x-margin;
	float map_start_y=Scaled_dynamic_map.info.origin.position.y-margin;
	float map_end_x =Scaled_dynamic_map.info.origin.position.x+Scaled_dynamic_map.info.width*Scaled_dynamic_map.info.resolution+margin;
	float map_end_y =Scaled_dynamic_map.info.origin.position.y+Scaled_dynamic_map.info.height*Scaled_dynamic_map.info.resolution+margin;

	if( (global_x> map_start_x) && (global_x< map_end_x) )
		if((global_y> map_start_y) && (global_y< map_end_y) )
			return true;

	return false;
}


bool MDPManager::IsTargetMoved(float target_x, float target_y, float criterion)
{
	float temp_dist=0.0;
	temp_dist= (GoalVector[0]-target_x)*(GoalVector[0]-target_x);
	temp_dist+=(GoalVector[1]-target_y)*(GoalVector[1]-target_y);
	temp_dist=sqrt(temp_dist);

	if(temp_dist>criterion)
		return true;
	else
		return false;

}

double MDPManager::getdistance(vector<double> cur, vector<double> goal)
{
	double temp_dist=0.0;
	for(int i(0);i<2;i++)
		temp_dist+=(cur[i]-goal[i])*(cur[i]-goal[i]);
	temp_dist = sqrt(temp_dist);

	return temp_dist;

}

void MDPManager::setDesiredHeading(double _heading)
{
	m_desired_heading=_heading;
}

void MDPManager::Human_MarkerCallback(const visualization_msgs::Marker::ConstPtr& msg)
{

	float human_target_goal_x=  msg->pose.position.x;
    float human_target_goal_y=  msg->pose.position.y;

    //control the time
    human_callback_count++;
    if(human_callback_count<30)
    	return;
    else
    	human_callback_count=0;

    CoordinateTransform_Rviz_Grid_Human(human_target_goal_x,human_target_goal_y,1);
   

   //check if person moved a lot 
   if( (dyn_path_num>0) && IsTargetMoved(human_target_goal_x,human_target_goal_y,3.0))
   	return;

   //check if person is in a range
   // if(!IsinDynamicMap(human_target_goal_x,human_target_goal_y))   
   // {
  	// 	ROS_INFO("human is out of my dynamic range");
  	// 	return;
   // }

    //dynamic goal setting
    if(booltrackHuman)
    {
    	if(dyn_path_num>0 && (!IsinDynamicMap(human_target_goal_x,human_target_goal_y)))
    		return;

    	if(dyn_path_num==0 || IsTargetMoved(human_target_goal_x,human_target_goal_y, 0.5))
    	{ 
	    	//global coordinate
	    	GoalVector.resize(2,0);
	    	GoalVector[0]=human_target_goal_x;
			GoalVector[1]=human_target_goal_y;
			double temp_yaw =atan(HeadingVector[1]/HeadingVector[0]);

			
			if(getdistance(CurVector,GoalVector)<1.2)
				return;

			HeadingVector.resize(2,0.0);
			HeadingVector[0]= GoalVector[0]-CurVector[0];
			HeadingVector[1]= GoalVector[1]-CurVector[1];
			setDesiredHeading(atan(HeadingVector[1]/HeadingVector[0]));

			 ROS_INFO("Human target (global)  x : %.3lf, y : %.3lf\n",GoalVector[0],GoalVector[1]);
			//should be transfertodynamic_coordinate
	    	cur_coord.resize(2);
		 	Human_Goal_Coord.resize(2);
		 	
		 	ROS_INFO("Cur robot (global)  x : %.3lf, y : %.3lf\n",CurVector[0],CurVector[1]);
		 	CoordinateTransform_Rviz_Grid_Start(CurVector[0],CurVector[1],1);
		 	CoordinateTransform_Rviz_Grid_Goal(GoalVector[0],GoalVector[1],1);

		 	ROS_INFO("Start : x : %d, y : %d , Goal : x %d, y : %d \n",cur_coord[0],cur_coord[1],Goal_Coord[0],Goal_Coord[1]);
		 	updateMap(m_dynamic_occupancy,cur_coord,Goal_Coord);

		 	geometry_msgs::Pose heading_msg;
		 	heading_msg.position.x=0;
		 	heading_msg.position.y=0;
		 	heading_msg.position.z=0;

		 	heading_msg.orientation.x=0;
		 	heading_msg.orientation.y=0;
		 	heading_msg.orientation.z=m_desired_heading;
		 	heading_msg.orientation.w=0;
		 	
		 	RobotHeading_pub.publish(heading_msg);

		 	if(Goal_Coord[0]<0 || Goal_Coord[1]<0)
		 		return;

		 	m_boolSolve=true;
	 	}


    }
    else{
    
    	m_boolSolve=false;

    }



}


void MDPManager::ClikedpointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	printf("Receive point\n");
    GoalVector.resize(2,0);
    GoalVector[0]=msg->point.x;
	GoalVector[1]=msg->point.y;
    // GoalVector[0]=msg->point.x-Map_orig_Vector[0];
    // GoalVector[1]=msg->point.y-Map_orig_Vector[1];
    // CurVector[0]=0.0;
    // CurVector[1]=0.0;
    // printf(" cur x index is %.3f, cur y index is %.3f \n",CurVector[0],CurVector[1]);  
    // printf(" goal x index is %.3f, goal y index is %.3f \n",GoalVector[0],GoalVector[1]);  
     cur_coord.resize(2);
	 Goal_Coord.resize(2);
	 
	 CoordinateTransform_Rviz_Grid_Start(CurVector[0],CurVector[1],0);
	 CoordinateTransform_Rviz_Grid_Goal(GoalVector[0],GoalVector[1],0);
	 updateMap(m_localoccupancy,cur_coord,Goal_Coord);

	 //for MDP planner 
	 // int map_type=0;	
  //    if(pMapParam->boolDynamic)	//static
  //    	updateMap(m_localoccupancy,cur_coord,Goal_Coord);
  //    else				//dynamic
  //    {
  //    	updateMap(m_dynamic_occupancy,cur_coord,Goal_Coord);

  //    }

    //Pulbish unitvector
    std::vector<float> unitgoalvector(2,0.0);
	unitgoalvector[0] = GoalVector[0]-CurVector[0];
	unitgoalvector[1] = GoalVector[1]-CurVector[1];
	float vector_norm=sqrt(unitgoalvector[0]*unitgoalvector[0]+unitgoalvector[1]*unitgoalvector[1]);
	unitgoalvector[0] =unitgoalvector[0]/vector_norm;
	unitgoalvector[1] =unitgoalvector[1]/vector_norm;

	std_msgs::Float32MultiArray unitgoal_msg;
	unitgoal_msg.data.resize(unitgoalvector.size());
	for(int i(0);i<unitgoalvector.size();i++)
	unitgoal_msg.data[i] = unitgoalvector[i];
	UnitGoalVec_pub.publish(unitgoal_msg);
	ROS_INFO("clicked goal_uint goal x : %.3lf, y : %.3lf\n",unitgoalvector[0],unitgoalvector[1]);
      // printf("x index is %.3f, y index is %.3f \n",Goal_Coord[0],Goal_Coord[1]);  
    m_boolSolve=true;

    return;
}

void MDPManager::MDPsolPublish()
{
	//  std_msgs::Int32MultiArray MDPsolution_msg;
	
	// MDPsolution_msg.data.resize(32*32); 
	// for(int i(0);i<32*32;i++)
	// 	MDPsolution_msg.data[i]=0.1*5*i;
	// MDPSol_pub.publish(MDPsolution_msg);


}

void MDPManager::Basepos_Callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{

  ROS_INFO("base position msg");


   // Map_orig_Vector[0]= msg->point.x-3.5;
   // Map_orig_Vector[1]= msg->point.y-3.5;


   printf("Map origin x index is %.3f, y index is %.3f \n",Map_orig_Vector[0],Map_orig_Vector[1]); 


   // CurVector[0]= msg->point.x;
   // CurVector[1]= msg->point.y;

   //    printf("Cur base x index is %.3f, y index is %.3f \n",CurVector[0],CurVector[1]); 

}


void MDPManager::dynamic_mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	//ROS_INFO("I am calling");
	double small_pos_x, small_pos_y=0.0;
	double dist_x,dist_y=0.0;
	int map_coord_i,map_coord_j=0;
	int numcount=0;
	int	original_width=msg->info.width;			//140
	int	original_height= msg->info.height;		//140
	double original_x=msg->info.origin.position.x;
	double original_y=msg->info.origin.position.y;
	double oroginal_res=0.05;					//0.05

	//for static space map
	Scaled_dynamic_map.info.width=14;
	Scaled_dynamic_map.info.height= 14;
	Scaled_dynamic_map.info.resolution=0.5;
	Scaled_dynamic_map.info.origin.position.x=CurVector[0]-DYN_OFFSET_X-0.5*Scaled_dynamic_map.info.resolution;
	Scaled_dynamic_map.info.origin.position.y=CurVector[1]-DYN_OFFSET_Y-0.5*Scaled_dynamic_map.info.resolution;
	// Scaled_dynamic_map.info.origin.position.x=CurVector[0]-0.5*Scaled_dynamic_map.info.width*0.5;
	// Scaled_dynamic_map.info.origin.position.y=CurVector[1]-0.5*Scaled_dynamic_map.info.height*0.5;
	Scaled_dynamic_map.data.resize(Scaled_dynamic_map.info.width*Scaled_dynamic_map.info.height);


	Scaled_dynamic_map_path.info.width=14;
	Scaled_dynamic_map_path.info.height= 14;
	Scaled_dynamic_map_path.info.resolution=0.5;
	Scaled_dynamic_map_path.info.origin.position.x=CurVector[0]-DYN_OFFSET_X-0.5*Scaled_dynamic_map.info.resolution;
	Scaled_dynamic_map_path.info.origin.position.y=CurVector[1]-DYN_OFFSET_Y-0.5*Scaled_dynamic_map.info.resolution;


	//for path map
	// Scaled_static_map_path.info.width=32;
	// Scaled_static_map_path.info.height= 32;
	// Scaled_static_map_path.info.resolution=0.5;
	// Scaled_static_map_path.info.origin.position.x=-4;
	// Scaled_static_map_path.info.origin.position.y=-4;
	//Scaled_static_map_path.data.resize(32*32);

   double base_origin_x =msg->info.origin.position.x;
   double base_origin_y =msg->info.origin.position.y;

	std::map<int,int> occupancyCountMap;
    int scaled_res=10;
    int map_idx=0;
    int scaled_result=0;

   for(int j(0);j<Scaled_dynamic_map.info.height;j++)
   	for(int i(0);i<Scaled_dynamic_map.info.width;i++)
   	{
   		map_idx=j*Scaled_dynamic_map.info.height+i;

   		//get global coordinate
   		double pos_x=i*Scaled_dynamic_map.info.resolution+Scaled_dynamic_map.info.origin.position.x;
   		double pos_y=j*Scaled_dynamic_map.info.resolution+Scaled_dynamic_map.info.origin.position.y;

   		numcount=0;
   		for(int k(0);k<scaled_res;k++)
   			for(int j(0);j<scaled_res;j++)
   			{
   				small_pos_x=pos_x+j*oroginal_res;
   				small_pos_y=pos_y+k*oroginal_res;
   				dist_x= small_pos_x-original_x;
				dist_y= small_pos_y-original_y;
				map_coord_i=floor(dist_x/oroginal_res);
				map_coord_j=floor(dist_y/oroginal_res);
				
				//static_map_ref_index
				int map_data_index=original_width*map_coord_j+map_coord_i;
				float temp_occupancy= msg->data[map_data_index];
    			 if(temp_occupancy>0)
				 	numcount++;
   			}

   			if(numcount>70)
   				scaled_result=50;
   			else
   				scaled_result=0;

   			Scaled_dynamic_map.data[map_idx]=scaled_result;
   	}

   	//remove human occupied cell
   	std::vector<int> HumansurroundingCoord(2,0);

   	Scaled_dynamic_map.data[Coord2CellNum(Human_Goal_Coord)]=0.0;
   	 for(int i(0);i<8;i++){
   	 	for(int j(0);j<2;j++)
  		{
   			HumansurroundingCoord[j]=Human_Goal_Coord[j]+ActionCC[i][j];
   		}


   		Scaled_dynamic_map.data[Coord2CellNum(HumansurroundingCoord)]=0.0;
   	 }



     //find index from
	 Scaled_dynamic_map.header.stamp =  ros::Time::now();
	 Scaled_dynamic_map.header.frame_id = "map"; 
     Scaled_dynamic_map_pub.publish(Scaled_dynamic_map);


    for(int i(0);i<m_dynamic_occupancy.size();i++)
    	m_dynamic_occupancy[i]=Scaled_dynamic_map.data[i];



}


void MDPManager::static_mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	// ROS_INFO("staticmap callback start");



	double small_pos_x, small_pos_y=0.0;
	double dist_x,dist_y=0.0;
	int map_coord_i,map_coord_j=0;
	int numcount=0;
	int	original_width=msg->info.width;
	int	original_height= msg->info.height;
	double original_x=-51.225;
	double original_y=-51.225;;
	double oroginal_res=0.05;

	//for static space map
	Scaled_static_map.info.width=Grid_Num_X;
	Scaled_static_map.info.height= Grid_Num_Y;
	Scaled_static_map.info.resolution=0.5;
	Scaled_static_map.info.origin.position.x=-4;
	Scaled_static_map.info.origin.position.y=-4;
	Scaled_static_map.data.resize(Scaled_static_map.info.width*Scaled_static_map.info.height);

	// if(msg->data[0]!=NULL)
	// {
		int datasize=msg->data.size();
		// ROS_INFO("staticmap size : %d",datasize);
		// for(int z(0);z<original_width*original_height;z++)
		// 	std::cout<<msg->data[z]<<std::endl;
	
	//for path map
	// Scaled_static_map_path.info.width=32;
	// Scaled_static_map_path.info.height= 32;
	// Scaled_static_map_path.info.resolution=0.5;
	// Scaled_static_map_path.info.origin.position.x=-4;
	// Scaled_static_map_path.info.origin.position.y=-4;
	//Scaled_static_map_path.data.resize(32*32);

   double base_origin_x =msg->info.origin.position.x;
   double base_origin_y =msg->info.origin.position.y;

	std::map<int,int> occupancyCountMap;
    int scaled_res=10;
    int map_idx=0;
    int scaled_result=0;

   for(int j(0);j<Scaled_static_map.info.height;j++)
   	for(int i(0);i<Scaled_static_map.info.width;i++)
   	{
   		map_idx=j*Scaled_static_map.info.height+i;
   		double pos_x=i*Scaled_static_map.info.resolution+Scaled_static_map.info.origin.position.x;
   		double pos_y=j*Scaled_static_map.info.resolution+Scaled_static_map.info.origin.position.y;

   		numcount=0;
   		for(int k(0);k<scaled_res;k++)
   			for(int j(0);j<scaled_res;j++)
   			{
   				small_pos_x=pos_x+j*oroginal_res;
   				small_pos_y=pos_y+k*oroginal_res;
   				dist_x= small_pos_x-original_x;
				dist_y= small_pos_y-original_y;
				map_coord_i=floor(dist_x/oroginal_res);
				map_coord_j=floor(dist_y/oroginal_res);
				
				int map_data_index=original_width*map_coord_j+map_coord_i;
				float temp_occupancy= msg->data[map_data_index];
    			 if(temp_occupancy>0)
				 	numcount++;
   			}

   			if(numcount>5)
   				scaled_result=50;
   			else
   				scaled_result=0;

   			Scaled_static_map.data[map_idx]=scaled_result;
   	}

     //find index from
	 Scaled_static_map.header.stamp =  ros::Time::now();
	 Scaled_static_map.header.frame_id = "map"; 
    Scaled_static_map_pub.publish(Scaled_static_map);


	
    m_localoccupancy.resize(Scaled_static_map.data.size());
    for(int i(0);i<m_localoccupancy.size();i++)
    	m_localoccupancy[i]=Scaled_static_map.data[i];

	}

   // ROS_INFO("staticmap callback start");

// }


void MDPManager::global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	global_pose[0]=msg->pose.position.x;
	global_pose[1]=msg->pose.position.y;


   tf::StampedTransform baselinktransform;
   listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(10.0));
   listener.lookupTransform("map", "base_link", ros::Time(0), baselinktransform);
   double yaw_tf =   tf::getYaw(baselinktransform.getRotation()); 

	global_pose[2]=yaw_tf;

   CurVector[0]= global_pose[0];
   CurVector[1]= global_pose[1];
   CurVector[2]= global_pose[2];

}


void MDPManager::Local_mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{

	 if(ReceiveData==0)
    {
      ROS_INFO("width : %d" ,msg->info.width);
      ROS_INFO("height : %d" ,msg->info.height);
      ROS_INFO("resolution : %lf" ,msg->info.resolution);
      ReceiveData++;

      pMapParam->Num_grid_X=(int) msg->info.width;
      pMapParam->Num_grid_Y=(int) msg->info.height;
      pMapParam->map_step= (double) msg->info.resolution;  

    }

   double base_origin_x =msg->info.origin.position.x;
   double base_origin_y =msg->info.origin.position.y;

   //ROS_INFO("origin x: %lf, y : %lf",base_origin_x,base_origin_y);
   //ROS_INFO("int msg");
   //  m_localoccupancy.resize(msg->data.size());

   // for(int i(0);i<msg->data.size();i++){
   //      m_localoccupancy[i]=(msg->data)[i];
   // }	

   ReceiveData++;


}


void MDPManager::Global2MapCoord(const vector<double>& _globalcoord,vector<int>& _MapCoord)
{
 
 _MapCoord.resize(2);
    
  _MapCoord[0]= (int)_globalcoord[0]/(pMapParam->map_step);
  _MapCoord[1]= (int)_globalcoord[1]/(pMapParam->map_step);

  //std::cout<<"x:"<<MapCoord[0]<<" , "<<"y:"<<MapCoord[1]<<std::endl;

  return ;
}



void MDPManager::updateMap(vector<int>& localmap_, vector<int>& local_start, vector<int>& local_goal)
{
	m_static_obs.clear();
	Policies.clear();	
	Rewards.clear();
	U.clear();
	MdpSols.clear();
	PolicyNum.clear();

	Policies.resize(pMapParam->MapSize, '0');
 	Rewards.resize(pMapParam->MapSize, Ra); 
 	U.resize(pMapParam->MapSize, 0.0); 
 	Up.resize(pMapParam->MapSize, 0.0); 
 	MdpSols.resize(pMapParam->MapSize, 0);
 	PolicyNum.resize(pMapParam->MapSize, 0);

 	m_Start.resize(2,0);	
    m_Goal.resize(2,0);							
 	m_Robot.resize(2,0);

 	m_Start[0]=cur_coord[0];
 	m_Start[1]=cur_coord[1];

	// cout<<" Start pos - X :"<<m_Start[0]<<", - Y : "<<m_Start[1]<<endl;
	// cout<<" Goal pos - X :"<<m_Goal[0]<<", - Y : "<<m_Goal[1]<<endl;

 	//Current Pos 
 	// m_Robot[0]=Start_X;
 	// m_Robot[1]=Start_Y;

 	m_Goal[0]=Goal_Coord[0];
 	m_Goal[1]=Goal_Coord[1];

 	// Policies[Coord2CellNum(m_Start)]='*';
  //	cout<<" Goal pos - X :"<<Goal_X<<", - Y : "<<Goal_Y<<endl;
	for(int i(0);i<localmap_.size();i++)
	{
		if(localmap_[i]>2.0 )
			m_static_obs.push_back(i);
	}

	vector<int> tempcoord(2,0);
	for(int i(0);i<m_static_obs.size();i++){
 		Rewards[m_static_obs[i]]=0.0;
 		Policies[m_static_obs[i]]='#';
 	}


	Rewards[Coord2CellNum(m_Goal)]=100;
 	Policies[Coord2CellNum(m_Goal)]='+';

 	printf("Update good!\n");


}

vector<int> MDPManager::Global2LocalCoord(vector<int> Global_coord)
{
	vector<int> Local_coords(2,0);
	
	Local_coords[0]=Global_coord[0]-Local_X_start;
	Local_coords[1]=Global_coord[1]-Local_Y_start;

	return Local_coords;
}

void MDPManager::setStartConfig( const vector<int> _Start)
{
		m_Start[0]=_Start[0];
		m_Start[1]=_Start[1];
}

void MDPManager::setGoalConfig( const vector<int> _Goal )
{
		m_Goal[0]=_Goal[0];
		m_Goal[1]=_Goal[1];

}

int  MDPManager::Coord2CellNum(std::vector<int> cell_xy)
{
	int index= cell_xy[0]+X_mapSize*cell_xy[1];


	return index;
}

void MDPManager::CellNum2Coord(const int Cell_idx, vector<int>& cell_xy)
{
	  cell_xy.resize(2,0);

	  int res =(int) Cell_idx / X_mapSize;
	  int div =(int) Cell_idx % X_mapSize;

	  cell_xy[0]=div;
	  cell_xy[1]=res;
}

bool MDPManager::MDPsolve()
{
	ROS_INFO("Solve");
	int colNum=6;
	int idx=0;

	Points.clear();
	Points.resize(colNum);
	//Check for row-wise or column-wise
	
	for(int j(0);j<Y_mapSize;j++){
		for(int i(0);i<X_mapSize;i++)	
		{
			Points[0].push_back(i);
			Points[1].push_back(j);
			Points[2].push_back((int)Rewards[idx]);
			//Points[2].push_back((int)R(i,j));
			Points[3].push_back(0);			//Up
			Points[4].push_back(0);			//U
			//Points[5].push_back(PiNums(i,j));
			Points[5].push_back(PolicyNum[idx]);
			idx++;
		}
	}	


	int 	iters = 0;
	double  diff  = 0.0;
	double  delta = 0.0;
	
	while(1)
	{
		if(Up.size()==U.size()){
			//cout<<"backup Up"<<endl;
			for(int i(0);i<Up.size();i++){
				U[i]=Up[i];
			}
		}
		
		for(int k(0);k<Num_Grids;k++){
			updateUprimePi(k);				//update Up and Policy, PolicyNum
			diff = abs(Up[k]-U[k]);

			//check maxim error for whole states
			if(diff > delta)
				delta=diff;
		}

		//terminal condition
		if(delta<deltaMin){
			cout<<"error converged"<<endl;
			break;
		}
		else if(iters>maxiter){
			cout<<"max iterations"<<endl;
			break;
		}
	
		iters++;
	}
	return false;
}

void MDPManager::printPath()
{
	for(int i(Y_mapSize-1);i>-1;i--){
		for(int j(0);j<X_mapSize;j++){

		int pos = i * X_mapSize + j;
		cout<<Rewards[pos]<<",";
		//cout<<Policies[pos];
		}
		cout<<endl;
	}

}

void MDPManager::setStaticObs(const vector<int> static_obs)
 {
 	for(int i(0);i<static_obs.size();i++){
 		pMapParam->OCC_Info[static_obs[i]]=St_OBS_CELL;
 	}

 }

 void MDPManager::setDynamicObs(const vector<int> dynamic_obs){

	for(int i(0);i<dynamic_obs.size();i++){
 		pMapParam->OCC_Info[dynamic_obs[i]]=Dy_OBS_CELL;
 	}

 }

 void MDPManager::setHumanObs(const vector<int> humans){

	for(int i(0);i<humans.size();i++){
	 		pMapParam->OCC_Info[humans[i]]=Human_CELL;
	 	}
 }

char MDPManager::getPolicychar(int policyidx)
{
	char policychars;
	switch(policyidx)
	{

		case 0:
			policychars='E';				break;
		case 1:
			policychars='R';				break;
		case 2:
			policychars='N';				break;
		case 3:
			policychars='Q';				break;
		case 4:
			policychars='W';				break;
		case 5:	
			policychars='Z';				break;
		case 6:
			policychars='S';				break;
		case 7:
			policychars='C';				break;
		default: 
			policychars='X';
	}

	return policychars;
}

void MDPManager::updateUprimePi(int state_id)
{
	vector<int> curpos;
	CellNum2Coord(state_id,curpos);
	int x_pos=curpos[0];
	int y_pos=curpos[1];
	// cout<<"---------------------"<<endl;
	// cout<<"state_id:"<<state_id<<"x pos :"<<x_pos<<", y pos :"<<y_pos<<endl;
	// cout<<"---------------------"<<endl;

	//for(int i(0);i<Num_Grids;i++)
	//cout<<"point[0][k] :"<<Points[0][i]<<", point[1][k] :"<<Points[1][i]<<endl;
	//cout<<"--------------num of grid :"<<Num_Grids<<endl;
	//cout<<"Update for Uprime : state_id :"<<state_id<<","<<x_pos<<","<<y_pos<<endl;
	
	for(int k(0);k<Num_Grids;k++){

		if(Points[0][k]==x_pos && Points[1][k]==y_pos){
			
			//Use bellman equation "computed using U(s), not using U'(s)"
			if(Rewards[k]!=Ra){	//Up(k)~=Ra)
				Up[k]=Rewards[k];
			}
			else{
				//cout<<"k:"<<k<<"x pos :"<<x_pos<<", y pos :"<<y_pos<<endl;	
				//get action index and the maximum reward value corresponding that action index
				map<int,double> maxmap;
				getMaxValueAction(x_pos,y_pos,maxmap);
				map<int,double>::iterator mnaxmapiter=maxmap.begin();
				

				//cout<<"maxvalue :"<<mnaxmapiter->second<<endl;
				//Update Uprime and Policyvector and policiesNumvector 
				// new_up_pi=[Up,Pi,PiNums] in matlab
				Up[k]=Rewards[k]+gamma*(mnaxmapiter->second);
				PolicyNum[k]=(mnaxmapiter->first);
				Policies[k]=getPolicychar(mnaxmapiter->first);

				//cout<<"k:"<<k<<", Up :"<<Up[k]<<", Pi[k] :"<<PolicyNum[k]<<endl;

			}
		}
	}
}

//returns true if cur_pos in inside a map // false is collision
bool MDPManager::checkNoBoundary(vector<int> cur_pos)
{
	int x_pos=cur_pos[0];
	int y_pos=cur_pos[1];

	if((x_pos<0) || (x_pos>X_mapSize-1))
		return false;
	if((y_pos<0) || (y_pos>Y_mapSize-1))
		return false;
		
	return true;

}
	
//returns true if cur_pos is safe (No collision with static obs, ex) Collision : false, NO collision : true
bool MDPManager::checkStaticObs(vector<int> cur_pos)
{
	// Input:current position 
	// Output : True or false

	int x_pos=cur_pos[0];
	int y_pos=cur_pos[1];

	for(int i(0);i<m_static_obs.size();i++)
	{
		vector<int> static_obs_pos;
		CellNum2Coord(m_static_obs[i],static_obs_pos);

		if((x_pos==static_obs_pos[0]) && (y_pos==static_obs_pos[1]))
			return false;
	}
	//cout<<"No static Obs"<<endl;
	return true;
}



bool MDPManager::checkObs(int cur_stid,int actionNum)
{
	// vector<int> cur_pos(2,0);
	// cur_pos=CellNum2Coord(cur_stid);
	// vector<int> next_pos(2,0);

	// next_pos[0]= cur_pos[0]+ActionCC[actionNum][0];
	// next_pos[1]= cur_pos[1]+ActionCC[actionNum][1];

	// cout<<"cur_pos: X : "<<cur_pos[0]<<", Y : "<<cur_pos[1]<<endl;
	// cout<<"action num : "<<actionNum<<endl;
	// cout<<"next_pos: X : "<<next_pos[0]<<", Y : "<<next_pos[1]<<endl;

	// if(checkNOBoundary(next_pos) && checkStaticObs(next_pos))
	// {
	// 	int next_pos_id=Coord2CellNum(next_pos);


	// }

}


double MDPManager::getactionvalue(int x_pos, int y_pos, int action_ix)
{
	//Get Current pos coordinate
	vector<int> cur_pos(2,0);
	cur_pos[0]=x_pos;
	cur_pos[1]=y_pos;
	
	int cur_st_id=Coord2CellNum(cur_pos);

	//Get next pos coordinate
	vector<int> next_pos(2,0);
	next_pos[0]= x_pos+ActionCC[action_ix][0];
	next_pos[1]= y_pos+ActionCC[action_ix][1];

	 //cout<<"cur st_id :"<<cur_st_id<<endl;
	//  cout<<"cur_pos: X : "<<x_pos<<", Y : "<<y_pos<<endl;
	//  cout<<"action num : "<<action_ix<<endl; 
	//  cout<<"next_pos: X : "<<next_pos[0]<<", Y : "<<next_pos[1]<<endl;

	//Cehck obstacles
	//check static obstacle
	if(checkNoBoundary(next_pos) && checkStaticObs(next_pos))
	{
		//cout<<"next_pos: X : "<<next_pos[0]<<", Y : "<<next_pos[1]<<endl;
		//cout<<"No collision"<<endl;
		int next_pos_id=Coord2CellNum(next_pos);
		return U[next_pos_id];
		//Return U
	}
	else{
		//cout<<"collision"<<endl;
		//cout<<"cur_st_id : "<<cur_st_id<<endl;
		return U[cur_st_id];
	}
}

vector<int> MDPManager::getneighboractionset(int action_idx)
{
	vector<int> neighboractionset(2,0);

	if(action_idx==0){
		neighboractionset[0]=(action_idx+1);
		neighboractionset[1]=7;	
	}
	else if(action_idx==7){
		neighboractionset[0]=0;
		neighboractionset[1]=6;	
	}
	else{
		neighboractionset[0]=(action_idx+1)%Action_dim;
		neighboractionset[1]=(action_idx-1)%Action_dim;
	}

	//cout<<"id:"<<action_idx<<", x :"<<neighboractionset[0]<<", y:"<<neighboractionset[1]<<endl;
	return neighboractionset;
}

int MDPManager::FindMaxIdx(vector<double> dataset)
{
	map<int,double> MaxIndex;
	MaxIndex.insert(make_pair(0,dataset[0]));
	int maxIndex=0;

	map<int,double>::iterator mapIter = MaxIndex.begin();

	for(int i=1;i<dataset.size();i++)
		{
			mapIter = MaxIndex.begin();		
			
			if(mapIter->second<dataset[i])
			{
				MaxIndex.clear();
				MaxIndex.insert(make_pair(i,dataset[i]));
				maxIndex=i;
			}
	}
	return maxIndex;

}
void MDPManager::Mapcoord2GlobalCoord(const vector<int>& _Mapcoord, vector<double>& GlobalCoord)
{

	GlobalCoord.resize(2);
	//globalCoord origin x, y;
	GlobalCoord[0]=Scaled_static_map.info.origin.position.x+Scaled_static_map.info.resolution*_Mapcoord[0]+0.5*Scaled_static_map.info.resolution;
	GlobalCoord[1]=Scaled_static_map.info.origin.position.y+Scaled_static_map.info.resolution*_Mapcoord[1]+0.5*Scaled_static_map.info.resolution;

	GlobalCoord[0]=Scaled_static_map.info.origin.position.x+Scaled_static_map.info.resolution*_Mapcoord[0]+0.5*Scaled_static_map.info.resolution;
	GlobalCoord[1]=Scaled_static_map.info.origin.position.y+Scaled_static_map.info.resolution*_Mapcoord[1]+0.5*Scaled_static_map.info.resolution;

}

void MDPManager::Mapcoord2DynamicCoord(const vector<int>& _Mapcoord, vector<double>& dynamicCoord)
{
	dynamicCoord.resize(2);
	dynamicCoord[0]=Scaled_dynamic_map.info.origin.position.x+Scaled_dynamic_map.info.resolution*_Mapcoord[0]+0.5*Scaled_dynamic_map.info.resolution;
	dynamicCoord[1]=Scaled_dynamic_map.info.origin.position.y+Scaled_dynamic_map.info.resolution*_Mapcoord[1]+0.5*Scaled_dynamic_map.info.resolution;


}




//get action(best) in Matlab
void MDPManager::getMaxValueAction(int x_pos, int y_pos,map<int,double>& maxmap)
{
	double bestvalue=0.0;
		
	vector<double> action_value(Action_dim,0.0);
	vector<double> action_valueSum(Action_dim,0.0);
	vector<int>    neighbor_action(2,0);
	
	for(int i(0);i<Action_dim;i++)
	{
		action_value[i]=getactionvalue(x_pos, y_pos,i);
		//cout<<"action value:"<<action_value[i]<<endl;
	}

	for (int i(0);i<Action_dim;i++)
	{
		action_valueSum[i]=Prob_good*action_value[i];
		neighbor_action=getneighboractionset(i);

		for(int j(0);j<2;j++)
			action_valueSum[i]+=Prob_bad*action_value[neighbor_action[j]];			
	}

	//cout<<"Here 3"<<endl;
	int maxIdx=FindMaxIdx(action_valueSum);
	bestvalue = action_valueSum[maxIdx];

	// for(int z(0);z<action_valueSum.size();z++)
	// 	cout<<"idx :"<<z<<", value :"<<action_valueSum[z]<<endl;
	// cout<<"maxidx : "<<maxIdx<<", maxvalue: "<<bestvalue<<endl;
	maxmap.clear();
	maxmap.insert(make_pair(maxIdx,bestvalue));

	return;
}

//Function for generating path
void MDPManager::pathPublish(){

	// std_msgs::Int32MultiArray obsmap_msg;
	// obsmap_msg.data = m_static_obs;
	// obsmap_Pub.publish(obsmap_msg);
	// MDPPath.clear();

}
//Generate dynamic path from solution
void MDPManager::generate_dynamicPath()
{
	vector<int> cur_pos(2,0);
	for(int i(0);i<2;i++)
		cur_pos[i]=m_Start[i];

	Dyn_MDPPath.clear();
	int cur_stid=Coord2CellNum(m_Start);
	int goal_stid=Coord2CellNum(m_Goal);

	if((goal_stid>pMapParam->MapSize) || (goal_stid<0))
		return;

	vector<double> t_values;
	vector<double> x_values;
	vector<double> y_values;
	vector<double> dyn_global_coords;

	//double* = new double [13]

	// x_values.push_back(m_Start[0]);
	// y_values.push_back(m_Start[1]);

	Mapcoord2DynamicCoord(m_Start,dyn_global_coords);
	x_values.push_back(dyn_global_coords[0]);
	y_values.push_back(dyn_global_coords[1]);

	cout<<"cur st id : "<<cur_stid<<endl;
	Dyn_MDPPath.push_back(cur_stid);
	int pathcount=0;

	while(1)
	{
		cur_stid=Coord2CellNum(cur_pos);
		if(cur_stid==goal_stid)
			break;

		//get next position from policy solution
		for(int i(0);i<2;i++)
			cur_pos[i]+=ActionCC[PolicyNum[cur_stid]][i];

		Mapcoord2DynamicCoord(cur_pos,dyn_global_coords);
		ROS_INFO("cur pos  : %d, x:  %d, y // global goal x : %.3lf , global goal y : %.3lf \n",cur_pos[0], cur_pos[1],dyn_global_coords[0],dyn_global_coords[1]);

		x_values.push_back(dyn_global_coords[0]);
		y_values.push_back(dyn_global_coords[1]);
		cur_stid=Coord2CellNum(cur_pos);
		Dyn_MDPPath.push_back(cur_stid);

		//ROS_INFO("Id : %d, x:  %d, y : %d \n",cur_stid, cur_pos);

		if(pathcount>20)
			return;

		pathcount++;
		
	}

	//Making Spline path=======================
	int data_size=x_values.size();
	for(int k=0;k<data_size;k++)
		printf("spline path index : %d, x_values : %lf , y values : %lf \n", k,x_values[k],y_values[k]);

	//Making t-vetors;
	t_values.resize(x_values.size());
	//t_values[0]=0;
	double time_length=1.0;
	double time_const=(time_length)/(data_size-1);
	for(int k(0);k<data_size;k++)
		t_values[k]=k*time_const;
	
	m_CubicSpline_x = new srBSpline;
	m_CubicSpline_x->_Clear();

	m_CubicSpline_y = new srBSpline;
	m_CubicSpline_y->_Clear();

	vector<double> Spline_x;
	vector<double> Spline_y;
	m_CubicSpline_x->CubicSplineInterpolation(t_values,x_values,x_values.size());
	m_CubicSpline_y->CubicSplineInterpolation(t_values,y_values,y_values.size());

	int path_size=6;
	if(data_size<3)
		path_size=2;

	double const_path=(time_length)/ path_size ;

	Spline_x.push_back(x_values[0]);
	Spline_y.push_back(y_values[0]);

	double ret_x=0.0;
	double ret_y=0.0;
	double t_idx=0.0;
	 for(int j(0);j<path_size;j++){
  	   t_idx=(j+1)*const_path;
       m_CubicSpline_x->getCurvePoint(ret_x,t_idx);
       m_CubicSpline_y->getCurvePoint(ret_y,t_idx);

       if(!std::isnan(ret_x))
       	Spline_x.push_back(ret_x);
       if(!std::isnan(ret_y))
       	Spline_y.push_back(ret_y);
     }

    
    //Publish SPline Path
  	nav_msgs::Path dynamicSplinePath;
  	dynamicSplinePath.header.frame_id = "map";
  	geometry_msgs::PoseStamped pose;
  	

	// pose.header.frame_id = "map_local";
	// ROS_INFO("Desired Yaw angle : %.3lf",m_desired_heading*180/(3.141592));

	 for (int i = 0; i < Spline_x.size(); i++)
 	 {
	    pose.pose.position.x=Spline_x[i];
	   	pose.pose.position.y=Spline_y[i];
	    // pose.pose.position.x=Spline_x[i]*0.25-3.5-0.5*0.25;
	   	// pose.pose.position.y=Spline_y[i]*0.25-3.5-0.5*0.25;
	   	printf("spline path2 index : %d, x coord : %lf , y coord : %lf \n", i,pose.pose.position.x,pose.pose.position.y);
	   	pose.pose.orientation = tf::createQuaternionMsgFromYaw(m_desired_heading);
	   	dynamicSplinePath.poses.push_back(pose);
	}

	 //SplinePath_pub.publish(path);
	 SplinePath_pub2.publish(dynamicSplinePath);
	
	 // Pre_dynamicSplinePath=dynamicSplinePath;
	 // Pre_dynamicSplinePath.header.frame_id=dynamicSplinePath.header.frame_id;
	 // Pre_dynamicSplinePath.poses=dynamicSplinePath.poses;


	 // Publish static map_path_grid
	for(int j(0);j<Scaled_dynamic_map_path.data.size();j++)
	 {	
	 	Scaled_dynamic_map_path.data[j]=0.0;
	 }


	 for(int k(0); k<Dyn_MDPPath.size();k++)
	 	Scaled_dynamic_map_path.data[Dyn_MDPPath[k]]=90;

	 Scaled_dynamic_map_path.header.stamp =  ros::Time::now();
	 Scaled_dynamic_map_path.header.frame_id = "map"; 
     Scaled_dynamic_map_path_pub.publish(Scaled_dynamic_map_path);

    //MDP solution publsih
 //    std_msgs::Int32MultiArray MDPsolution_msg;
	
	// MDPsolution_msg.data.resize(PolicyNum.size()); 
	// for(int i(0);i<PolicyNum.size();i++)
	// 	MDPsolution_msg.data[i]=PolicyNum[i];
	// MDPSol_pub.publish(MDPsolution_msg);

	//Publish path after planning
	// std_msgs::Int32MultiArray pathmap_msg;
	// pathmap_msg.data = MDPPath;
	// Path_Pub.publish(pathmap_msg);
	// ROS_INFO("publish");

}


void MDPManager::publishZeropaths()
{

	  	nav_msgs::Path ZeroSplinePath;
  		ZeroSplinePath.header.frame_id = "map";
  		geometry_msgs::PoseStamped zeropose;
  	
	
	    zeropose.pose.position.x=CurVector[0];
	   	zeropose.pose.position.y=CurVector[1];
	   	zeropose.pose.orientation = tf::createQuaternionMsgFromYaw(m_desired_heading);
	   	ZeroSplinePath.poses.push_back(zeropose);
	   	// printf("spline path2 index : %d, x coord : %lf , y coord : %lf \n", i,zeropose.pose.position.x,zeropose.pose.position.y);
	
	SplinePath_pub2.publish(ZeroSplinePath);

}


void MDPManager::publishpaths()
{
	// if(booltrackHuman)
 	// {
 		SplinePath_pub.publish(path);
		//SplinePath_pub2.publish(Pre_dynamicSplinePath);
		Scaled_dynamic_map_path_pub.publish(Scaled_dynamic_map_path);
	// }
	// else
	// {	
	// 	//publishZeropaths();

	// }
}

//Generate path from solution
void MDPManager::generatePath()
{
	vector<int> cur_pos(2,0);
	for(int i(0);i<2;i++)
		cur_pos[i]=m_Start[i];

	MDPPath.clear();
	int cur_stid=Coord2CellNum(m_Start);
	int goal_stid=Coord2CellNum(m_Goal);

	
	vector<double> t_values;
	vector<double> x_values;
	vector<double> y_values;
	vector<double> global_coords;

	//double* = new double [13]

	// x_values.push_back(m_Start[0]);
	// y_values.push_back(m_Start[1]);

	Mapcoord2GlobalCoord(m_Start,global_coords);
	x_values.push_back(global_coords[0]);
	y_values.push_back(global_coords[1]);

	cout<<"cur st id : "<<cur_stid<<endl;
	MDPPath.push_back(cur_stid);
	
	while(1)
	{
		//get next position from policy solution
		for(int i(0);i<2;i++)
			cur_pos[i]+=ActionCC[PolicyNum[cur_stid]][i];

		Mapcoord2GlobalCoord(cur_pos,global_coords);
		ROS_INFO("cur pos Id : %d, x:  %d, y // global x : %.3lf , global y : %.3lf \n",cur_pos[0], cur_pos[1],global_coords[0],global_coords[1]);

		x_values.push_back(global_coords[0]);
		y_values.push_back(global_coords[1]);

		cur_stid=Coord2CellNum(cur_pos);
		MDPPath.push_back(cur_stid);

		//ROS_INFO("Id : %d, x:  %d, y : %d \n",cur_stid, cur_pos);

		if(cur_stid==goal_stid)
			break;
	}

	//Making Spline path=======================
	int data_size=x_values.size();
	//for(int k=0;k<data_size;k++)
		//printf("spline path index : %d, x_values : %lf , y values : %lf \n", k,x_values[k],y_values[k]);

	//Making t-vetors;
	t_values.resize(x_values.size());
	//t_values[0]=0;
	double time_length=1.0;
	double time_const=(time_length)/data_size;
	for(int k(0);k<data_size;k++)
		t_values[k]=k*time_const;
	
	m_CubicSpline_x = new srBSpline;
	m_CubicSpline_x->_Clear();

	m_CubicSpline_y = new srBSpline;
	m_CubicSpline_y->_Clear();

	vector<double> Spline_x;
	vector<double> Spline_y;
	m_CubicSpline_x->CubicSplineInterpolation(t_values,x_values,x_values.size());
	m_CubicSpline_y->CubicSplineInterpolation(t_values,y_values,y_values.size());

	int path_size=8;	
	double const_path=(time_length)/ path_size ;

	Spline_x.push_back(x_values[0]);
	Spline_y.push_back(y_values[0]);

	double ret_x=0.0;
	double ret_y=0.0;
	double t_idx=0.0;
	 for(int j(0);j<path_size;j++){
  	   t_idx=(j+1)*const_path;
       m_CubicSpline_x->getCurvePoint(ret_x,t_idx);
       m_CubicSpline_y->getCurvePoint(ret_y,t_idx);

       Spline_x.push_back(ret_x);
       Spline_y.push_back(ret_y);
     }

     //Publish SPline Path
  	nav_msgs::Path path ;
  	path.header.frame_id = "map";
  	geometry_msgs::PoseStamped pose;
	// pose.header.frame_id = "map_local";

	 for (int i = 0; i < Spline_x.size(); i++)
 	 {	    	    
	    pose.pose.position.x = Spline_x[i];
	    pose.pose.position.y = Spline_y[i];
	    printf("spline path index : %d, x coord : %lf , y coord : %lf \n", i,pose.pose.position.x,pose.pose.position.y);
	    pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.05);
	    path.poses.push_back(pose);
	}
	 
	 SplinePath_pub.publish(path);

	 // Publish static map_path
	for(int j(0);j<Scaled_static_map_path.data.size();j++)
	 {	
	 	Scaled_static_map_path.data[j]=0.0;
	 }


	 for(int k(0); k<MDPPath.size();k++)
	 	Scaled_static_map_path.data[MDPPath[k]]=90;

	 Scaled_static_map_path.header.stamp =  ros::Time::now();
	 Scaled_static_map_path.header.frame_id = "map"; 
     Scaled_static_map_path_pub.publish(Scaled_static_map_path);

    //MDP solution publsih
    std_msgs::Int32MultiArray MDPsolution_msg;
	
	MDPsolution_msg.data.resize(PolicyNum.size()); 
	for(int i(0);i<PolicyNum.size();i++)
		MDPsolution_msg.data[i]=PolicyNum[i];
	MDPSol_pub.publish(MDPsolution_msg);

	//Publish path after planning
	// std_msgs::Int32MultiArray pathmap_msg;
	// pathmap_msg.data = MDPPath;
	// Path_Pub.publish(pathmap_msg);
	// ROS_INFO("publish");

}	



