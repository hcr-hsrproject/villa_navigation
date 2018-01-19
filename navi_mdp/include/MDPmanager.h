#include <vector>
#include <fstream>
#include <string>
#include <map>
#include <stdint.h>
#include "ros/ros.h"
#include <ros/package.h>
#include <eigen3/Eigen/Dense>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8.h"
#include "srBSpline.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/tf.h>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include <tf/transform_datatypes.h>
#include <cmath>
#include <cfloat>

#define FREE_CELL 0
#define St_OBS_CELL 1
#define Dy_OBS_CELL 1
#define Human_CELL 3

#define Grid_STEP 0.5
#define Grid_Num_X 24
#define Grid_Num_Y 24

#define Start_X 1 
#define Start_Y 1
//80 20
#define Goal_X 2
#define Goal_Y 2

#define DYN_OFFSET_X 3.5
#define DYN_OFFSET_Y 3.5


#define Num_action 8
#define deltaMin 1E-05
#define Maxiteration 300

#define ra (-1.0)
using namespace Eigen;
using namespace std;

class MapParam
{
public :

	MapParam();
	MapParam(int width_,int height_,double res_);
	~MapParam();

    double   		  	map_step;
   	float 		  		m_cell_x_width;
	float 		  		m_cell_y_width;
	int   		  		Num_grid_X;
	int   		  		Num_grid_Y;
	int                 MapSize;
	bool                boolDynamic;

	std::vector<int>    Cell_Info;
	std::vector<int>    OCC_Info;
	std::vector<int>    Robot_localpos;
	std::vector<int>    State_Type;
	std::vector<float>  State_Distance;
	std::vector<float>  NearestHuman_V;
	std::vector<float>  RobotHeading_V;

	void setWidth (int width_);
	void setHeight(int height_);
	void setResolution (double resolution);
	void set_Cell_Info(std::vector<int> _inputCellInfo);
	void set_OCC_Info(std::vector<int> _inputOCCInfo);
	void set_Robot_Info(std::vector<int> _inputRobotInfo);
	void set_State_Type(std::vector<int> _State_Type);
	void set_State_Distance(std::vector<float> _State_Distance);
	void set_NearestHuman_V(std::vector<float> _NearestHuman_V);
	void set_RobotHeading_V(std::vector<float> _RobotHeading_V);
};


class MDPManager
{
 public:
 	MDPManager(MapParam* _pMapParam);
 	MDPManager():maxiter(Maxiteration),Action_dim(8),gamma(1),Ra(ra),publishnum(0),m_boolSolve(false){}
 	~MDPManager();

 	MapParam* 	pMapParam;  

 	tf::TransformListener 	  listener;
	vector< std::vector<int> > Points;
	vector< vector<int> > ActionCC;
 	vector<int>		  m_Start;							//Start position of(x,y)
  	vector<int>       m_Goal;							//Goal position of (x,y)
 	vector<int>       m_Robot;					    	//Current Robot position of (x,y)
 	int 	          Feature_dim;
 	int               X_mapSize;
 	int               Y_mapSize;
 	int               Num_Grids;
 	int               State_dim;
 	int               Action_dim;
 	vector<char>  	  Policies;		// Policy (Pi)
 	vector<double>    Rewards; 		// R
 	vector<int> 	  MdpSols;		// Solution of MDP
 	vector<int>  	  PolicyNum;	// Policy (PiNum)
 	vector<double>	  Up;			// Uprime, used in updates
 	vector<double>	  U;			// Long term Utility


 	vector<int>		  m_static_obs;
 	vector<int>		  m_dynamic_obs;
 	vector<int>		  m_human_obs;
 	vector<int>		  cell_xy;
 	vector<int>       m_localoccupancy;
 	vector<int>       m_dynamic_occupancy;
 	vector<float>	  human_global;
 	int               human_callback_count;


 	std::vector<double> global_pose;
 	std::vector<double> Map_orig_Vector;
	std::vector<double> CurVector;
	std::vector<double> GoalVector;
	std::vector<double> HeadingVector;
	std::vector<int> cur_coord;
	std::vector<int> Goal_Coord;
	std::vector<int> Human_Goal_Coord;
	std::vector<int> MapCoord;
 	
 	double Ra;
 	double gamma;
	double Prob_good;
	double Prob_bad;
 	
 	int Local_X_start;
 	int Local_Y_start;

 	int maxiter;
 	int publishnum;
 	int ReceiveData;
 	vector<int>  MDPPath;
 	vector<int>  Dyn_MDPPath;

 	srBSpline*           m_Spline;
 	srBSpline*           m_CubicSpline_x;
 	srBSpline*           m_CubicSpline_y;

 
 	bool    m_boolSolve;
 	int     dyn_path_num;
 	double	m_desired_heading;
 	
	ros::NodeHandle  m_node; 	
	ros::Publisher   obsmap_Pub;
	ros::Publisher   Scaled_static_map_pub;
	ros::Publisher   Scaled_static_map_path_pub;
	ros::Publisher   Scaled_dynamic_map_pub;
	ros::Publisher   Scaled_dynamic_map_path_pub;

	ros::Publisher   Path_Pub;
	ros::Subscriber  Localmap_sub;
	ros::Publisher 	 SplinePath_pub;
	ros::Publisher 	 SplinePath_pub2;
	ros::Publisher   UnitGoalVec_pub;
	ros::Publisher   MDPSol_pub;
	ros::Publisher   RobotHeading_pub;
	

	//Static_mdp
	int  scaling=12;
	nav_msgs::OccupancyGrid Scaled_static_map;
	nav_msgs::OccupancyGrid Scaled_static_map_path;

	nav_msgs::OccupancyGrid Scaled_dynamic_map;
	nav_msgs::OccupancyGrid Scaled_dynamic_map_path;

	bool       booltrackHuman;
	nav_msgs::Path path;
	nav_msgs::Path Pre_dynamicSplinePath;

	//functions
 	void 			Init();								 //Initialize function
 	void 			setPMapParam(MapParam* _pMapParam);
 	void 			setStartConfig (const vector<int> Start );
 	void			setGoalConfig  (const vector<int> Goal);
 	void  			setStaticObs(const vector<int> static_obs);
 	void  			setDynamicObs(const vector<int> dynamic_obs);
 	void  			setHumanObs(const vector<int> humans);
 	void 			CellNum2Coord(const int Cell_idx, vector<int>& cell_xy);
 	int  			Coord2CellNum(vector<int> cell_xy);
 	vector<int>     Global2LocalCoord(vector<int> Global_coord);
 	bool            MDPsolve();

 	void			updateUprimePi(int state_id);
 	void 			getMaxValueAction(int x_pos,int y_pos,map<int,double>& maxmap);
 	double 			getactionvalue(int x_pos, int y_pos, int action_ix);
 	bool			checkObs(int cur_stid,int actionNum);
 	bool			checkNoBoundary(vector<int> cur_pos);
 	bool			checkStaticObs(vector<int> cur_pos);
 	vector<int> 	getneighboractionset(int action_idx);
 	int             FindMaxIdx(vector<double> dataset);
 	char            getPolicychar(int policyidx);
 	void			printPath();
 	void 			generatePath();
 	void 			generate_dynamicPath();
 	void            pathPublish();
 	void  			updateMap(vector<int>& localmap_,vector<int>& local_start, vector<int>& local_goal);
 	void 			Local_mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
 	void 			static_mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
 	void 			dynamic_mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
 	void			ClikedpointCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
 	void 			base_pose_callback(const nav_msgs::Odometry::ConstPtr& msg);
 	void 			Basepos_Callback(const geometry_msgs::PointStamped::ConstPtr& msg);
 	void 			Global2MapCoord(const vector<double>& _globalcoord, vector<int>& MapCoord);
 	void 			Human_target_cmdCallback(const std_msgs::Int8::ConstPtr& msg);
 	void			Human_MarkerCallback(const visualization_msgs::Marker::ConstPtr& msg);
 	void    		CoordinateTransform_Rviz_Grid_Start(double _x, double _y,int map_type);
	void    		CoordinateTransform_Rviz_Grid_Goal(double _x, double _y,int map_type);
	void 			CoordinateTransform_Rviz_Grid_Human(double _x, double _y,int map_type);
	void            Mapcoord2GlobalCoord(const vector<int>& _Mapcoord, vector<double>& GlobalCoord);
	void 			Mapcoord2DynamicCoord(const vector<int>& _Mapcoord, vector<double>& dynamicCoord);
	void			MDPsolPublish();
	void 			publishpaths();
	bool            IsinDynamicMap(float global_x, float global_y);
	bool            IsTargetMoved(float global_x, float global_y,float criterion);
	void 			setDesiredHeading(double _heading);
	double			getdistance(vector<double> cur, vector<double> goal);
	void 			publishZeropaths();
	void 			global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

};

