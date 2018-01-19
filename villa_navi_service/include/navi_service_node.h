#include <vector>
#include <fstream>
#include <string>
#include <map>
#include <stdint.h>
#include "ros/ros.h"
#include <ros/package.h>
#include <Eigen/Dense>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/PoseArray.h"
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <villa_navi_service/GoTargetPos.h>

#define FOVW 29				//field of view width
#define MATH_PI 3.14159265359
#define P_H 0.2 // Prior prob
#define P_S_given_H 0.8
#define P_S_given_Hc 0.5

#define P_Sc_given_H 0.01
#define P_Sc_given_Hc 0.99

#define Same_POS_diff 0.5
#define MAX_UPDATE_ITER 50
#define MAX_VIEW_UPDATE_ITER 100
#define LASER_ANGLE_RES 0.25
#define LASER_Data_Length 914
#define LASER_Point_Step 16


#define LASER_ANGLE_MIN -2.09875845909
#define LASER_ANGLE_MAX 2.09875845909
#define LASER_ANGLE_STEP 0.00436332309619



class villa_navi_srv{

public:
	villa_navi_srv();
	~villa_navi_srv();


	ros::Publisher static_belief_map_pub;
	ros::Publisher belief_pub;
	ros::Publisher human_target_pub;
	ros::Publisher human_leg_target_pub;
	ros::Publisher human_target_Intcmd_pub;
	ros::Publisher Headscan_pub;
	ros::Publisher Human_boxes_pub;
	ros::Publisher Gaze_point_pub;
	ros::Publisher Gaze_activate_pub;
	ros::Publisher setNavTarget_pub;
	ros::Publisher human_laser_pub;
	ros::Publisher human_laser_scan_pub;
	
	int index;
	int m_numofhuman;
	int m_receiveiter;
	int m_yolo_recieveiter;
	int m_updateiter;
	int m_viewupdateiter;
	int m_leg_updateiter;
	int belief_size;
	int targetup;
	tf::TransformListener 	  listener;

	std::vector<double> Robot_Pos;				//x,y,theta
	std::vector<double> Head_Pos;				//x,y,theta
	std::vector<double> global_pose;
	std::vector<double> Human_target_candidate;				//x,y,theta
	std::vector<double> Track_human_target;				//x,y,theta
	std::vector<double> leg_target;				//x,y,theta
	std::vector< std::vector< double > > Last_detected_human;
	std::vector< std::vector< double > > Cur_detected_human;
	std::vector< std::vector< double > > Cur_existed_human;
	std::vector< std::vector< double > > Cur_leg_human;
	std::vector< std::vector< double > > Cur_leg_yolo_human;
	std::vector< double >  viewpoint_robot;
	std::vector<double> angle_people_set;
	std::vector<double> angle_criticalvalue;

	std::vector<int> human_occupied_idx;
	std::vector<int> human_occupied_leg_idx;
	std::vector<int> visiblie_idx_set;

	std::vector<int> index_of_human_occ_cells_updated_recently;
	std::map<int, float> map_index_of_human_cells_to_prob;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	
	nav_msgs::OccupancyGrid dynamic_belief_map;
	nav_msgs::OccupancyGrid human_belief_map;
	nav_msgs::OccupancyGrid static_belief_map;
	std_msgs::Int8 track_cmd;
	visualization_msgs::MarkerArray human_boxes_array;
	bool OnceTargeted;


	void Init_parameters();
	void InitializeBelief();
	void Publish_nav_target(float _x, float _y, float _theta);
	void setViewpointTarget(const std::vector<double> pos);
	bool goTarget(villa_navi_service::GoTargetPos::Request &req, villa_navi_service::GoTargetPos::Response &res);	

	std::vector<double> m_dyn_occupancy;
	std::vector<double> m_prob_occupancy;
	std::vector<int>    m_yolo_idx_set;
	std::vector<int>    m_leg_idx_set;
	std::vector<double> m_human_posx;
	std::vector<double> m_human_posy;


	int pub_iters;
	int detect_iters;
	double Camera_angle;
	int print_iter;

};