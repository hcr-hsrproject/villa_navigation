#include <string>
#include <boost/thread/mutex.hpp>


// ros stuff
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include <tf/transform_datatypes.h>
#include <keyboard/Key.h>


// people tracking stuff
#include "tracker.h"
#include "detector_particle.h"
#include "gaussian_vector.h"

// messages
#include <std_msgs/Int8.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud.h>
#include <people_msgs/PositionMeasurement.h>
#include <message_filters/time_sequencer.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/OccupancyGrid.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// log files
#include <fstream>
#include <vector>


#define FOVW 60       //field of view width
#define MATH_PI 3.14159265359
#define Target_Dist_person 1.0
#define LASER_Dist_person  2.5

class Edgeleg_manager
{
public:
  /// constructor
  Edgeleg_manager(ros::NodeHandle nh);

  /// destructor
  virtual ~Edgeleg_manager();

  /// callback for messages
  void callbackRcv(const people_msgs::PositionMeasurement::ConstPtr& message);

  /// callback for dropped messages
  void callbackDrop(const people_msgs::PositionMeasurement::ConstPtr& message);


  void filter_result_callback(const people_msgs::PositionMeasurement::ConstPtr& msg);
  void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);
  void edge_leg_callback(const geometry_msgs::PoseArray::ConstPtr& message);
  void global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void human_yolo_callback(const visualization_msgs::MarkerArray::ConstPtr& message);
  void filter_act_callback(const std_msgs::Int8::ConstPtr& msg);
  void keyboard_callback(const keyboard::Key::ConstPtr& msg);
  void scaled_static_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  bool check_staticObs(float x_pos,float y_pos);
  int globalcoord_To_SScaled_map_index(float x_pos,float y_pos);
  bool getlinevalue(int line_type,double input_x, double input_y);

  int CoordinateTransform_Global2_staticMap(float global_x, float global_y);

  bool check_cameraregion(float x_pos,float y_pos);
  int FindNearesetLegIdx();
  void publish_leg_boxes();
  void getCameraregion();
  void publish_cameraregion();
  void publish_target();
  void Publish_nav_target();
  void publish_filtered_target();

  double getDistance_from_Vec(std::vector<double> origin, double _x, double _y);
  bool Comparetwopoistions(std::vector<double> pos,std::vector<double> pos2,double criterion);
  /// tracker loop11
  void spin();


private:

  ros::NodeHandle nh_;

  ros::Publisher people_measurement_pub_;
  ros::Publisher One_People_pos_pub;
  ros::Publisher static_belief_map_pub;
  ros::Publisher human_target_pub;
  ros::Publisher filtered_human_target_pub;
  ros::Publisher setNavTarget_pub;



  ros::Subscriber people_meas_sub_;
  ros::Subscriber people_yolo_sub;
  ros::Subscriber edge_leg_sub;
  ros::Subscriber filter_act_sub;
  ros::Subscriber globalpose_sub;
  ros::Subscriber keyboard_sub;
  ros::Subscriber Scaled_static_map_sub;
  ros::Subscriber filter_result_sub;
  ros::Subscriber sound_cmd_sub;
  

  ros::Publisher Leg_boxes_pub;
  visualization_msgs::MarkerArray human_leg_boxes_array;

  // tf listener
  tf::TransformListener robot_state_;
  tf::TransformListener     listener;

  unsigned int filter_counter_;
  double freq_, start_distance_min_, reliability_threshold_;
  
  std::string fixed_frame_;
  boost::mutex filter_mutex_;

  sensor_msgs::PointCloud  meas_cloud_;
  unsigned int meas_visualize_counter_;

  std::vector<int> visiblie_idx_set;

  // Track only one person who the robot will follow.
  bool follow_one_person_;

  std::vector<double> global_pose;
  std::vector< std::vector< double > > Cur_leg_human;
  std::vector< std::vector< double > > cur_yolo_people;
  std::vector<double> leg_target;
  std::vector<double> filtered_leg_target;
  std::vector<double> Head_Pos; 

  int num_of_detected_human;
  nav_msgs::OccupancyGrid Scaled_map;
  nav_msgs::OccupancyGrid static_belief_map;
  int edge_leg_iter;
  int belief_size;
  int pub_iters;
  bool OnceTarget;

}; // class


