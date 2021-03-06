#include "ros/ros.h"
#include "laser_processor.h"
#include "calc_leg_features.h"

#include "opencv2/ml.hpp"
#include <opencv2/opencv.hpp>
#include "opencv2/core/utility.hpp"
#include "people_msgs/PositionMeasurement.h"
#include "people_msgs/PositionMeasurementArray.h"
#include <people_msgs/Person_v2.h>
#include <people_msgs/People_v2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
//#include <visualization_msgs/MarkerArrary.h>
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Header.h"

#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

#include "people_tracking_filter/tracker_kalman.h"
#include "people_tracking_filter/gaussian_pos_vel.h"

#include "people_tracking_filter/state_pos_vel.h"
#include "people_tracking_filter/rgb.h"

#include <algorithm>

#include <tf/transform_broadcaster.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"

#include <dynamic_reconfigure/server.h>
//#include <human_filter/LegDetectionConfig.h>
#include <algorithm>
// services

using namespace std;
using namespace laser_processor;
using namespace ros;
using namespace tf;
using namespace estimation;
using namespace BFL;
using namespace MatrixWrapper;

static const double no_observation_timeout_s = 0.5;
static const double max_second_leg_age_s     = 2.0;   //2.0
static const double max_track_jump_m         = 1.0; //1.0;
static const double max_meas_jump_m          = 0.75; //0.75; // 1.0
static const double leg_pair_separation_m    = 0.45; //previos 0.5 mkmk
//static const string fixed_frame              = "odom_combined";
static const string fixed_frame              = "map";
static const string GLOBAL_FRAME              = "map";
static const string laser_frame              = "base_range_sensor_link";
static const string LASER_FRAME             = "base_range_sensor_link";
static double cov_meas_legs_m          = 0.025;
static double cov_meas_people_m        = 0.025;

static double kal_p =4, kal_q = 0.002, kal_r = 10;
static bool use_filter = true;
static string detector_="hsrb/base_scan";
//static const unsigned int num_particles=100; // particle

class SavedPersonFeature
{
public:

	string id_;
	string object_id;
	string person_name;

	Stamped<Point> position_;
	Stamped<Vector3> velocity_;

	//static int nextid;
	//TransformListener& tfl_;
	BFL::StatePosVel sys_sigma_;
	TrackerKalman filter_;
	ros::Time time_;
	ros::Time meas_time_;

	double reliability, p, probability;

	// person tracker
	SavedPersonFeature(Stamped<Point> loc, std::string& id, std::string& name)
	: sys_sigma_(Vector3(0.05, 0.05, 0.05), Vector3(1.0, 1.0, 1.0)),
	  filter_("tracker_people",sys_sigma_),
	  reliability(-1.), p(4)
	{
		object_id = id;
		person_name = name;
		time_ = loc.stamp_;

		//P-Matrix = covariance matrix
		//Q-Matrix = process noise covariance matrix
		//F-Matrix = state matrix

		StatePosVel prior_sigma(Vector3(0.1,0.1,0.1), Vector3(0.0000001, 0.0000001, 0.0000001));
		filter_.initialize(loc, prior_sigma, time_.toSec());
		StatePosVel est;
		filter_.getEstimate(est);
		updatePosition();
	}
	/*
	 * predicts parameter values ahead of current
	 * measurements
	 */
	void propagate(ros::Time time)
	{
		time_ = time;
		filter_.updatePrediction(time.toSec());
		updatePosition();
	}

	/*
	 * updates current values with correction:
	 * estimates parameter values using future, current
	 * and previous measurements
	 */
	void update(Stamped<Point> loc, double probability)
	{
		//float cov_meas = 0.05;
		//float cov_meas = 0.0025;
		meas_time_ = loc.stamp_;
		time_ = meas_time_;

		//R-Matrix
		SymmetricMatrix cov(3);
		cov = 0.0;
		cov(1,1) = cov_meas_people_m;
		cov(2,2) = cov_meas_people_m;
		cov(3,3) = cov_meas_people_m;

		filter_.updateCorrection(loc, cov);
		updatePosition();

	}
	/*
	 * time between prediction and correction:
	 * lifetime of a tracker
	 */
	double getLifetime()
	{
		return filter_.getLifetime();
	}

	double getReliability()
	{
		return reliability;
	}

private:
	/*
	 * estimates parameter values using current and
	 * previous measurements
	 */
	void updatePosition()
	{
		StatePosVel est;
		filter_.getEstimate(est);

		position_[0] = est.pos_[0];
		position_[1] = est.pos_[1];
		position_[2] = est.pos_[2];

		velocity_[0] = est.vel_[0];
		velocity_[1] = est.vel_[1];
		velocity_[2] = est.vel_[2];

		position_.stamp_ = time_;
		position_.frame_id_ = fixed_frame;
		velocity_.stamp_ = time_;
		velocity_.frame_id_ = fixed_frame;
	}
};

class SavedFeature
{
public:
	static int nextid;
	TransformListener& tfl_;

	BFL::StatePosVel sys_sigma_;
	TrackerKalman filter_;

	string id_;
	string object_id;
	ros::Time time_;
	ros::Time meas_time_;

	double reliability, p, probability;

	Stamped<Point> position_;
	Stamped<Vector3> velocity_;
	SavedFeature* other;
	float dist_to_person_;

	// one leg tracker
	SavedFeature(Stamped<Point> loc, TransformListener& tfl)
	: tfl_(tfl),
	  sys_sigma_(Vector3(0.05, 0.05, 0.05), Vector3(1.0, 1.0, 1.0)),
	  filter_("tracker_legs",sys_sigma_),
	  reliability(-1.), p(4)
	{
		char id[100];
		//snprintf(id,100,"legtrack %d", nextid++);
		id_ = std::string(id);

		object_id = "";
		time_ = loc.stamp_;
		meas_time_ = loc.stamp_;
		other = NULL;

		try {
			tfl_.transformPoint(fixed_frame, loc, loc);
		} catch(...) {
			ROS_WARN("TF exception spot 6.");
		}
		StampedTransform pose( Pose(Quaternion(0.0, 0.0, 0.0, 1.0), loc), loc.stamp_, id_, loc.frame_id_);
		tfl_.setTransform(pose);

		StatePosVel prior_sigma(Vector3(0.1,0.1,0.1), Vector3(0.0000001, 0.0000001, 0.0000001));
		filter_.initialize(loc, prior_sigma, time_.toSec());
		StatePosVel est;
		filter_.getEstimate(est);
		updatePosition();

	}

	void propagate(ros::Time time)
	{
		time_ = time;
		filter_.updatePrediction(time.toSec());
		updatePosition();
	}

	void update(Stamped<Point> loc, double probability)
	{
		StampedTransform pose( Pose(Quaternion(0.0, 0.0, 0.0, 1.0), loc), loc.stamp_, id_, loc.frame_id_);
		tfl_.setTransform(pose);

		meas_time_ = loc.stamp_;
		time_ = meas_time_;

		SymmetricMatrix cov(3);
		cov = 0.0;
		cov(1,1) = cov_meas_legs_m;
		cov(2,2) = cov_meas_legs_m;
		cov(3,3) = cov_meas_legs_m;

		filter_.updateCorrection(loc, cov);
		updatePosition();

		if(reliability<0 || !use_filter){
			reliability = probability;
			p = kal_p;
		}
		else{
			p += kal_q;
			double k = p / (p+kal_r);
			reliability += k * (probability - reliability);
			p *= (1 - k);
		}
	}

	double getLifetime()
	{
		return filter_.getLifetime();
	}

	double getReliability()
	{
		return reliability;
	}

private:
	void updatePosition()
	{
		StatePosVel est;
		filter_.getEstimate(est);

		position_[0] = est.pos_[0];
		position_[1] = est.pos_[1];
		position_[2] = est.pos_[2];

		velocity_[0] = est.vel_[0];
		velocity_[1] = est.vel_[1];
		velocity_[2] = est.vel_[2];

		position_.stamp_ = time_;
		position_.frame_id_ = fixed_frame;
		velocity_.stamp_ = time_;
		velocity_.frame_id_ = fixed_frame;

		double nreliability = fmin(1.0, fmax(0.1, est.vel_.length() / 0.5));
	}

};

int SavedFeature::nextid = 0;

class Legs
{
public:
	Stamped<Point> loc_;
	string type_;
	Legs(Stamped<Point> loc, string type)
	: loc_(loc)
	, type_(type)
	{};

};

class Pair
{
public:
	Point pos1_;
	Point pos2_;
	float dist_sep_m;
	Pair(Point pos1,Point pos2,float dist)
	:pos1_(pos1)
	,pos2_(pos2)
	,dist_sep_m(dist)
	{};
};

class MatchedFeature
{
public:
	SampleSet* candidate_;
    geometry_msgs::Pose m_pose;
	//Legs* candidate_;
	SavedFeature* closest_;
	float distance_;
	double probability_;

	MatchedFeature(SampleSet* candidate, SavedFeature* closest, float distance, double probability)
	: candidate_(candidate)
	, closest_(closest)
	, distance_(distance)
	, probability_(probability)
	{}
    
	MatchedFeature(geometry_msgs::Pose Pose_, SavedFeature* closest, float distance, double probability)
	: m_pose(Pose_)
	, closest_(closest)
	, distance_(distance)
	, probability_(probability)
	{}


	inline bool operator< (const MatchedFeature& b) const
	{
		return (distance_ <  b.distance_);
	}
};


int g_argc;
char** g_argv;
string scan_topic = "hsrb/base_scan";


// actual legdetector node
class LegDetector
{
 
 bool pauseSent;
 int counter;

 ros::ServiceClient client_map;  // clent for the getMap service
 nav_msgs::GetMap srv_map;
 nav_msgs::OccupancyGrid static_map;
 nav_msgs::OccupancyGrid dynamic_map;
        
 int ind_x, ind_y;

 short int map_data []; 

 double tmp;

 int indtmp;
  

public:
	NodeHandle nh_;
	TransformListener tfl_;
	ScanMask mask_;
	int mask_count_;
	// CvRTrees forest;
	cv::Ptr<cv::ml::RTrees> forest;
	float connected_thresh_;
	int feat_count_;
	char save_[100];
	list<SavedFeature*> saved_features_;
	//list<SavedFeature*> saved_features_;
	boost::mutex saved_mutex_;
	int feature_id_;

	bool use_seeds_;
    bool publish_legs_, publish_people_, publish_leg_markers_, publish_people_markers_, publish_vel_markers_;
	int next_p_id_;
	double leg_reliability_limit_;
	int min_points_per_group;

    ros::Publisher people_measurements_pub_;
	ros::Publisher leg_measurements_pub_;
	ros::Publisher people_pub_;
	ros::Publisher markers_pub_;
    ros::Subscriber edge_leg_sub;
    ros::Subscriber filter_leg_sub;
    ros::Subscriber op_human_sub;
    ros::Subscriber globalpose_sub;

    //TODO : dynamic_reconfigure
    //dynamic_reconfigure::Server<human_filter::LegDetectionConfig> server_;
    // topics

	message_filters::Subscriber<people_msgs::PositionMeasurementArray> people_sub_;
	message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
    //tf::MessageFilter<people_msgs::PositionMeasurement> people_notifier_;
	tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;

	list<SavedPersonFeature*>saved_people_;
	tf::TransformBroadcaster br_;
    // services
    //
    //ros::ServiceServer service_server_detect_legs_;
    //list<SavedPersonFeature*>saved_people_;

    // detections data for service_server_detect_legs_ service
    vector<geometry_msgs::Point32> detected_legs;
    vector<geometry_msgs::Point> edge_legs;
    vector<geometry_msgs::Point> filter_legs;
    vector<geometry_msgs::Point> op_poses;
    std::vector<double> global_pose;
    //geometry_msgs::PoseArray edge_legposes;
    geometry_msgs::PoseStamped temp_leg_transformed_pose;
    geometry_msgs::PoseStamped temp_loc_transformed_pose;
    geometry_msgs::PoseArray human_op_poses_array;
    tf::StampedTransform transform_sensor_base;

	LegDetector(ros::NodeHandle nh):
		nh_(nh),
		mask_count_(0),
		next_p_id_(0),
        //connected_thresh_(0.07),
        connected_thresh_(0.1),
        leg_reliability_limit_(-0.100),
		feat_count_(0),
        laser_sub_(nh_,scan_topic,10),
		laser_notifier_(laser_sub_,tfl_,fixed_frame,10)
	{
        if (g_argc > 1) {

            forest = cv::ml::RTrees::create();
            cv::String feature_file = cv::String(g_argv[1]);
            forest = cv::ml::StatModel::load<cv::ml::RTrees>(feature_file);
            feat_count_ = forest->getVarCount();
            printf("Loaded forest with %d features: %s\n", feat_count_, g_argv[1]);

        } else {
            printf("Please provide a trained random forests classifier as an input.\n");
            shutdown();
        }

        op_human_sub =nh_.subscribe<geometry_msgs::PoseArray>("/openpose_pose_array", 10, &LegDetector::openpose_pose_callback,this);
        leg_measurements_pub_ = nh_.advertise<people_msgs::PositionMeasurementArray>("leg_tracker_measurements",0);
        people_measurements_pub_ = nh_.advertise<people_msgs::PositionMeasurementArray>("people_tracker_measurements", 0);
        //people_pub_ = nh_.advertise<people_msgs::PositionMeasurementArray>("people",0);
        people_pub_ = nh_.advertise<people_msgs::People_v2>("people",0);
        markers_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 20);

        //laser_notifier_.
        laser_notifier_.registerCallback(boost::bind(&LegDetector::laserCallback, this, _1));
        laser_notifier_.setTolerance(ros::Duration(0.01));
        laser_notifier_.setTargetFrame(fixed_frame);
        
        people_sub_.subscribe(nh_,"people_tracker_measurements",10);
        people_sub_.registerCallback(boost::bind(&LegDetector::peopleCallback, this, _1));
        
        globalpose_sub=nh_.subscribe<geometry_msgs::PoseStamped>("/global_pose",10,&LegDetector::global_pose_callback,this);
        edge_leg_sub=nh_.subscribe<geometry_msgs::PoseArray>("/edge_leg_detector", 10, &LegDetector::edge_leg_callback,this);
        //filter_leg_sub=nh_.subscribe<geometry_msgs::PoseArray>("/openpose_filter_pose_array", 10, &LegDetector::filter_leg_callback,this);
        //people_sub_.subscribe(nh_,"people_tracker_measurements",10);
        //people_sub_.registerCallback(boost::bind(&LegDetector::peopleCallback, this, _1));
        
        //people_sub_ = nh_.subscribe<people_msgs::PositionMeasurementArray>("/people_tracker_measurements", 30, &LegDetector::peopleCallback,this);
        // /Todo
        //dynamic_reconfigure::Server<human_filter::LegDetectionConfig>::CallbackType f;
        //f = boost::bind(&LegDetector::configure, this, _1, _2);
        //server_.setCallback(f);
        

        publish_legs_           = true;
        publish_people_         = true;
        //publish_leg_markers_    = false;
        //publish_vel_markers_    = false;
        //publish_people_markers_ = false; 
        publish_leg_markers_    = true;
        publish_vel_markers_    = true;
        publish_people_markers_ = true; 

        global_pose.resize(2,0.0);
		feature_id_ = 0;

        //map_msgs
        nav_msgs::OccupancyGrid::ConstPtr dyn_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/dynamic_obstacle_map_ref");
        nav_msgs::OccupancyGrid::ConstPtr sta_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/static_obstacle_map_ref");
        dynamic_map = *(dyn_msg);
        static_map = *(sta_msg);
        //client_map = nh_.serviceClient<nav_msgs::GetMap>("/dynamic_obstacle_map_ref"); // geting the clent for the map ready
                       
                if (dynamic_map.data.size()>0) {  // call to srv_map OK
                  printf ("The cells in the ocupancy grid with size %d are:\n", dynamic_map.data.size());
                  printf (" width %d are:\n", dynamic_map.info.width);
                  printf (" height %d are:\n", dynamic_map.info.height);
                  printf (" resolution %f is:\n", dynamic_map.info.resolution);
                  geometry_msgs::Pose pose = dynamic_map.info.origin;
                  printf (" x position is %f \n", pose.position.x);
                  printf (" y position is %f \n", pose.position.y);
                  printf (" z position is %f \n", pose.position.z);
                  printf (" x orientation is %f \n", pose.orientation.x);
                  printf (" y orientation is %f \n", pose.orientation.y);
                  printf (" z orientation is %f \n", pose.orientation.z);
                  printf (" w orientation is %f \n", pose.orientation.w);

                  int map_width =  dynamic_map.info.width;
                  int map_height = dynamic_map.info.height;
                } 
                else
            {
           ROS_ERROR("Failed to call service GetMap");
            }
       }

    ~LegDetector()
    {
    }

    //void configure(human_filter::LegDetectionConfig &config, uint32_t level)
    //{
        //connected_thresh_       = config.connection_threshold;
        //min_points_per_group    = config.min_points_per_group;
        //leg_reliability_limit_  = config.leg_reliability_limit;
        //publish_legs_           = config.publish_legs;
        //publish_people_         = config.publish_people;
        //publish_leg_markers_    = config.publish_leg_markers;
        //publish_vel_markers_    = config.publish_vel_markers;
        //publish_people_markers_ = config.publish_people_markers;

        //no_observation_timeout_s = config.no_observation_timeout;
        //max_second_leg_age_s     = config.max_second_leg_age;
        //max_track_jump_m         = config.max_track_jump;
        //max_meas_jump_m          = config.max_meas_jump;
        //leg_pair_separation_m    = config.leg_pair_separation;
        //cov_meas_legs_m			 = config.cov_meas_legs;
        //cov_meas_people_m        = config.cov_meas_people;

        //if(fixed_frame.compare(config.fixed_frame) != 0){
            //fixed_frame              = config.fixed_frame;
            //laser_notifier_.setTargetFrame(fixed_frame);
             //people_notifier_.setTargetFrame(fixed_frame);
        //}
        //kal_p                    = config.kalman_p;
        //kal_q                    = config.kalman_q;
        //kal_r                    = config.kalman_r;
    //    //use_filter               = config.kalman_on == 1;
        //use_filter               = 1;
    //}


//mk
//
//IS_NEAR_OPhuman
    bool IS_NEAR_OPhuman(double pos_x,double  pos_y)
    {
        double  seperation_treshold=2.0;
        bool IsClose = false;
        double dist =0.0;

        if(op_poses.size()>0){
            //should compare cluster and poses_array
            for(size_t idx (0); idx<op_poses.size();idx++ )
            {

                float x_diff = abs(op_poses[idx].x-pos_x);
                float y_diff = abs(op_poses[idx].y-pos_y);
                float dist = sqrt(pow(x_diff,2)+pow(y_diff,2));

                ROS_INFO("distance to target %.3lf",dist);

                if(dist < seperation_treshold) 
                    IsClose = true;

            }
        }

        return IsClose;
    }
    bool IS_NEAR_legs(double pos_x,double  pos_y)
    {
        double  seperation_treshold=1.00;
        bool IsClose = false;
        double dist =0.0;

        if(filter_legs.size()>0){
            //should compare cluster and poses_array
            for(size_t idx (0); idx<filter_legs.size();idx++ )
            {

                float x_diff = abs(filter_legs[idx].x-pos_x);
                float y_diff = abs(filter_legs[idx].y-pos_y);
                float dist = sqrt(pow(x_diff,2)+pow(y_diff,2));
                ROS_INFO("distance to target %.3lf",dist);

                if(dist < seperation_treshold) 
                    IsClose = true;

            }
        }

        return IsClose;
    }
	void peopleCallback(const people_msgs::PositionMeasurementArray::ConstPtr& people_meas)
	{
        //ROS_INFO("start people callback");
		if (people_meas->people.empty())
			return;

		boost::mutex::scoped_lock lock(saved_mutex_);
		list<SavedPersonFeature*>::iterator it;
		people_msgs::PositionMeasurement ppl;
		list<SavedPersonFeature*> saved_people;

		//if there are some people in memory list
		if(saved_people_.size() != 0){
			//predict distribution of error measurement values over the next time
			// by a known correct state from the previous time ->
			for(list<SavedPersonFeature*>::iterator iter_ = saved_people_.begin();
					iter_ != saved_people_.end(); ++iter_){
				(*iter_)->propagate(people_meas->header.stamp);
			}
		}

		//extracts the data from the comming message and saves new people in the list
		for(int i = 0; i < people_meas->people.size(); i++){
			ppl = people_meas->people.at(i);
			Stamped<Point> person_loc(Vector3(ppl.pos.x, ppl.pos.y, ppl.pos.z), people_meas->header.stamp, ppl.header.frame_id);
			//if the list is empty, add the person to the list
			if(saved_people.size() == 0){
				saved_people.insert(saved_people.end(), new SavedPersonFeature(person_loc, ppl.object_id, ppl.name));
			}else{
				bool found = false;
				//if the list is not empty, check for a person name in the list
				for (it = saved_people.begin(); it != saved_people.end(); ++it){
					if(ppl.name.compare((*it)->person_name) == 0 )
						found = true;
				}
				//if there is no same name in the list, add the person to the list
				if(!found){
					found = false;
					saved_people.insert(saved_people.end(), new SavedPersonFeature(person_loc, ppl.object_id, ppl.name));
				}
			}
		}

		// Compare two lists:
		// saved_people_ - global list over the time with current detected people
		// saved_people  - temporary list within this message with the current detected people

		//if the memory list of people is empty, put all new people on the list
		// same people are excluded
		if(saved_people_.size() == 0){
			for(list<SavedPersonFeature*>::iterator iter = saved_people.begin(); iter != saved_people.end(); ++iter){
				saved_people_.push_back((*iter));
			}
		}else{ // if the memory list is not empty, check the list for the same people to update there values
			for(list<SavedPersonFeature*>::iterator iter = saved_people.begin(); iter != saved_people.end(); ++iter)
			{
				bool found_temp = false;
				for(list<SavedPersonFeature*>::iterator iter_ = saved_people_.begin(); iter_ != saved_people_.end(); ++iter_){
					if((*iter)->person_name.compare((*iter_)->person_name) == 0){
						found_temp = true;
						// update distribution over current state of values
						//by known prediction of state and next measurements:
						(*iter_)->update((*iter)->position_, 1.0);
					}
				}

				if(found_temp == false ){
					saved_people_.push_back((*iter));
				}
			}
		}

		//erase unnecessary tracks on people if they are called the same
		for(list<SavedPersonFeature*>::iterator iter = saved_people_.begin(); iter != saved_people_.end(); ++iter){
			bool found_temp = false;
			for(list<SavedPersonFeature*>::iterator iter_ = saved_people.begin(); iter_ != saved_people.end(); ++iter_){
				if((*iter)->person_name.compare((*iter_)->person_name) == 0){
					found_temp = true;
				}
			}
			if(found_temp == false ){
				delete (*iter);
				saved_people_.erase(iter++);
			}
		}


		//PRINT_LIST(saved_people_, "LIST saved_people ::  ");
		//publish data

		int i = 0;

		vector<people_msgs::Person_v2> people;

		for (list<SavedPersonFeature*>::iterator sp_iter = saved_people_.begin();
				sp_iter != saved_people_.end(); sp_iter++,i++){
			//ROS_INFO("Velocity [%f, %f, %f]}: ", (*sp_iter)->velocity_[0], (*sp_iter)->velocity_[1], (*sp_iter)->velocity_[2]);
			people_msgs::Person_v2 person;
			person.detector = detector_;
			person.name = (*sp_iter)->person_name; // name of the person
			person.position.position.x = (*sp_iter)->position_[0];
			person.position.position.y = (*sp_iter)->position_[1];
			person.position.position.z = (*sp_iter)->position_[2];

			person.velocity.x = (*sp_iter)->velocity_[0];
			person.velocity.y = (*sp_iter)->velocity_[1];
			person.velocity.z = (*sp_iter)->velocity_[2];

			people.push_back(person);

			double dx = (*sp_iter)->velocity_[0], dy = (*sp_iter)->velocity_[1];
			visualization_msgs::Marker m;
			m.header.stamp = people_meas->header.stamp;
			m.header.frame_id = fixed_frame;
			m.ns = "SPEED";
			m.type = m.ARROW;
			m.pose.position.x = (*sp_iter)->position_[0];
			m.pose.position.y = (*sp_iter)->position_[1];
			m.pose.position.z = (*sp_iter)->position_[2];
			m.pose.orientation.x = (*sp_iter)->velocity_[0];
			m.pose.orientation.y = (*sp_iter)->velocity_[1];
			m.pose.orientation.z = 0.0;
			m.scale.x = sqrt(dx*dx+dy*dy);
			//ROS_INFO("speed %f", m.scale.x);
			//m.scale.x = .4;
			m.scale.y = .05;
			m.scale.z = .05;
			m.color.a = 1;
			m.color.r = 1;
			m.lifetime = ros::Duration(0.5);

			markers_pub_.publish(m);
		}
		//people_msgs::PositionMeasurementArray array;
		people_msgs::People_v2 array;
		array.header.stamp = ros::Time::now();
		array.people = people;
		people_pub_.publish(array);
	}

	 //void edge_leg_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
    void global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {

        global_pose[0]=msg->pose.position.x;
        global_pose[1]=msg->pose.position.y;

    }


    void filter_leg_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
     {

         filter_legs.clear();
         int num_leg_detected = msg->poses.size(); 
         for(int i(0);i<num_leg_detected;i++){

             geometry_msgs::Point tmp_point;
             tmp_point.x = msg->poses[i].position.x;
             tmp_point.y = msg->poses[i].position.y;
             tmp_point.z=0.0;

             filter_legs.push_back(tmp_point);

         }
         //printf("size edge_leg : %d \n", static_cast<int>(filter_legs.size()));
         //tfl_.waitForTransform(LASER_FRAME,GLOBAL_FRAME,  ros::Time(0), ros::Duration(2.0));
     }

    void openpose_pose_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
    {
        //openpose comes w.r.t map frame
        ROS_INFO("openposes callback: poses size : %d ", msg->poses.size());
        human_op_poses_array = *msg;

        op_poses.clear();
        int num_leg_detected = msg->poses.size(); 
         for(int i(0);i<num_leg_detected;i++){
             geometry_msgs::Point tmp_point;
             tmp_point.x = msg->poses[i].position.x;
             tmp_point.y = msg->poses[i].position.y;
             tmp_point.z=0.0;
             op_poses.push_back(tmp_point);
         }

    }

    void edge_leg_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
    {

        filter_legs.clear();
        int num_leg_detected = msg->poses.size(); 
         std::vector<double> tempVec(2,0.0);

         for(int i(0);i<num_leg_detected;i++)
         {
             geometry_msgs::Vector3Stamped gV, tV;

             gV.vector.x = msg->poses[i].position.x;
             gV.vector.y = msg->poses[i].position.y;
             gV.vector.z = 1.0;

             gV.header.stamp = ros::Time();
             gV.header.frame_id = "/base_range_sensor_link";
             tfl_.transformVector("/map", gV, tV);

             geometry_msgs::Point tmp_point;
             tmp_point.x = tV.vector.x+global_pose[0];
             tmp_point.y = tV.vector.y+global_pose[1];
             tmp_point.z=0.0;

             //check with open_pose_array
             bool IsNear_Openposes = IS_NEAR_OPhuman(tmp_point.x,tmp_point.y);
             if(IsNear_Openposes )
             { 
                 filter_legs.push_back(tmp_point);
             }
             else{

                 ROS_INFO("passed");
             }

         }

         //tf::StampedTransform transform_sensor;
         //tfl_.waitForTransform(LASER_FRAME,GLOBAL_FRAME,  ros::Time(0), ros::Duration(1.0));
         //edge_legs.clear();

         //int num_leg_detected = msg->poses.size(); 
         //for(int i(0);i<num_leg_detected;i++){

             //geometry_msgs::PoseStamped temp_leg_pose;
             //temp_leg_pose.pose = msg->poses[i];
             //temp_leg_pose.header.stamp=ros::Time::now();
             //temp_leg_pose.header.frame_id=LASER_FRAME;

             //try{

                 //tfl_.lookupTransform(LASER_FRAME,GLOBAL_FRAME, ros::Time(1.0), transform_sensor);
                 //tfl_.transformPose(GLOBAL_FRAME, temp_leg_pose,  temp_leg_transformed_pose);
                 //temp_leg_transformed_pose.header.stamp=ros::Time::now();
                 //temp_leg_transformed_pose.header.frame_id=GLOBAL_FRAME;
             //} catch(...) {
                 //ROS_WARN("TF exception spot 1.");
             //}

             //geometry_msgs::Point tmp_point;
             //tmp_point.x = temp_leg_transformed_pose.pose.position.x;
             //tmp_point.y = temp_leg_transformed_pose.pose.position.y;
             //tmp_point.z=0.0;

             //edge_legs.push_back(tmp_point);

         //}
         //printf("size edge_leg : %d \n", static_cast<int>(edge_legs.size()));
         //tfl_.waitForTransform(LASER_FRAME,GLOBAL_FRAME,  ros::Time(0), ros::Duration(2.0));
     }
    
//original callback using transform point
     //void edge_leg_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
     //{

       //  //ROS_INFO("leg_callback";
       //  //edge_legposes =(*msg);
         
         //edge_legs.clear();
         //int num_leg_detected = msg->poses.size(); 
         //for(int i(0);i<num_leg_detected;i++){

         //Stamped<Point> leg_candidate(tf::Point(msg->poses[i].position.x,msg->poses[i].position.y,
         //0.0), ros::Time(), laser_frame);
         //Stamped<Point> temp_point;
         //try {
         //tfl_.transformPoint(fixed_frame, leg_candidate, temp_point);
         //ROS_INFO("original x, y : %.3lf, %.3lf", leg_candidate.x(),leg_candidate.y());
         //ROS_INFO("transform x, y : %.3lf, %.3lf", leg_candidate.x(),leg_candidate.y());
         //} catch(...) {
         //ROS_WARN("TF exception spot 1.");
         //}

         //geometry_msgs::Point tmp_point;
         //tmp_point.x=static_cast<double>(leg_candidate.x());
         //tmp_point.y=static_cast<double>(leg_candidate.y());
         //tmp_point.z=0.0;

         //edge_legs.push_back(tmp_point);

         //}
         //printf("size edge_leg : %d \n", static_cast<int>(edge_legs.size()));

     //}

	 void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
	{ 
        std::cout<< scan->header.frame_id<<std::endl;
        //geometry_msgs::Point32 pt_temp; // used in building the detected_legs vector
        //detected_legs.clear(); //to be ready for the new detections
        //
        //
        ScanProcessor processor(*scan, mask_);
        processor.splitConnected(connected_thresh_);
        processor.removeLessThan(5);
        //processor.filterwithPosesarray(edge_legs);
        //processor.filterwithPosesarray(filter_legs);
        //processor.filterwithPosesarray(op_poses);

        // OpenCV 3
        cv::Mat tmp_mat = cv::Mat(1, feat_count_, CV_32FC1);

		// if no measurement matches to a tracker in the last <no_observation_timeout>  seconds: erase tracker
		ros::Time purge = scan->header.stamp + ros::Duration().fromSec(-no_observation_timeout_s);
		list<SavedFeature*>::iterator sf_iter = saved_features_.begin();
		while (sf_iter != saved_features_.end()){
			if ((*sf_iter)->meas_time_ < purge){
				if( (*sf_iter)->other )
					(*sf_iter)->other->other = NULL;
				delete (*sf_iter);
				saved_features_.erase(sf_iter++);
			}else
				++sf_iter;
		}

        // System update of trackers, and copy updated ones in propagate list
        list<SavedFeature*> propagated;
        for (list<SavedFeature*>::iterator sf_iter = saved_features_.begin();
                sf_iter != saved_features_.end();
                sf_iter++){
            (*sf_iter)->propagate(scan->header.stamp);
            propagated.push_back(*sf_iter);
        }

        // Detection step: build up the set of "candidate" clusters
        // For each candidate, find the closest tracker (within threshold) and add to the match list
        // If no tracker is found, start a new one
        multiset<MatchedFeature> matches;
        for (list<SampleSet*>::iterator i = processor.getClusters().begin();
                i != processor.getClusters().end();i++){
            vector<float> f = calcLegFeatures(*i, *scan);

            for (int k = 0; k < feat_count_; k++)
                tmp_mat.data[k] = (float)(f[k]);

            // OpenCV 3
            // Probability is the fuzzy measure of the probability that the second element should be chosen,
            // in opencv2 RTrees had a method predict_prob, but that disapeared in opencv3, this is the
            // substitute.
            float probability = 0.5 -
                forest->predict(tmp_mat, cv::noArray(), cv::ml::RTrees::PREDICT_SUM) /
                forest->getRoots().size();
            
            //printf("probablity: %f",probability);
            
            //tf::StampedTransform transform_sensor_base;
            //********************************************
            //geometry_msgs::PoseStamped loc_pose;
            //loc_pose.header=scan->header;
            //loc_pose.header.stamp=ros::Time::now();
            //loc_pose.pose.position.x =(*i)->center().x();
            //loc_pose.pose.position.y =(*i)->center().y();
            //loc_pose.pose.position.z =(*i)->center().z();
            //loc_pose.pose.orientation.x =0.0;
            //loc_pose.pose.orientation.y =0.0;
            //loc_pose.pose.orientation.z =0.0;
            //loc_pose.pose.orientation.w =1.0;

            tfl_.waitForTransform(LASER_FRAME,GLOBAL_FRAME,  ros::Time(0), ros::Duration(1.0));
            tfl_.lookupTransform(laser_frame,GLOBAL_FRAME, ros::Time(0), transform_sensor_base);
           // ********************************************

            //printf("probablity: %f",probability);
            Stamped<Point> loc((*i)->center(), scan->header.stamp, scan->header.frame_id);
            try {
            //********************************************
                //temp_loc_transformed_pose.header.frame_id=GLOBAL_FRAME;
                //tfl_.transformPose (GLOBAL_FRAME, loc_pose,  temp_loc_transformed_pose) ;
                //temp_loc_transformed_pose.header.stamp=ros::Time::now();
                //temp_loc_transformed_pose.header.frame_id=GLOBAL_FRAME;
            //********************************************
              tfl_.transformPoint(fixed_frame, loc,loc);
            } catch(...) {
                ROS_WARN("TF exception spot 3.");
            }

            //Stamped<Point> loc(tf::Point(temp_loc_transformed_pose.pose.position.x,
                                                 //temp_loc_transformed_pose.pose.position.y,
                                                 //temp_loc_transformed_pose.pose.position.z),
                                                 //ros::Time(0), GLOBAL_FRAME);
 
            list<SavedFeature*>::iterator closest = propagated.end();
            float closest_dist = max_track_jump_m;

            for (list<SavedFeature*>::iterator pf_iter = propagated.begin();
                    pf_iter != propagated.end();pf_iter++){
                // find the closest distance between candidate and trackers
                float dist = loc.distance((*pf_iter)->position_);
            //********************************************
                //float dist = getdistance(loc_pose.pose,(*pf_iter)->position_);
            //********************************************
                if ( dist < closest_dist ){
                    closest = pf_iter;
                    closest_dist = dist;
                }
            }
            // Nothing close to it, start a new track
            if (closest == propagated.end()){
                list<SavedFeature*>::iterator new_saved = saved_features_.insert(saved_features_.end(), new SavedFeature(loc, tfl_));
                //printf("add new tracker\n");
            }else // Add the candidate, the tracker and the distance to a match list
                matches.insert(MatchedFeature(*i,*closest,closest_dist,probability));

            //printf("closest tracker found\n");
        }


        // loop through _sorted_ matches list
        // find the match with the shortest distance for each tracker
        while (matches.size() > 0){
            multiset<MatchedFeature>::iterator matched_iter = matches.begin();
            bool found = false;
            list<SavedFeature*>::iterator pf_iter = propagated.begin();
            while (pf_iter != propagated.end()){
                // update the tracker with this candidate
                if (matched_iter->closest_ == *pf_iter){
                    // Transform candidate to fixed frame
                    //
                    //********************************************
                    //geometry_msgs::PoseStamped loc_pose;
                    //loc_pose.header=scan->header;
                    //loc_pose.header.frame_id=LASER_FRAME;
                    //loc_pose.header.stamp=ros::Time::now();
                    //loc_pose.pose.position.x =matched_iter->candidate_->center().x();
                    //loc_pose.pose.position.y =matched_iter->candidate_->center().y();
                    //loc_pose.pose.position.z =matched_iter->candidate_->center().z();
                    //loc_pose.pose.orientation.x =0.0;
                    //loc_pose.pose.orientation.y =0.0;
                    //loc_pose.pose.orientation.z =0.0;
                    //loc_pose.pose.orientation.w =1.0;
                    //********************************************

                    tfl_.lookupTransform(laser_frame,GLOBAL_FRAME, ros::Time(0), transform_sensor_base);

                    Stamped<Point> loc(matched_iter->candidate_->center(), scan->header.stamp, scan->header.frame_id);

                    //Stamped<Point> tmp_loc;
                    try {
                        tfl_.transformPoint(fixed_frame, loc, loc);
                        //********************************************
                        //temp_loc_transformed_pose.header.frame_id=GLOBAL_FRAME;
                        //tfl_.lookupTransform(laser_frame,GLOBAL_FRAME, ros::Time(0), transform_sensor_base);
                        //tfl_.transformPose (GLOBAL_FRAME, loc_pose,  temp_loc_transformed_pose) ;
                        //temp_loc_transformed_pose.header.stamp=ros::Time::now();
                        //temp_loc_transformed_pose.header.frame_id=GLOBAL_FRAME;
                        //********************************************
                    } catch(...) {
                        ROS_WARN("TF exception spot 4.");
                    }

                    //********************************************
                    //Stamped<Point> loc(tf::Point(temp_loc_transformed_pose.pose.position.x,
                                                 //temp_loc_transformed_pose.pose.position.y,
                                                 //temp_loc_transformed_pose.pose.position.z),
                                                 //ros::Time(0), GLOBAL_FRAME);
                    //********************************************
                    //
                    // Update the tracker with the candidate location
                    matched_iter->closest_->update(loc, matched_iter->probability_);

                    //printf("upadatedx closest tracker\n");
                    // remove this match and
                    matches.erase(matched_iter);
                    propagated.erase(pf_iter++);
                    found = true;
                    break;
                }else{  // still looking for the tracker to update
                    pf_iter++;
                }
            }

            // didn't find tracker to update, because it was deleted above
            // try to assign the candidate to another tracker
            if (!found){
                //printf("didn't find tracker\n");
                //
                //********************************************
                //geometry_msgs::PoseStamped loc_pose;
                //loc_pose.header=scan->header;
                //loc_pose.header.frame_id=LASER_FRAME;
                //loc_pose.header.stamp=ros::Time::now();
                //loc_pose.pose.position.x =matched_iter->candidate_->center().x();
                //loc_pose.pose.position.y =matched_iter->candidate_->center().y();
                //loc_pose.pose.position.z =matched_iter->candidate_->center().z();
                //loc_pose.pose.orientation.x =0.0;
                //loc_pose.pose.orientation.y =0.0;
                //loc_pose.pose.orientation.z =0.0;
                //loc_pose.pose.orientation.w =1.0;
                //********************************************

                tfl_.lookupTransform(laser_frame,GLOBAL_FRAME, ros::Time(0), transform_sensor_base);
                Stamped<Point> loc(matched_iter->candidate_->center(), scan->header.stamp, scan->header.frame_id);

                //Stamped<Point> loc(matched_iter->candidate_->center(), scan->header.stamp, scan->header.frame_id);
                try {
                    tfl_.transformPoint(fixed_frame, loc, loc);
                    //********************************************
                    //temp_loc_transformed_pose.header.frame_id=GLOBAL_FRAME;
                    //tfl_.lookupTransform(laser_frame,GLOBAL_FRAME, ros::Time(0), transform_sensor_base);
                    //tfl_.transformPose (GLOBAL_FRAME, loc_pose,  temp_loc_transformed_pose) ;
                    //temp_loc_transformed_pose.header.stamp=ros::Time::now();
                    //temp_loc_transformed_pose.header.frame_id=GLOBAL_FRAME;
                    //********************************************
                } catch(...) {
                    ROS_WARN("TF exception spot 5.");
                }

                //********************************************
                //Stamped<Point> loc(tf::Point(temp_loc_transformed_pose.pose.position.x,
                                                 //temp_loc_transformed_pose.pose.position.y,
                                                 //temp_loc_transformed_pose.pose.position.z),
                                                 //ros::Time(0), GLOBAL_FRAME);
                //********************************************

                list<SavedFeature*>::iterator closest = propagated.end();
                float closest_dist = max_track_jump_m;

                for (list<SavedFeature*>::iterator remain_iter = propagated.begin();
                        remain_iter != propagated.end();remain_iter++){
                    float dist = loc.distance((*remain_iter)->position_);
                    if ( dist < closest_dist ){
                        closest = remain_iter;
                        closest_dist = dist;
                    }
                }

                // no tracker is within a threshold of this candidate
                // so create a new tracker for this candidate
                if (closest == propagated.end())
                    list<SavedFeature*>::iterator new_saved = saved_features_.insert(saved_features_.end(), new SavedFeature(loc, tfl_));
                else
                    matches.insert(MatchedFeature(matched_iter->candidate_,*closest,closest_dist, matched_iter->probability_));
                matches.erase(matched_iter);
            }
        }

        // if(!use_seeds_)
        pairLegs();

        // Publish Data!
        int i = 0;
        vector<people_msgs::PositionMeasurement> people;
        vector<people_msgs::PositionMeasurement> legs;

        //printf("size of savedFeature:%d", saved_features_.size());
        for (list<SavedFeature*>::iterator sf_iter = saved_features_.begin();
                sf_iter != saved_features_.end(); sf_iter++,i++){
            // reliability
            double reliability = (*sf_iter)->getReliability();
            //  ROS_INFO("reliability %f", reliability);
            double people_pos_x = (*sf_iter)->position_[0];
            double people_pos_y = (*sf_iter)->position_[1];

            if(!IS_NEAR_OPhuman(people_pos_x,people_pos_y))
                continue;


            if ((*sf_iter)->getReliability() > leg_reliability_limit_
                    && publish_legs_){

                people_msgs::PositionMeasurement pos;
                pos.header.stamp = scan->header.stamp;
                pos.header.frame_id = fixed_frame;
                pos.name = "leg_detection";
                pos.object_id = (*sf_iter)->id_;
                pos.pos.x = (*sf_iter)->position_[0];
                pos.pos.y = (*sf_iter)->position_[1];
                pos.pos.z = (*sf_iter)->position_[2];
                pos.vel.x = (*sf_iter)->velocity_[0];
                pos.vel.y = (*sf_iter)->velocity_[1];
                pos.vel.z = (*sf_iter)->velocity_[2];
                pos.reliability = reliability;
                pos.covariance[0] = pow(0.3 / reliability,2.0);
                pos.covariance[1] = 0.0;
                pos.covariance[2] = 0.0;
                pos.covariance[3] = 0.0;
                pos.covariance[4] = pow(0.3 / reliability,2.0);
                pos.covariance[5] = 0.0;
                pos.covariance[6] = 0.0;
                pos.covariance[7] = 0.0;
                pos.covariance[8] = 10000.0;
                pos.initialization = 0;
                legs.push_back(pos);
            }

            if (publish_leg_markers_){
                visualization_msgs::Marker m;
                m.header.stamp = (*sf_iter)->time_;
                m.header.frame_id = fixed_frame;
                m.ns = "LEGS";
                m.id = i;
                m.type = m.SPHERE;
                m.pose.position.x = (*sf_iter)->position_[0];
                m.pose.position.y = (*sf_iter)->position_[1];
                m.pose.position.z = (*sf_iter)->position_[2];

                m.scale.x = .1;
                m.scale.y = .1;
                m.scale.z = .1;
                m.color.a = 1;
                m.lifetime = ros::Duration(0.5);
                if((*sf_iter)->object_id != ""){
                    m.color.r = 1;
                }else{
                    m.color.b = (*sf_iter)->getReliability();
                }
                markers_pub_.publish(m);
            }

            if (publish_people_ || publish_people_markers_ ){
                SavedFeature* other = (*sf_iter)->other;
                if(other!=NULL && other<(*sf_iter)){

                    Stamped<Point> one = (*sf_iter)->position_, two = (other)->position_;
                    double ddx = one[0]-two[0], ddy = one[1]-two[1], ddz = one[2]-two[2];
                    double d =  sqrt(ddx*ddx + ddy*ddy + ddz*ddz);
                    //ROS_INFO("Person %s with distance %f",  (*sf_iter)->object_id.c_str() , d);

                    double dx = ((*sf_iter)->position_[0] + other->position_[0])/2,
                           dy = ((*sf_iter)->position_[1] + other->position_[1])/2,
                           dz = ((*sf_iter)->position_[2] + other->position_[2])/2;

                    double vx = ((*sf_iter)->velocity_[0]- ((*sf_iter)->velocity_[0] - other->velocity_[0])/2),
                           vy = ((*sf_iter)->velocity_[1]- ((*sf_iter)->velocity_[1] - other->velocity_[1])/2),
                           vz = ((*sf_iter)->velocity_[2]- ((*sf_iter)->velocity_[2] - other->velocity_[2])/2);

                    double speed = sqrt(vx*vx + vy*vy + vz*vz);
                    //ROS_INFO("speed %f: ", speed );

                    if (publish_people_ ){

                        reliability = reliability * other->reliability;
                        people_msgs::PositionMeasurement pos;
                        pos.header.stamp = (*sf_iter)->time_;
                        pos.header.frame_id = fixed_frame;
                        pos.name = (*sf_iter)->object_id;
                        pos.object_id = (*sf_iter)->id_ + "|" + other->id_;
                        pos.pos.x = dx;
                        pos.pos.y = dy;
                        pos.pos.z = dz;
                        pos.vel.x = vx;
                        pos.vel.y = vy;
                        pos.vel.z = vz;
                        pos.reliability = reliability;
                        pos.covariance[0] = pow(0.3 / reliability,2.0);
                        pos.covariance[1] = 0.0;
                        pos.covariance[2] = 0.0;
                        pos.covariance[3] = 0.0;
                        pos.covariance[4] = pow(0.3 / reliability,2.0);
                        pos.covariance[5] = 0.0;
                        pos.covariance[6] = 0.0;
                        pos.covariance[7] = 0.0;
                        pos.covariance[8] = 10000.0;
                        pos.initialization = 0;

                        people.push_back(pos);

                        ros::Time time = ros::Time::now();
                        tf::Transform person(tf::Quaternion(0,0,0,1), tf::Vector3(dx, dy, dz));
                        try
                        {
                            br_.sendTransform(tf::StampedTransform(person, time,
                                        "/map" , pos.name.c_str()));
                                        //"/base_link" , pos.name.c_str()));
                        }catch (tf::TransformException ex){
                            ROS_ERROR("Broadcaster unavailable %s", ex.what());
                        }
                    }

                    if (publish_people_markers_ ){
                        visualization_msgs::Marker m;
                        m.header.stamp = (*sf_iter)->time_;
                        m.header.frame_id = fixed_frame;
                        m.ns = "PEOPLE";
                        m.id = i;
                        m.type = m.SPHERE;
                        m.pose.position.x = dx;
                        m.pose.position.y = dy;
                        m.pose.position.z = dz;
                        m.scale.x = .2;
                        m.scale.y = .2;
                        m.scale.z = .2;
                        m.color.a = 1;
                        m.color.g = 1;
                        m.lifetime = ros::Duration(0.5);
                        markers_pub_.publish(m);
                    }

                    if(publish_people_markers_ ){
                        visualization_msgs::Marker m;
                        m.header.stamp = (*sf_iter)->time_;
                        m.header.frame_id = fixed_frame;
                        m.ns = "TEXT";
                        m.id = i;
                        m.type = m.TEXT_VIEW_FACING;
                        m.pose.position.x = dx;
                        m.pose.position.y = dy;
                        m.pose.position.z = dz;
                        m.pose.orientation.w = 1.0;
                        m.text = (*sf_iter)->object_id.c_str();
                        m.scale.z = 0.3;
                        m.color.a = 1;
                        m.color.r = 0.5;
                        m.color.g = 0.5;
                        m.lifetime = ros::Duration(0.5);
                        markers_pub_.publish(m);
                    }
                    /*
                       if (publish_people_markers_ ){
                       visualization_msgs::Marker m;
                       m.header.stamp = (*sf_iter)->time_;
                       m.header.frame_id = fixed_frame;
                       m.ns = "SPEED";
                       m.id = i;
                       m.type = m.ARROW;
                       m.pose.position.x = dx;
                       m.pose.position.y = dy;
                       m.pose.position.z = dz;
                       m.pose.orientation.x = vx;
                       m.pose.orientation.y = vy;
                       m.pose.orientation.z = vz;

                       m.scale.x = .4;
                       m.scale.y = .05;
                       m.scale.z = .05;
                       m.color.a = 1;
                       m.color.r = 1;
                       m.lifetime = ros::Duration(0.5);

                       markers_pub_.publish(m);
                       }
                       */
                }
            }
        }
        people_msgs::PositionMeasurementArray array;
        array.header.stamp = ros::Time::now();
        array.header.frame_id = fixed_frame;

        if(publish_legs_){
            array.people = legs;
            leg_measurements_pub_.publish(array);
        }

        if(publish_people_){
            array.people = people;
            people_measurements_pub_.publish(array);
        }
    }



    double distance( list<SavedFeature*>::iterator it1,  list<SavedFeature*>::iterator it2)
    {
        Stamped<Point> one = (*it1)->position_, two = (*it2)->position_;
        double dx = one[0]-two[0], dy = one[1]-two[1], dz = one[2]-two[2];
        return sqrt(dx*dx+dy*dy+dz*dz);
    }

    double getdistance(geometry_msgs::Pose pose1 , Stamped<Point> pose2)
    {
        tf::Point one(pose1.position.x,pose1.position.y,pose1.position.z);
        Stamped<Point> two = pose2;
        double dx = one[0]-two[0], dy = one[1]-two[1], dz = one[2]-two[2];
        return sqrt(dx*dx+dy*dy+dz*dz);
    }


    inline void PRINT_LIST	(list<SavedPersonFeature*>& coll, const char* optcstr="")
    {
        std::cout << optcstr;
        for (	list<SavedPersonFeature*>::iterator it =coll.begin();it !=coll.end(); ++it) {
            std::cout << (*it)->person_name << ' ';
        }
        std::cout << std::endl;
    }

    void pairLegs()
    {
        // Deal With legs that already have ids
        list<SavedFeature*>::iterator begin = saved_features_.begin();
        list<SavedFeature*>::iterator end = saved_features_.end();
        list<SavedFeature*>::iterator leg1, leg2, best, it;

        for (leg1 = begin; leg1 != end; ++leg1){
            // If this leg has no id, skip
            if ((*leg1)->object_id == "")
                continue;

            leg2 = end;
            best = end;
            // ROS_INFO("leg pair separation %f", leg_pair_separation_m);

            double closest_dist = leg_pair_separation_m;
            for ( it = begin; it != end; ++it){
                if(it==leg1) continue;

                if ( (*it)->object_id == (*leg1)->object_id ) {
                    leg2 = it;
                    break;
                }

                if ((*it)->object_id != "")
                    continue;

                double d = distance(it, leg1);
                if (((*it)->getLifetime() <= max_second_leg_age_s)
                        && (d < closest_dist)){
                    closest_dist = d;
                    best = it;
                }
            }

            if(leg2 != end){
                double dist_between_legs = distance(leg1, leg2);
                if (dist_between_legs > leg_pair_separation_m){
                    (*leg1)->object_id = "";
                    (*leg1)->other = NULL;
                    (*leg2)->object_id = "";
                    (*leg2)->other = NULL;
                }else{
                    (*leg1)->other = *leg2;
                    (*leg2)->other = *leg1;
                }
            }else if(best != end){
                (*best)->object_id = (*leg1)->object_id;
                (*leg1)->other = *best;
                (*best)->other = *leg1;

            }
        }

        // Attempt to pair up legs with no id
        for(;;){
            list<SavedFeature*>::iterator best1 = end, best2 = end;
            double closest_dist = leg_pair_separation_m;

            for (leg1 = begin; leg1 != end; ++leg1){
                // If this leg has an id or low reliability, skip
                if ((*leg1)->object_id != ""
                        || (*leg1)->getReliability() < leg_reliability_limit_)
                    continue;

                for ( leg2 = begin; leg2 != end; ++leg2){
                    if(((*leg2)->object_id != "")
                            || ((*leg2)->getReliability() < leg_reliability_limit_)
                            || (leg1==leg2)) continue;

                    double d = distance(leg1, leg2);

                    if(d < closest_dist){
                        best1 = leg1;
                        best2 = leg2;
                    }
                }
            }

            if(best1 != end){
                char id[100];
                float number = next_p_id_;
                //snprintf(id,100,"Person%d", next_p_id_++);
                (*best1)->object_id = std::string(id);
                (*best2)->object_id = std::string(id);
                (*best1)->other = *best2;
                (*best2)->other = *best1;
            }else{
                break;
            }
        }
    }

};


int main(int argc, char **argv)
{
	ros::init(argc, argv,"laser_processor");
	g_argc = argc;
	g_argv = argv;
      
    //if (g_argc > 2) {
        scan_topic = "hsrb/base_scan";
        //scan_topic = g_argv[2];
        //printf("Listening on topic %s : \n", g_argv[2]);
    //} else {
        //printf("Please provide the input topic as a parameter,e.g. scan_front. Assuming scan_front ! \n");
        //shutdown();
    //}

    ros::NodeHandle nh;
    LegDetector ld(nh);
	ROS_INFO("Execute main");
    ros::spin();

    return 0;
}
