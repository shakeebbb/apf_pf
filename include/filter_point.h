#ifndef FILTERPOINT_H
#define FILTERPOINT_H

#include <random>
#include <chrono>
#include <algorithm>
#include <iostream>
#include <string>
#include <omp.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include "pcl/PCLPointCloud2.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/distances.h"
#include "pcl/point_representation.h"
#include "pcl_ros/point_cloud.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/CameraInfo.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class filter_point_class
{		

private:

	// ROS Related
	ros::Subscriber ptCloudSub_;
	ros::Subscriber camInfoSub_;
	ros::Subscriber twistSub_;
	ros::Subscriber imuSub_;

	ros::Publisher vizPub_;
	ros::Publisher ptPub_;
	ros::Publisher ptPivPub_;
	ros::Publisher actPub_;
  ros::Publisher forcePub_;
  ros::Publisher beliefPub_;
  ros::Publisher computeTimePub_;
	
	tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener* tfListenerPtr_;
  
  std::string baseFrameId_;
  std::string camFrameId_;
  
  geometry_msgs::TransformStamped camToBaseTransform_;
		
	// States, Discretizations
	geometry_msgs::TwistStamped robotVel_; 
		
	pcl::PointXYZ* voxArr_;
	int voxArrSize_;
	int voxGridWidth_;
	int voxGridHeight_;
	int voxGridDepth_;
		
	int imgWidth_;
	int imgHeight_;
	int pixInt_;
		
	double minDist_;
	double maxDist_;
	double distInt_;
		
	std::discrete_distribution<int>* voxTransModel_;
	
	std::vector<double> sdevTrans_; // sdevVox
	std::vector<double> sdevObsv_; // sdevNPts, sdevDist
	
	tf2::Quaternion imuOrient_;
	
	// Actions
	double** actArr_;
	int actArrSize_;
	std::vector<double> minAct_; // x, y, yaw
	std::vector<double> maxAct_;
	std::vector<double> actInt_;

  bool apfOut_;
  bool qmdpOut_;
		
	// Distributions
	std::discrete_distribution<int> voxBeliefDistr_;
	
	// QMDP
	Eigen::MatrixXd alphaMat_;
	Eigen::MatrixXd rewMat_;
	
	int alphaItr_;
	
	std::vector<double> rewQ_;
	double repPotMaxDist_;
	double repPotGain_;
		
	// Observations, Features
	int* ptsVox_;
	
	pcl::PointXYZ ptPiv_;
		
	int nOutliers_;
	
	int voxPartArrSize_;
	
	double camInfoP_[9];
		
	// Filter Status
	uint8_t isInitialized_;
	
	double lookaheadT_;
	double deltaT_;
	
	// IO
	std::fstream* fout_;
	std::string filePath_;
	bool readFromFile_;

  bool display_;
  bool viz_;
	
	// Parallelization
	int nThreads_;
		
public:
	
	// *******************************************************************
	filter_point_class(ros::NodeHandle*);
	~filter_point_class();

	// *******************************************************************
	void pt_cloud_cb(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&);
	void cam_info_cb(const sensor_msgs::CameraInfo::ConstPtr&);
	void twist_cb(const geometry_msgs::TwistStamped::ConstPtr&);
	void imu_cb(const sensor_msgs::Imu::ConstPtr&);
	
	// *******************************************************************
	pcl::PointXYZ apply_action(int, int, double, bool = true);
	int point2_to_voxel(pcl::PointXYZ);
	pcl::PointXYZ point2_to_point3(pcl::PointXYZ, bool = true);
	pcl::PointXYZ indx_to_vox(int);
	double man_dist_vox(int, int);
	double man_dist_to_bound(int);
	void wait_for_params(ros::NodeHandle*);
	void populate_transition_model();
	void update_belief();
	static int random_index(double*, int&);
	void extract_features(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&);
	bool point_to_voxel(const pcl::PointXYZ&, int&, int&, double&);
	bool is_valid(const pcl::PointXYZ&);
	void publish_viz(double*, std::string = "all");
	void publish_action(int);
  void publish_force(double*);
  void publish_points();
  void publish_compute_time(ros::Time&);
	void display(std::string, int);
	double norm_pdf(double, double, double, bool = true);
	void discretize_image();
	void populate_reward_model();
	double repulsive_potential(pcl::PointXYZ);
	void discretize_actions();
	void compute_alpha_vectors(int);
	int update_action();
	std::discrete_distribution<int> particle_filter(std::discrete_distribution<int>&, 
																		std::discrete_distribution<int>*, double*, int);
	bool read_from_file(std::string);
	bool write_to_file(std::string);
	bool read_csv(std::string, Eigen::MatrixXd&, int, int);
	bool read_csv(std::string, std::discrete_distribution<int>*, int, int);
	bool write_csv(std::string, Eigen::MatrixXd);

  geometry_msgs::Vector3 get_rep_force(int, double, double, double);
  
};
	
#endif
