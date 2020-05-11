#ifndef FILTERPOINT_H
#define FILTERPOINT_H

#include <random>
#include <chrono>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
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
#include "visualization_msgs/Marker.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class filter_point_class
{		

private:

	// ROS Related
	ros::Subscriber ptCloudSub_;
	ros::Subscriber camInfoSub_;
	ros::Subscriber twistSub_;
	ros::Subscriber imuSub_;
	ros::Publisher ptVizPub_;
	ros::Publisher ptPub_;
	ros::Publisher ptPivPub_;
	
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
		
	float minDist_;
	float maxDist_;
	float distInt_;
	
	float** velArr_;
	int velArrSize_;
	std::vector<float> minVel_; // x, y, yaw
	std::vector<float> maxVel_;
	std::vector<float> velInt_;
		
	std::discrete_distribution<int>** voxTransModel_;
	std::discrete_distribution<int>** velTransModel_;
	
	std::vector<float> sdevTrans_; // sdevVox, sdevVel
	std::vector<float> sdevObsv_; // sdevVel, sdevNPts, sdevDist
	
	tf2::Quaternion imuOrient_;
	
	// Actions
	float** actArr_;
	int actArrSize_;
	std::vector<float> minAct_; // x, y, yaw
	std::vector<float> maxAct_;
	std::vector<float> actInt_;
		
	// Distributions
	std::discrete_distribution<int> voxBeliefDistr_;
	std::discrete_distribution<int> velBeliefDistr_;
		
	// Observations, Features
	int* ptsVox_;
	
	pcl::PointXYZ ptPiv_;
		
	int nOutliers_;
	
	int voxPartArrSize_;
	int velPartArrSize_;
	
	float camInfoP_[9];
		
	// Filter Status
	uint8_t isInitialized_;
	
	float deltaT_;
		
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
	int apply_action(int, int);
	int point2_to_voxel(pcl::PointXYZ);
	pcl::PointXYZ point2_to_point3(pcl::PointXYZ, bool);
	pcl::PointXYZ indx_to_vox(int);
	float man_dist_vox(int, int);
	float man_dist_to_bound(int);
	void wait_for_params(ros::NodeHandle*);
	void populate_transition_model();
	void update_belief(int);
	static int random_index(float*, int&);
	void extract_features(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&);
	bool point_to_voxel(const pcl::PointXYZ&, int&, int&, float&);
	bool is_valid(const pcl::PointXYZ&);
	void publish_voxels();
	void display(int);
	float norm_pdf(float, float, float, bool = true);
	std::discrete_distribution<int> particle_filter(std::discrete_distribution<int>&, 
																		std::discrete_distribution<int>*, float*, int);
};
	
#endif
