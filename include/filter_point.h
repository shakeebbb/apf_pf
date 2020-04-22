#ifndef FILTERPOINT_H
#define FILTERPOINT_H

#include "ros/ros.h"
#include <random>
#include <chrono>
#include "sensor_msgs/PointCloud2.h"
#include "pcl/PCLPointCloud2.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/common/distances.h"
#include "pcl/point_representation.h"
#include "pcl_ros/point_cloud.h"
#include "geometry_msgs/PointStamped.h"
#include "visualization_msgs/Marker.h"

class filter_point_class
{		

private:

	// ROS Related
	ros::Subscriber ptCloudSub_;
	ros::Publisher ptVizPub_;
	ros::Publisher ptPub_;
	std::string frameId_;
		
	// States, Discretizations
	float* distArr_;
	int distArrSize_; 
		
	pcl::PointXYZ* voxArr_;
	int voxArrSize_;
	int voxGridWidth_;
	int voxGridHeight_;
		
	int imgWidth_;
	int imgHeight_;
	int discPix_;
		
	float minDist_;
	float maxDist_;
	float discInt_;
		
	float** transModel_;
		
	// Distributions
	std::discrete_distribution<int>* transDistr_;
	std::discrete_distribution<int> voxDistr_;
		
	// Observations, Features
	int* ptsVox_;
	float* distVox_;
	float distVoxPiv_;
		
	int nOutliers_;
	float* obsWeight_;
	
	int* beliefArr_;
	int partArrSize_;
		
	pcl::PointXYZ domPt_;
	int domVoxIndx_;
		
	// Filter Status
	bool isInitialized_;
		
public:
	
	// *******************************************************************
	filter_point_class(ros::NodeHandle*);
	~filter_point_class();

	// *******************************************************************
	void pt_cloud_cb(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&);
	
	// *******************************************************************
	void wait_for_params(ros::NodeHandle*);
	void populate_transition_model();
	void update_belief();
	static int random_index(float*, int&);
	void extract_features(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&);
	bool point_to_voxel(const pcl::PointXYZ&, int&, int&, float&);
	bool is_valid(const pcl::PointXYZ&);
	void publish_voxels();
	void display(int);
};
	
#endif
