#include "filter_point.h"

// ***************************************************************************
filter_point_class::filter_point_class(ros::NodeHandle* nh)
{
	wait_for_params(nh);	
	
	ptCloudSub_ = nh->subscribe("filter_point_node/pt_cloud_in", 100, &filter_point_class::pt_cloud_cb, this);
	
	isInitialized_ = false;
	
	ROS_INFO("Waiting for the input point cloud ...");
	while(!isInitialized_)
	ros::spinOnce();
	
	std::cout << "," << std::endl;
	ptVizPub_ = nh->advertise<visualization_msgs::Marker>("filter_point_node/pt_cloud_viz", 100);
	ptPub_ = nh->advertise<geometry_msgs::PointStamped>("filter_point_node/pt_out", 100);
	
	std::cout << "," << std::endl;
	distArrSize_ = ceil((maxDist_ - minDist_)/discInt_ + 1);
	distArr_ = new float[distArrSize_];
	
	std::cout << "," << std::endl;
	for (int i=0; i<distArrSize_; i++)
	distArr_[i] = minDist_ + (i*discInt_);
			
	voxGridWidth_ = ceil(float(imgWidth_)/float(discPix_));
	voxGridHeight_ = ceil(float(imgHeight_)/float(discPix_));
	
	std::cout << "," << std::endl;
	voxArrSize_ = voxGridWidth_*voxGridHeight_*distArrSize_ + 1;
	voxArr_ = new pcl::PointXYZ[voxArrSize_];
	ptsVox_ = new int[voxArrSize_];
	distVox_ = new float[voxArrSize_];
	obsWeight_ = new float[voxArrSize_];
			
	distVoxPiv_ = 0;
	nOutliers_ = 50; // per voxel
	
	std::cout << "," << std::endl;
	beliefArr_ = new int[voxArrSize_];
	std::fill(beliefArr_, beliefArr_ + voxArrSize_, partArrSize_/voxArrSize_);
	
	populate_transition_model();
}

// ***************************************************************************
filter_point_class::~filter_point_class()
{
	delete distArr_;
			
	for (int i=0; i<distArrSize_; i++)
	delete transModel_[i];
	delete transModel_;
	
	delete voxArr_;		
	delete ptsVox_;
	delete distVox_;
	delete obsWeight_;
	delete beliefArr_;
}

// ***************************************************************************
void filter_point_class::wait_for_params(ros::NodeHandle *nh)
{
	while(!nh->getParam("filter_point_node/distance_interval", discInt_));
	while(!nh->getParam("filter_point_node/pixel_interval", discPix_));
	while(!nh->getParam("filter_point_node/min_distance", minDist_));
	while(!nh->getParam("filter_point_node/max_distance", maxDist_));
	while(!nh->getParam("filter_point_node/n_particles", partArrSize_));
	
	ROS_INFO("Parameters for filter_point retreived from the parameter server");
}

// ***************************************************************************
void filter_point_class::pt_cloud_cb(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msgPtr)
{
	ros::Time begin = ros::Time::now();
	
	if(msgPtr->height == 1)
	{
		ROS_WARN("Input point cloud is unordered");
		return;
	}
	
	if(isInitialized_)
	{
		extract_features(msgPtr);
		update_belief();
		publish_voxels();
		//display(3);
	}
	
	if(!isInitialized_)
	{
		ROS_INFO("Frame Id: %s", msgPtr->header.frame_id.c_str());
		ROS_INFO("Image Height: %i", msgPtr->height);
		ROS_INFO("Image Width: %i", msgPtr->width);
		ROS_INFO("Data Size: %lu", msgPtr->points.size());
		ROS_INFO("Is Dense: %i", msgPtr->is_dense);
		
		frameId_ = msgPtr->header.frame_id;
		imgWidth_ = msgPtr->width;
		imgHeight_ = msgPtr->height;
		
		isInitialized_ = true;
	}
	
	ros::Duration elapsed = ros::Time::now() - begin;
	ROS_INFO("Time Elapsed: %f", elapsed.toSec());
}

// ***************************************************************************
void filter_point_class::populate_transition_model()
{
	transDistr_ = new std::discrete_distribution<int>[voxArrSize_];
	transModel_ = new float*[voxArrSize_];
	for (int i=0; i<voxArrSize_; i++)
	{
		transModel_[i] = new float[voxArrSize_];
		for (int j=0; j<voxArrSize_; j++)
		{
			int indexVoxZ1 = floor(float(i) / float(voxGridWidth_*voxGridHeight_));
			int indexVoxY1 = floor(float(i - indexVoxZ1*voxGridWidth_*voxGridHeight_) / float(voxGridWidth_));
			int indexVoxX1 = i - indexVoxZ1*voxGridWidth_*voxGridHeight_ - indexVoxY1*voxGridWidth_;	
							
			int indexVoxZ2 = floor(float(j) / float(voxGridWidth_*voxGridHeight_));
			int indexVoxY2 = floor(float(j - indexVoxZ2*voxGridWidth_*voxGridHeight_) / float(voxGridWidth_));
			int indexVoxX2 = j - indexVoxZ2*voxGridWidth_*voxGridHeight_ - indexVoxY2*voxGridWidth_;
					
			int indexVoxZC = 0;
			int indexVoxYC = voxGridHeight_/2;
			int indexVoxXC = voxGridWidth_/2;					
					
			int manDistMax = abs(voxGridWidth_) + abs(voxGridHeight_) + abs(distArrSize_);
					
			if (i == (voxArrSize_-1))
			{
				int manDist = abs(indexVoxX2 - indexVoxXC) + abs(indexVoxY2 - indexVoxYC) + abs(indexVoxZ2 - indexVoxZC);
				transModel_[i][j] = float(manDist) / float(manDistMax); 
			}
			else if (j == (voxArrSize_-1))
			{
				int manDist = abs(indexVoxX1 - indexVoxXC) + abs(indexVoxY1 - indexVoxYC) + abs(indexVoxZ1 - indexVoxZC);
				transModel_[i][j] = float(manDist) / float(manDistMax); // Can be made less conservative manDist << manDistMax
			}
			else if (j >= i)
			{	
				int manDist = abs(indexVoxX2 - indexVoxX1) + abs(indexVoxY2 - indexVoxY1) + abs(indexVoxZ2 - indexVoxZ1);

				transModel_[i][j] = 1 - float(manDist) / float(manDistMax);
			}
			else
				transModel_[i][j] = transModel_[j][i];
		}
			transDistr_[i] = std::discrete_distribution<int>(transModel_[i], transModel_[i]+voxArrSize_);
	}
}

// ***************************************************************************
void filter_point_class::update_belief() 
	{
	//------ Belief can be zero to 1: better solution?
	// --------voxelDist = discrete_distribution<int>(obsWeight, obsWeight+nVoxels);
	//mt19937 generator;
	//generator.seed(seed);
			
	std::discrete_distribution<int> beliefDist(beliefArr_, beliefArr_+voxArrSize_);
	std::fill(beliefArr_, beliefArr_ + voxArrSize_, 0);
			
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator(seed);			
			
	int associatedWeight[partArrSize_];
	int associatedVoxel[partArrSize_];
			
	for (int i=0; i<partArrSize_; i++)
	{
		int sampledIndex = beliefDist(generator);
		int nextIndex = transDistr_[sampledIndex](generator);
				
		associatedVoxel[i] = nextIndex;
		associatedWeight[i] = obsWeight_[nextIndex];
	}
	
	std::discrete_distribution<int> particleDist(associatedWeight, associatedWeight+partArrSize_);
			
	int maxIndex = 0;
	int maxPart = 0;
			
	for (int i=0; i<partArrSize_; i++)
	{
		int sampledIndex = particleDist(generator);
		beliefArr_[associatedVoxel[sampledIndex]]++;
				
		if (beliefArr_[associatedVoxel[sampledIndex]] > maxPart)
		{
			maxPart = beliefArr_[associatedVoxel[sampledIndex]];
			maxIndex = associatedVoxel[sampledIndex];
		}
	}
	
	domPt_ = voxArr_[maxIndex];	
}

// ***************************************************************************
int filter_point_class::random_index(float* belief, int& size)
{
	std::discrete_distribution<int> distObject(belief, belief+size);
	std::cout << "Probabilities: " << "[ ";
	
	for(int i=0; i<size; i++)
		std::cout << distObject.probabilities()[i] << ", ";
	std::cout << " ]" << std::endl;
			
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
			
	std::default_random_engine generator(seed);
			
	return distObject(generator);	
} 
		
// ***************************************************************************
void filter_point_class::extract_features(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& ptCloudPtr)
{
	std::fill(voxArr_, voxArr_ + voxArrSize_, pcl::PointXYZ(0,0,0));
	std::fill(ptsVox_, ptsVox_ + voxArrSize_, 0);
	std::fill(distVox_, distVox_ + voxArrSize_, 0);
	distVoxPiv_ = maxDist_;
  			
  int indexVox;
  float distVox;
  int nValidPts = 0;
  		
	for (int i=0; i<ptCloudPtr->size(); i++)
	{
		if(!is_valid(ptCloudPtr->points[i]))
		continue;
				
		nValidPts++;
  	if(!point_to_voxel(ptCloudPtr->points[i], i, indexVox, distVox))
  	continue;

  	ptsVox_[indexVox] ++;
  	voxArr_[indexVox] = ptCloudPtr->points[i];
  	distVox_[indexVox] = distVox;
  				
  	if(distVox_[indexVox] < distVoxPiv_)
  	distVoxPiv_ = distVox_[indexVox];
  }
  		
  for (int i=0; i<(voxArrSize_-1); i++)
  {
  	float nearnessWeight = 1 - abs(distVox_[i] - distVoxPiv_) / (maxDist_ - minDist_);
  	obsWeight_[i] = nearnessWeight * ptsVox_[i];
  }
  obsWeight_[voxArrSize_-1] = nOutliers_;
}
		
// ***************************************************************************
bool filter_point_class::point_to_voxel(const pcl::PointXYZ& pt, int& indexPt, int& indexVox, float& distVox)
{
	distVox = pcl::euclideanDistance (pcl::PointXYZ(0,0,0), pt);
			
	if((distVox > maxDist_) || (distVox < minDist_))
	return false;
			
	int indexVoxX = floor(float(indexPt % imgWidth_) / float(discPix_));
	int indexVoxY = floor(floor(float(indexPt)/float(imgWidth_)) / float(discPix_));
	int indexVoxZ = floor((distVox - minDist_) /  discInt_);
			
	indexVox = indexVoxX + voxGridWidth_*(indexVoxY + voxGridHeight_*indexVoxZ);
			
	return true;
}
		
// ***************************************************************************
bool filter_point_class::is_valid(const pcl::PointXYZ& point)
{
	return !(isnan(point.x) || isnan(point.y) || isnan(point.z));
}
		
// ***************************************************************************
void filter_point_class::publish_voxels()
{
	visualization_msgs::Marker markerMsg;
	
	markerMsg.header.frame_id = frameId_;
	markerMsg.header.stamp = ros::Time::now();
	markerMsg.type = visualization_msgs::Marker::CUBE_LIST;
	markerMsg.action = visualization_msgs::Marker::ADD;
	markerMsg.lifetime = ros::Duration(0);
	markerMsg.frame_locked = true;
	
	geometry_msgs::Pose pose;
	pose.orientation.w = 1;
	markerMsg.pose = pose;
	
	geometry_msgs::Vector3 scale;
	scale.x = 0.05;
	scale.y = 0.05;
	scale.z = 0.05;
	markerMsg.scale = scale;
	
	markerMsg.ns = "filter_point";
	for (int i=0; i<voxArrSize_; i++)
	{
		markerMsg.id = i;
		
		geometry_msgs::Point pt;
		pt.x = voxArr_[i].x; 
		pt.y = voxArr_[i].y;
		pt.z = voxArr_[i].z;
		markerMsg.points.push_back(pt);
		
		//float shade = float(beliefArr_[i])/float(partArrSize_);
		
		std_msgs::ColorRGBA color;
		color.r = 0; 
		color.g = 0; 
		color.b = 1; 
		color.a = 1;
		markerMsg.colors.push_back(color);
	}
	
	ptVizPub_.publish(markerMsg);
			
	geometry_msgs::PointStamped ptMsg;
	ptMsg.header.stamp = ros::Time::now();
	ptMsg.header.frame_id = frameId_;
	ptMsg.point.x = domPt_.x;
	ptMsg.point.y = domPt_.y;
	ptMsg.point.z = domPt_.z;

	ptPub_.publish(ptMsg);		
}
		
// ***************************************************************************
void filter_point_class::display(int precision)
{
	std::cout << std::setprecision(precision) << std::endl;
	std::cout << "<===============================================>" << std::endl; 
	std::cout << "Number of possible states: " << voxArrSize_ << std::endl;
	std::cout << "All possible states: [";
	for (int i=0; i<(voxArrSize_-1); i++)
		std::cout << voxArr_[i] << ", ";
		
	std::cout << voxArr_[voxArrSize_-1] << "]" << std::endl;
	std::cout << "Transition Model: " << std::endl;
	for (int i=0; i<voxArrSize_; i++)
	{
		for (int j=0; j<voxArrSize_; j++)
			std::cout << transModel_[i][j] << "\t";
		std::cout << std::endl;
	}

	for (int i=0; i<voxArrSize_; i++)
	{
		std::cout << "Number of Voxel Points: " << ptsVox_[i] << std::endl;
		std::cout << "Distance from Origin: " << distVox_[i] << std::endl;
		std::cout << "Distance of Nearest Voxel: " << distVoxPiv_ << std::endl;
		std::cout << "Distance from Nearest Voxel: " << abs(distVox_[i] - distVoxPiv_) << std::endl;
		std::cout << "Observation Weight: " << obsWeight_[i] << std::endl;
		std::cout << "Number of Particles: " << beliefArr_[i] << std::endl << std::endl;
		std::cout << "Probability of Voxel: " << float(beliefArr_[i])/float(partArrSize_) << std::endl; 
	}
	std::cout << std::endl;
};

// *******************************************************************
