#include "filter_point.h"

// ***************************************************************************
filter_point_class::filter_point_class(ros::NodeHandle* nh)
{
	// ROS stuff
	wait_for_params(nh);	
	
	ptCloudSub_ = nh->subscribe("filter_point_node/pt_cloud_in", 100, &filter_point_class::pt_cloud_cb, this);
	camInfoSub_ = nh->subscribe("filter_point_node/cam_info_in", 100, &filter_point_class::cam_info_cb, this);
	twistSub_ = nh->subscribe("filter_point_node/twist_in", 100, &filter_point_class::twist_cb, this);
	imuSub_ = nh->subscribe("filter_point_node/imu_in", 100, &filter_point_class::imu_cb, this);
	
	isInitialized_ = 0x00;
	
	ROS_INFO("Initializing filter_point ...");
	while( (isInitialized_ & 0x03) != 0x03 )
	ros::spinOnce();
	
	ROS_INFO("Received input cloud and camera info");
	
	std::cout << "," << std::endl;
	ptVizPub_ = nh->advertise<visualization_msgs::Marker>("filter_point_node/pt_cloud_viz", 100);
	ptPub_ = nh->advertise<geometry_msgs::PointStamped>("filter_point_node/pt_out", 100);
	ptPivPub_ = nh->advertise<geometry_msgs::PointStamped>("filter_point_node/pt_piv_out", 100);
	
	tfListenerPtr_ = new tf2_ros::TransformListener(tfBuffer_);
	
	// Voxel array
	voxGridWidth_ = ceil(float(imgWidth_)/float(pixInt_));
	voxGridHeight_ = ceil(float(imgHeight_)/float(pixInt_));
	voxGridDepth_ = ceil( (maxDist_ - minDist_)/distInt_ + 1 );
	
	std::cout << "," << std::endl;
	voxArrSize_ = voxGridWidth_*voxGridHeight_*voxGridDepth_ + 1;
	voxArr_ = new pcl::PointXYZ[voxArrSize_];
	
	int count = 0;
	for (int i=0; i<voxGridDepth_; i++)
		for (int j=0; j<voxGridHeight_; j++)
			for (int k=0; k<voxGridWidth_; k++)
			{
				voxArr_[count] = pcl::PointXYZ(k*pixInt_, j*pixInt_, minDist_ + i*distInt_);
				count ++;
			}
	
	// Voxel observations
	ptsVox_ = new int[voxArrSize_];
	
	std::cout << "," << std::endl;
	
	// Voxel belief
	int voxBelief[voxArrSize_];
	std::fill(voxBelief, voxBelief + voxArrSize_, velPartArrSize_/voxArrSize_);
	voxBeliefDistr_ = std::discrete_distribution<int>(voxBelief, voxBelief + voxArrSize_);
	
	int velBelief[velArrSize_];
	std::fill(velBelief, velBelief + velArrSize_, velPartArrSize_/voxArrSize_);
	velBeliefDistr_ = std::discrete_distribution<int>(velBelief, velBelief + velArrSize_);
	
	std::cout << "," << std::endl;
	
	// Action array
	actArrSize_ = ceil((maxAct_[0] - minAct_[0])/actInt_[0] + 1) * 
								ceil((maxAct_[1] - minAct_[1])/actInt_[1] + 1) * 
								ceil((maxAct_[2] - minAct_[2])/actInt_[2] + 1) + 1;
	
	std::cout << "actArrSize_ = " << actArrSize_ << std::endl;
	
	actArr_ = new float*[actArrSize_];
	count = 0;
	for (float i=minAct_[0]; i<=maxAct_[0]; i+=actInt_[0])
	{
		for (float j=minAct_[1]; j<=maxAct_[1]; j+=actInt_[1])
		{
			for (float k=minAct_[2]; k<=maxAct_[2]; k+=actInt_[2])
			{
				actArr_[count] = new float[3];
				actArr_[count][0] = i;
				actArr_[count][1] = j;
				actArr_[count][2] = k;
				count++;
				std::cout << i << ", " << j << ", " << k << std::endl;
			}
		}
		getchar();
	}
	
	std::cout << "k" << std::endl;
	
	actArr_[actArrSize_-1] = new float[3];
	actArr_[actArrSize_-1][0] = 0;
	actArr_[actArrSize_-1][1] = 0;
	actArr_[actArrSize_-1][2] = 0;
	
	std::cout << "," << std::endl;
	
	// Velocity array
	velArrSize_ = ceil((maxVel_[0] - minVel_[0])/velInt_[0] + 1) * 
								ceil((maxVel_[1] - minVel_[1])/velInt_[1] + 1) * 
								ceil((maxVel_[2] - minVel_[2])/velInt_[2] + 1) + 1;
	velArr_ = new float*[velArrSize_];
	count = 0;
	for (float i=minVel_[0]; i<=maxVel_[0]; i+=velInt_[0])
		for (float j=minVel_[1]; j<=maxVel_[1]; j+=velInt_[1])
			for (float k=minVel_[2]; k<=maxVel_[2]; k+=velInt_[2])
			{
				velArr_[count] = new float[3];
				velArr_[count][0] = i;
				velArr_[count][1] = j;
				velArr_[count][2] = k;
				count++;
			}
	
	velArr_[velArrSize_-1] = new float[3];
	
	velArr_[velArrSize_-1][0] = 0;
	velArr_[velArrSize_-1][1] = 0;
	velArr_[velArrSize_-1][2] = 0;
	
	std::cout << "," << std::endl;
	
	// Static transforms
	while(!tfBuffer_.canTransform (baseFrameId_, camFrameId_, ros::Time(0)))
	ROS_WARN_THROTTLE(1, "Transform unavailable: %s to %s", baseFrameId_.c_str(), camFrameId_.c_str());
	
	camToBaseTransform_ = tfBuffer_.lookupTransform (baseFrameId_, camFrameId_, ros::Time(0));
	
	// Function calls	 
	populate_transition_model();
}

// ***************************************************************************
filter_point_class::~filter_point_class()
{
	for (int i=0; i<actArrSize_; i++)
	delete voxTransModel_[i];
	delete voxTransModel_;
	
	for (int i=0; i<actArrSize_; i++)
	delete velTransModel_[i];
	delete velTransModel_;
	
	for (int i=0; i<actArrSize_; i++)
	delete actArr_[i];
	delete actArr_;
	
	for (int i=0; i<velArrSize_; i++)
	delete velArr_[i];
	delete velArr_;
	
	delete voxArr_;		
	delete ptsVox_;
}

// ***************************************************************************
void filter_point_class::wait_for_params(ros::NodeHandle *nh)
{
	while(!nh->getParam("filter_point_node/distance_interval", distInt_));
	while(!nh->getParam("filter_point_node/pixel_interval", pixInt_));
	while(!nh->getParam("filter_point_node/min_distance", minDist_));
	while(!nh->getParam("filter_point_node/max_distance", maxDist_));
	
	while(!nh->getParam("filter_point_node/n_particles_vox", voxPartArrSize_));
	while(!nh->getParam("filter_point_node/n_particles_vel", velPartArrSize_));
	while(!nh->getParam("filter_point_node/outliers_per_voxel", nOutliers_));
	
	while(!nh->getParam("filter_point_node/trans_noise_sdev", sdevTrans_));
	while(!nh->getParam("filter_point_node/obsv_noise_sdev", sdevObsv_));
	
	while(!nh->getParam("filter_point_node/min_action_values", minAct_));
	while(!nh->getParam("filter_point_node/max_action_values", maxAct_));
	while(!nh->getParam("filter_point_node/action_intervals", actInt_));
	
	while(!nh->getParam("filter_point_node/min_velocity_values", minVel_));
	while(!nh->getParam("filter_point_node/max_velocity_values", maxVel_));
	while(!nh->getParam("filter_point_node/velocity_intervals", velInt_));
	
	while(!nh->getParam("filter_point_node/sampling_time", deltaT_));
	while(!nh->getParam("filter_point_node/base_frame_id", baseFrameId_));
	
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
	
	if((isInitialized_ & 0x03) == 0x03)
	{
		extract_features(msgPtr);
		update_belief(actArrSize_-1);
		publish_voxels();
		//display(3);
	}
	
	if((isInitialized_ & 0x01) != 0x01)
	{
		ROS_INFO("Frame Id: %s", msgPtr->header.frame_id.c_str());
		ROS_INFO("Image Height: %i", msgPtr->height);
		ROS_INFO("Image Width: %i", msgPtr->width);
		ROS_INFO("Data Size: %lu", msgPtr->points.size());
		ROS_INFO("Is Dense: %i", msgPtr->is_dense);
		
		camFrameId_ = msgPtr->header.frame_id;
		imgWidth_ = msgPtr->width;
		imgHeight_ = msgPtr->height;
		
		isInitialized_ |= 0x01;
	}
	
	ros::Duration elapsed = ros::Time::now() - begin;
	ROS_INFO("Time Elapsed: %f", elapsed.toSec());
}

// ***************************************************************************
void filter_point_class::cam_info_cb(const sensor_msgs::CameraInfo::ConstPtr& msgPtr)
{
	if( (isInitialized_ & 0x02) != 0x02 )
	{
		std::cout << "Matrix P ";
		
		for(int i=0; i<9; i++)
		{
			camInfoP_[i] = msgPtr->P[i];
			
			std::cout << camInfoP_[i] << ", ";
		}
		
		std::cout << std::endl;
		
		isInitialized_ |= 0x02;
	}
	
	std::cout << "cam_info" << std::endl;
}

// ***************************************************************************
void filter_point_class::imu_cb(const sensor_msgs::Imu::ConstPtr& msgPtr)
{
	imuOrient_ = tf2::Quaternion(msgPtr->orientation.x,
															 msgPtr->orientation.y,
															 msgPtr->orientation.z,
															 msgPtr->orientation.w);
}

// ***************************************************************************
void filter_point_class::twist_cb(const geometry_msgs::TwistStamped::ConstPtr& msgPtr)
{
	geometry_msgs::Vector3Stamped linVelIn;
	linVelIn.header = msgPtr->header;
	linVelIn.vector = msgPtr->twist.linear;
	
	geometry_msgs::Vector3Stamped angVelIn;
	angVelIn.header = msgPtr->header;
	angVelIn.vector = msgPtr->twist.angular;
	
	geometry_msgs::Vector3Stamped linVelOut;
	geometry_msgs::Vector3Stamped angVelOut;
	
	try
	{
		geometry_msgs::TransformStamped twistTransform = tfBuffer_.lookupTransform (baseFrameId_, msgPtr->header.frame_id, ros::Time(0));
		tf2::doTransform(linVelIn, linVelOut, twistTransform);
		tf2::doTransform(angVelIn, angVelOut, twistTransform);
		
		robotVel_.header = twistTransform.header;
		robotVel_.twist.linear = linVelOut.vector;
		robotVel_.twist.linear = angVelOut.vector;
	}
	catch(tf2::TransformException &ex)
	{
		ROS_WARN("%s",ex.what());
		robotVel_ = *msgPtr;
	}
}

// ***************************************************************************
int filter_point_class::apply_action(int indxVoxIn, int indxAct)
{
	if(indxVoxIn == (voxArrSize_ - 1))
	return voxArrSize_;
	
	//std::cout << "Projecting 2.5D point " << voxArr_[indxVoxIn] << std::endl;
	
	pcl::PointXYZ point3 = point2_to_point3(voxArr_[indxVoxIn], true);
	
	//std::cout << "Projected 3D point " << point3 << std::endl;

	tf2::Transform cam2base( tf2::Quaternion(camToBaseTransform_.transform.rotation.x, camToBaseTransform_.transform.rotation.y,
														  						 camToBaseTransform_.transform.rotation.z, camToBaseTransform_.transform.rotation.w), 
													 tf2::Vector3(camToBaseTransform_.transform.translation.x,
																			  camToBaseTransform_.transform.translation.y,
																			  camToBaseTransform_.transform.translation.z) ); 
	
	tf2::Transform baseP2base( tf2::Quaternion(actArr_[indxAct][2]*deltaT_, 0, 0), 
														 tf2::Vector3(actArr_[indxAct][0]*deltaT_, 0, actArr_[indxAct][1]*deltaT_) );
	
	tf2::Vector3 point3Tf = cam2base.inverse() * baseP2base.inverse() * cam2base * tf2::Vector3(point3.x, point3.y, point3.z);
	
	//std::cout << "Tranformed 3D point " << point3Tf.x() << ", " << point3Tf.y() << ", " << point3Tf.z() << std::endl;
	
	pcl::PointXYZ point2 = point2_to_point3(pcl::PointXYZ(point3Tf.x(), point3Tf.y(), point3Tf.z()), false);
	
	//std::cout << "Projected 2.5D point " << point2 << std::endl;
	
	int indxVoxOut;
	
	if(point2.x >= voxGridWidth_*pixInt_ || point2.x < 0 ||
		 point2.y >= voxGridHeight_*pixInt_ || point2.y < 0 ||
		 point2.z >= maxDist_ || point2.z <= minDist_)
	indxVoxOut = voxArrSize_;
	
	else
	{
		//std::cout	<< "Next 2.5D point " << point2 << std::endl;
		indxVoxOut = point2_to_voxel(point2);
	}

	//std::cout << "Returning next voxel" << std::endl;
	return indxVoxOut;
}

// ***************************************************************************
int filter_point_class::point2_to_voxel(pcl::PointXYZ pointIn)
{
	pcl::PointXYZ pointOut;
	
	int xIndx = ceil(float(pointIn.x) / float(pixInt_));
	int yIndx = ceil(float(pointIn.y) / float(pixInt_));
	int zIndx = ceil((pointIn.z - minDist_) / distInt_);
	
	return xIndx + voxGridWidth_*(yIndx + voxGridHeight_*zIndx);
}

// ***************************************************************************
pcl::PointXYZ filter_point_class::point2_to_point3(pcl::PointXYZ pointIn, bool direction)
{
	pcl::PointXYZ pointOut;
	
	float fx = camInfoP_[0];
	float cx = camInfoP_[2];
	float fy = camInfoP_[5];
	float cy = camInfoP_[6];
	
	//std::cout << "Camera intrinsics: " << fx << ", " << cx << ", " << fy << ", " << cy << std::endl;
		
	if(direction) // u,v,w => x,y,z
	{
		pointOut.x = (pointIn.x - cx) * pointIn.z / fx;
		pointOut.y = (pointIn.y - cy) * pointIn.z / fy;
		pointOut.z = pointIn.z;
	}
	else // x,y,z => u,v,w
	{		
		pointOut.x = (pointIn.x / pointIn.z) * fx + cx;
		pointOut.y = (pointIn.y / pointIn.z) * fy + cy;
		pointOut.z = pointIn.z;
	}
	
	return pointOut;
}

// ***************************************************************************
float filter_point_class::man_dist_vox(int indxVox1, int indxVox2)
{
	if( indxVox1 == (voxArrSize_ - 1) )
	return man_dist_to_bound(indxVox2);
	
	if( indxVox2 == (voxArrSize_ - 1) )
	return man_dist_to_bound(indxVox1);
	
	return abs( (voxArr_[indxVox2].x - voxArr_[indxVox1].x) / pixInt_ ) + 
				 abs( (voxArr_[indxVox2].y - voxArr_[indxVox1].y) /pixInt_ ) + 
				 abs( (voxArr_[indxVox2].z - voxArr_[indxVox1].z) /pixInt_ );
}

// ***************************************************************************
float filter_point_class::man_dist_to_bound(int indxVox1)
{	
	float dist[5];
	
	// right, left, front, upper, lower
	
	dist[0] = abs( voxArr_[indxVox1].x / pixInt_ - voxGridWidth_ ); 
	dist[1] = abs( voxArr_[indxVox1].x / pixInt_ + 1 );
	dist[2] = abs( (voxArr_[indxVox1].z - minDist_) / distInt_ - voxGridDepth_ );
	dist[3] = abs( voxArr_[indxVox1].y / pixInt_ + 1 );
	dist[4] = abs( voxArr_[indxVox1].y / pixInt_ - voxGridHeight_); 
	
	return *std::min_element(dist, dist+5);
}

// ***************************************************************************
pcl::PointXYZ filter_point_class::indx_to_vox(int indxVox)
{
	pcl::PointXYZ outVox;
	
	outVox.z = floor(float(indxVox) / float(voxGridWidth_*voxGridHeight_));
	outVox.y = floor(float(indxVox - outVox.z*voxGridWidth_*voxGridHeight_) / float(voxGridWidth_));
	outVox.x = indxVox - outVox.z*voxGridWidth_*voxGridHeight_ - outVox.y*voxGridWidth_;
	
	return outVox;
}

// ***************************************************************************
float filter_point_class::norm_pdf(float mean, float sdev, float xIn, bool isHalf)
{
    static const float inv_sqrt_2pi = 0.3989422804014327;
    float a = (xIn - mean) / sdev;

    return inv_sqrt_2pi / sdev * std::exp(-0.5f * a * a);
}

// ***************************************************************************
void filter_point_class::populate_transition_model()
{
	voxTransModel_ = new std::discrete_distribution<int>*[actArrSize_];
	
	for (int i=0; i<actArrSize_; i++)
	{
		voxTransModel_[i] = new std::discrete_distribution<int>[voxArrSize_];
		
		for (int j=0; j<voxArrSize_; j++)
		{
			//if(abs(actArr_[i][0]) <= 0.2 && abs(actArr_[i][1]) <= 0.2 && abs(actArr_[i][2]) <= 0.2)
			//{
				std::cout << std::endl;
				std::cout << "Applying action " << actArr_[i][0] << ", " << actArr_[i][1] << ", " << actArr_[i][2] << std::endl;
				std::cout << "Current voxel " << voxArr_[j].x << ", " << voxArr_[j].y << ", " << voxArr_[j].z << std::endl;  
			//}
			int indxVoxP = apply_action(j, i);
			//if(abs(actArr_[i][0]) <= 0.2 && abs(actArr_[i][1]) <= 0.2 && abs(actArr_[i][2]) <= 0.2)
			//{
				std::cout << "Next voxel Id " << indxVoxP << std::endl;
				std::cout << "Next voxel " << voxArr_[indxVoxP].x << ", " << voxArr_[indxVoxP].y << ", " << voxArr_[indxVoxP].z << std::endl; 
			//}
			float transWeights[voxArrSize_];
			
			for (int k=0; k<voxArrSize_; k++)
			transWeights[k] = norm_pdf( 0, sdevTrans_[0], man_dist_vox(indxVoxP, k), false );
			
			voxTransModel_[i][j] = std::discrete_distribution<int>(transWeights, transWeights+voxArrSize_);
		}
	}
	
	velTransModel_ = new std::discrete_distribution<int>*[actArrSize_];
	
	for (int i=0; i<actArrSize_; i++)
	{
		velTransModel_[i] = new std::discrete_distribution<int>[velArrSize_];
		
		float transWeights[velArrSize_];
		
		for (int k=0; k<velArrSize_; k++)
		transWeights[k] = norm_pdf(actArr_[i][0], sdevTrans_[1], velArr_[k][0]) * 
											norm_pdf(actArr_[i][1], sdevTrans_[1], velArr_[k][1]) * 
											norm_pdf(actArr_[i][2], sdevTrans_[1], velArr_[k][2]);
		
		velTransModel_[i][0] = std::discrete_distribution<int>(transWeights, transWeights+velArrSize_);
		
		for (int j=1; j<velArrSize_; j++)
		velTransModel_[i][j] = velTransModel_[i][0];
	}
}

// ***************************************************************************
void filter_point_class::update_belief(int actIndx) 
{
	//------ Belief can be zero to 1: better solution?
	// --------voxelDist = discrete_distribution<int>(obsWeight, obsWeight+nVoxels);
	//mt19937 generator;
	//generator.seed(seed);
	
	float voxObsWeight[voxArrSize_];
	float velObsWeight[velArrSize_];
	
	for (int i=0; i<(voxArrSize_-1); i++)
  {  	
  	voxObsWeight[i] = norm_pdf(0, sdevObsv_[0], (pixInt_ * pixInt_) - ptsVox_[i], false) * 
  										 norm_pdf(0, sdevObsv_[1], voxArr_[i].z, false);
  }
  voxObsWeight[voxArrSize_-1] = norm_pdf(0, sdevObsv_[0], (pixInt_ * pixInt_) - nOutliers_, false);
	
	for (int i=0; i<velArrSize_; i++)
  {
  	velObsWeight[i] = norm_pdf(robotVel_.twist.linear.x, sdevObsv_[2], actArr_[actIndx][0]) * 
  										 norm_pdf(robotVel_.twist.linear.y, sdevObsv_[2], actArr_[actIndx][1]) * 
  										 norm_pdf(robotVel_.twist.linear.z, sdevObsv_[2], actArr_[actIndx][2]);
  }
			
	//std::discrete_distribution<int> voxBeliefDist(voxBeliefArr_, voxBeliefArr_+voxArrSize_);
	//std::fill(voxBeliefArr_, voxBeliefArr_ + voxArrSize_, 0);
	
	voxBeliefDistr_ = particle_filter(voxBeliefDistr_, voxTransModel_[actIndx], voxObsWeight, voxPartArrSize_);	
	velBeliefDistr_ = particle_filter(velBeliefDistr_, velTransModel_[actIndx], velObsWeight, velPartArrSize_);	
}

// ***************************************************************************
std::discrete_distribution<int> 
filter_point_class::particle_filter(std::discrete_distribution<int>& initBeliefDist, 
																		std::discrete_distribution<int>* transModel,
																		float *obsWeight, int nParts)
{		
	int partWeight[nParts];
	int partState[nParts];
	
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator(seed);
			
	for (int i=0; i<nParts; i++)
	{
		int sampledState = initBeliefDist(generator);
		int nextState = transModel[sampledState](generator);
				
		partState[i] = nextState;
		partWeight[i] = obsWeight[nextState];
	}
	
	std::discrete_distribution<int> partDistr(partWeight, partWeight+nParts);

	int updatedBelief[initBeliefDist.probabilities().size()];
			
	for (int i=0; i<nParts; i++)
	{
		int sampledPart = partDistr(generator);
		updatedBelief[partState[sampledPart]]++;
	}
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
	//std::fill(voxArr_, voxArr_ + voxArrSize_, pcl::PointXYZ(0,0,0));
	std::fill(ptsVox_, ptsVox_ + voxArrSize_, 0);
	ptPiv_ = pcl::PointXYZ(0,0,0);
  			
  int indexVox;
  float distVox;
  int nValidPts = 0;
  float distVoxPiv = maxDist_;
  		
	for (int i=0; i<ptCloudPtr->size(); i++)
	{
		if(!is_valid(ptCloudPtr->points[i]))
		continue;
				
		nValidPts++;
  	if(!point_to_voxel(ptCloudPtr->points[i], i, indexVox, distVox))
  	continue;

  	ptsVox_[indexVox] ++;
  				
  	if(voxArr_[indexVox].z < distVoxPiv)
  	{
  		distVoxPiv = voxArr_[indexVox].z;
  		ptPiv_ = ptCloudPtr->points[i];
  	}
  }
}
		
// ***************************************************************************
bool filter_point_class::point_to_voxel(const pcl::PointXYZ& pt, int& indexPt, int& indexVox, float& distVox)
{
	distVox = pcl::euclideanDistance (pcl::PointXYZ(0,0,0), pt);
			
	if((distVox > maxDist_) || (distVox < minDist_))
	return false;
	
	//std::cout << "Detected valid points at distance: " << distVox << std::endl;
	int indexVoxX = floor(float(indexPt % imgWidth_) / float(pixInt_));
	int indexVoxY = floor(floor(float(indexPt)/float(imgWidth_)) / float(pixInt_));
	int indexVoxZ = floor((distVox - minDist_) /  distInt_);
			
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
	
	markerMsg.header.frame_id = camFrameId_;
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
		
		float shade = 0; // float(voxBeliefArr_[i])/float(voxBeliefArr_[domVoxIndx_]);
		
		std_msgs::ColorRGBA color;
		color.r = shade; 
		color.g = 0; 
		color.b = 1 - color.r; 
		color.a = 1;
		markerMsg.colors.push_back(color);
	}
	
	ptVizPub_.publish(markerMsg);
			
	geometry_msgs::PointStamped ptMsg;
	ptMsg.header.stamp = ros::Time::now();
	ptMsg.header.frame_id = camFrameId_;
	//ptMsg.point.x = domPt_.x;
	//ptMsg.point.y = domPt_.y;
	//ptMsg.point.z = domPt_.z;

	ptPub_.publish(ptMsg);
	
	geometry_msgs::PointStamped ptPivMsg;
	ptPivMsg.header.stamp = ros::Time::now();
	ptPivMsg.header.frame_id = camFrameId_;
	ptPivMsg.point.x = ptPiv_.x;
	ptPivMsg.point.y = ptPiv_.y;
	ptPivMsg.point.z = ptPiv_.z;

	ptPivPub_.publish(ptPivMsg);
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
			std::cout << voxTransModel_[i][j] << "\t";
		std::cout << std::endl;
	}

	for (int i=0; i<voxArrSize_; i++)
	{
		//std::cout << "Number of Voxel Points: " << ptsVox_[i] << std::endl;
		//std::cout << "Distance from Origin: " << distVox_[i] << std::endl;
		//std::cout << "Distance of Nearest Voxel: " << distVoxPiv_ << std::endl;
		//std::cout << "Distance from Nearest Voxel: " << abs(distVox_[i] - distVoxPiv_) << std::endl;
		//std::cout << "Observation Weight: " << obsWeight_[i] << std::endl;
		//std::cout << "Number of Particles: " << voxBeliefArr_[i] << std::endl << std::endl;
		//std::cout << "Probability of Voxel: " << float(voxBeliefArr_[i])/float(partArrSize_) << std::endl; 
	}
	std::cout << std::endl;
};

// *******************************************************************
