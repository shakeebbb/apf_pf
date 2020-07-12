#include "filter_point.h"

// ***************************************************************************
filter_point_class::filter_point_class(ros::NodeHandle* nh)
{
	// ROS Stuff
	wait_for_params(nh);
	
  imgSub_ = nh->subscribe("img_in", 1, &filter_point_class::img_cb, this);
	ptCloudSub_ = nh->subscribe("pt_cloud_in", 1, &filter_point_class::pt_cloud_cb, this);
	camInfoSub_ = nh->subscribe("cam_info_in", 1, &filter_point_class::cam_info_cb, this);
	//twistSub_ = nh->subscribe("twist_in", 1, &filter_point_class::twist_cb, this);
	//imuSub_ = nh->subscribe("imu_in", 1, &filter_point_class::imu_cb, this);
	
	isInitialized_ = 0x00;
	
	ROS_INFO("Waiting for input cloud and camera info ...");
	while( (isInitialized_ & 0x03) != 0x03 )
	ros::spinOnce();
	
	vizPub_ = nh->advertise<visualization_msgs::MarkerArray>("viz_out", 100);
	ptPub_ = nh->advertise<geometry_msgs::PointStamped>("pt_out", 100);
	ptPivPub_ = nh->advertise<geometry_msgs::PointStamped>("pt_piv_out", 100);
	actPub_ = nh->advertise<geometry_msgs::Vector3Stamped>("twist_out", 100);
	
	tfListenerPtr_ = new tf2_ros::TransformListener(tfBuffer_);
	
	ROS_INFO("Setting up voxel and observation array ...");
	discretize_image();
	ptsVox_ = new int[voxArrSize_];
	
	ROS_INFO("Setting up initial belief array ...");
	int voxBelief[voxArrSize_];
	std::fill(voxBelief, voxBelief + voxArrSize_, 1);
	
	voxBeliefDistr_ = std::discrete_distribution<int>(voxBelief, voxBelief + voxArrSize_);
	
	ROS_INFO("Setting up action array");
	discretize_actions();

	ROS_INFO("Waiting for camera to base transform ...");
	while(!tfBuffer_.canTransform (baseFrameId_, camFrameId_, ros::Time(0)))
	ROS_WARN_THROTTLE(1, "Transform unavailable: %s to %s", baseFrameId_.c_str(), camFrameId_.c_str());
	
	camToBaseTransform_ = tfBuffer_.lookupTransform (baseFrameId_, camFrameId_, ros::Time(0));
	
	fout_ = new std::fstream;
	if (!readFromFile_)
	{
		ROS_INFO("Populating transition model ..."); 
		populate_transition_model();
	
		ROS_INFO("Populating reward model ...");
		populate_reward_model();
		
		ROS_INFO("Computing alpha vectors ...");
		compute_alpha_vectors(alphaItr_);
		
		if (!write_to_file(filePath_))
			ROS_WARN("File write unsuccesful: File path -> %s", filePath_.c_str());
	}
	else
	{
		read_from_file(filePath_);
	}
	
	if (nThreads_ < 1)
		omp_set_num_threads(1);
	else
		omp_set_num_threads(nThreads_);
}

// ***************************************************************************
void filter_point_class::compute_alpha_vectors(int iterations)
{
	alphaMat_.resize(voxArrSize_, actArrSize_);
	alphaMat_.setZero();

	Eigen::MatrixXd alphaMatMax(voxArrSize_, actArrSize_);
	Eigen::MatrixXd transMat(voxArrSize_, voxArrSize_);
	
  for (int i = 0; i < voxArrSize_; i++)
		transMat.row(i) = Eigen::VectorXd::Map(&voxTransModel_[i].probabilities()[0], voxArrSize_);
		
	//std::cout << voxTransModel_[0].probabilities()[0] << "===>" << transMat(0, 0) << std::endl;
	//std::cout << voxTransModel_[0].probabilities()[4] << "===>" << transMat(0, 4) << std::endl;
	//std::cout << voxTransModel_[100].probabilities()[24] << "===>" << transMat(100, 24) << std::endl;
	//std::cout << voxTransModel_[0].probabilities()[2] << "===>" << transMat(0, 2) << std::endl;
	//std::cout << voxTransModel_[70].probabilities()[90] << "===>" << transMat(70, 90) << std::endl;
	//std::cout << voxTransModel_[43].probabilities()[5] << "===>" << transMat(43, 5) << std::endl;
	
	Eigen::VectorXd optimalAlphas(voxArrSize_);
	for (int i=0; i<iterations; i++)
	{
		optimalAlphas = alphaMat_.rowwise().maxCoeff();
		for (int j=0; j<actArrSize_; j++)
			alphaMatMax.col(j) = optimalAlphas;
		alphaMat_ = rewMat_ + 0.95 * transMat * alphaMatMax;
	}
}

// ***************************************************************************
void filter_point_class::populate_reward_model()
{
	rewMat_.resize(voxArrSize_, actArrSize_);
	
	for (int i=0; i<voxArrSize_; i++)
		for (int j=0; j<actArrSize_; j++)
		{
			if( i == (voxArrSize_-1) )
			rewMat_(i, j) = - rewQ_[0]*pow(actArr_[j][0], 2) 
											-	rewQ_[1]*pow(actArr_[j][1], 2) 
											-	rewQ_[2]*pow(actArr_[j][2], 2);
			else
			{
				//rewMat_(i, j) = repulsive_potential(point2_to_point3(voxArr_[i])) -
				rewMat_(i, j) = - rewQ_[0]*pow(actArr_[j][0], 2) 
												-	rewQ_[1]*pow(actArr_[j][1], 2) 
												-	rewQ_[2]*pow(actArr_[j][2], 2);
				
				for (int k=0; k<(lookaheadT_/deltaT_ + 1); k++)
				rewMat_(i, j) -= repulsive_potential( apply_action(i, j, k*deltaT_) );						
			}
		}
}

// ***************************************************************************
void filter_point_class::discretize_image()
{
	voxGridWidth_ = ceil(double(imgWidth_)/double(pixInt_));
	voxGridHeight_ = ceil(double(imgHeight_)/double(pixInt_));
	voxGridDepth_ = floor( (maxDist_ - minDist_)/distInt_ + 1);
	
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
			
	voxArr_[voxArrSize_-1] = pcl::PointXYZ(0, 0, 0);
}

// ***************************************************************************
void filter_point_class::discretize_actions()
{
	int horzVelSize = floor((maxAct_[0] - minAct_[0])/actInt_[0] + 1);
	int vertVelSize = floor((maxAct_[1] - minAct_[1])/actInt_[1] + 1);
	int rotVelSize = floor((maxAct_[2] - minAct_[2])/actInt_[2] + 1);
	bool zeroIsIncluded = false;
	
	actArrSize_ = horzVelSize * vertVelSize * rotVelSize + 1;
								
	//std::cout << maxAct_[0] << ", " << maxAct_[1] << ", " << maxAct_[2] << std::endl;
	//std::cout << minAct_[0] << ", " << minAct_[1] << ", " << minAct_[2] << std::endl;
	//std::cout << actInt_[0] << ", " << actInt_[1] << ", " << actInt_[2] << std::endl;
	//std::cout << "nActions: " << actArrSize_ << std::endl;
	
	actArr_ = new double*[actArrSize_];
	int count = 0;
	for (int i=0; i<horzVelSize; i++)
	{
		for (int j=0; j<vertVelSize; j++)
		{
			for (int k=0; k<rotVelSize; k++)
			{
				//std::cout << k << "==>" << k+actInt_[2] <<  "==>" << ((k+actInt_[2]) > 6.28) << std::endl;
				//getchar();
				
				actArr_[count] = new double[3];
				actArr_[count][0] = minAct_[0] + i*actInt_[0];
				actArr_[count][1] = minAct_[1] + j*actInt_[1];
				actArr_[count][2] = minAct_[2] + k*actInt_[2];
				
				if (actArr_[count][0] == 0 && actArr_[count][1] == 0 && actArr_[count][2] == 0)
				zeroIsIncluded = true;
				
				count++;
				//std::cout << i << ", " << j << ", " << k << std::endl;
			}
		}
	}
	
	//std::cout << "zeroIsIncluded: " << zeroIsIncluded << std::endl;
	if (zeroIsIncluded)
	{
		//delete actArr_[actArrSize_-1];
		actArrSize_ -= 1;
	}
	else
	{
		actArr_[actArrSize_-1] = new double[3];
		actArr_[actArrSize_-1][0] = 0;
		actArr_[actArrSize_-1][1] = 0;
		actArr_[actArrSize_-1][2] = 0;
	}
}

// ***************************************************************************
filter_point_class::~filter_point_class()
{
	delete voxTransModel_;
	
	for (int i=0; i<actArrSize_; i++)
	delete actArr_[i];
	delete actArr_;
	
	delete voxArr_;		
	delete ptsVox_;
 
  delete fout_;

  delete tfListenerPtr_;

}

// ***************************************************************************
void filter_point_class::wait_for_params(ros::NodeHandle *nh)
{
	while(!nh->getParam("distance_interval", distInt_));
	while(!nh->getParam("pixel_interval", pixInt_));
	while(!nh->getParam("min_distance", minDist_));
	while(!nh->getParam("max_distance", maxDist_));
	
	while(!nh->getParam("n_particles_vox", voxPartArrSize_));
	while(!nh->getParam("outliers_per_voxel", nOutliers_));
	
	while(!nh->getParam("trans_noise_sdev", sdevTrans_));
	while(!nh->getParam("obsv_noise_sdev", sdevObsv_));
	
	while(!nh->getParam("min_action_values", minAct_));
	while(!nh->getParam("max_action_values", maxAct_));
	while(!nh->getParam("action_intervals", actInt_));
	
	while(!nh->getParam("reward_Q", rewQ_));
	while(!nh->getParam("repulsive_potential_max_distance", repPotMaxDist_));
	while(!nh->getParam("repulsive_potential_gain", repPotGain_));
	
	while(!nh->getParam("alpha_vector_iterations", alphaItr_));
	
	while(!nh->getParam("lookahead_time", lookaheadT_));
	while(!nh->getParam("sampling_time", deltaT_));
	while(!nh->getParam("base_frame_id", baseFrameId_));
	
	while(!nh->getParam("matrices_file_path", filePath_));
	while(!nh->getParam("read_from_file", readFromFile_));
	
	while(!nh->getParam("n_threads", nThreads_));

  while(!nh->getParam("input_mode", inputMode_));
	
	ROS_INFO("Parameters for filter_point retreived from the parameter server");
}

// ***************************************************************************
void filter_point_class::img_cb(const sensor_msgs::Image::ConstPtr& msgPtr)
{
  if (!inputMode_)
    return;

	ros::Time begin = ros::Time::now();
	
	if((isInitialized_ & 0x03) == 0x03)
	{
    cv_bridge::CvImageConstPtr cvPtr;
    try
    {
      cvPtr = cv_bridge::toCvShare(msgPtr);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
		extract_features<cv_bridge::CvImageConstPtr>(cvPtr, imgWidth_, imgHeight_);
		update_belief();
		int actIndx = update_action();
		publish_action(actIndx);
		publish_viz("all", actIndx);
		display("beliefs", 3);
	}
	
	if((isInitialized_ & 0x01) != 0x01)
	{
		ROS_INFO("Frame Id: %s", msgPtr->header.frame_id.c_str());
		ROS_INFO("Image Height: %i", msgPtr->height);
		ROS_INFO("Image Width: %i", msgPtr->width);
		ROS_INFO("Data Size: %lu", msgPtr->data.size());
		
		camFrameId_ = msgPtr->header.frame_id;
		imgWidth_ = msgPtr->width;
		imgHeight_ = msgPtr->height;
		
		isInitialized_ |= 0x01;
	}
	
	ros::Duration elapsed = ros::Time::now() - begin;
	ROS_INFO("Time Elapsed: %f", elapsed.toSec());
}

// ***************************************************************************
void filter_point_class::pt_cloud_cb(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msgPtr)
{
  if (inputMode_)
    return;

	ros::Time begin = ros::Time::now();
	
	if(msgPtr->height == 1)
	{
		ROS_WARN("Input point cloud is unordered");
		return;
	}
	
	if((isInitialized_ & 0x03) == 0x03)
	{
		extract_features<pcl::PointCloud<pcl::PointXYZ>::ConstPtr>(msgPtr, imgWidth_, imgHeight_);
		update_belief();
		int actIndx = update_action();
		publish_action(actIndx);
		publish_viz("all", actIndx);
		display("beliefs", 3);
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
		//std::cout << "Matrix P ";
		
		for(int i=0; i<9; i++)
		{
			camInfoP_[i] = msgPtr->P[i];
			
			//std::cout << camInfoP_[i] << ", ";
		}
		
		std::cout << std::endl;
		
		isInitialized_ |= 0x02;
	}
	
	//std::cout << "cam_info" << std::endl;
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
pcl::PointXYZ filter_point_class::apply_action(int indxVoxIn, int indxAct, double deltaT, bool isHolonomic)
{
	if(indxVoxIn == (voxArrSize_ - 1))
	return point2_to_point3(voxArr_[indxVoxIn], true);
	
	//std::cout << "2.5D point (cam frame) " << voxArr_[indxVoxIn] << std::endl;
	
	pcl::PointXYZ point3 = point2_to_point3(voxArr_[indxVoxIn], true);
	
	//std::cout << "3D point (cam frame)" << point3 << std::endl;

	tf2::Transform cam2base( tf2::Quaternion(camToBaseTransform_.transform.rotation.x, camToBaseTransform_.transform.rotation.y,
														  						 camToBaseTransform_.transform.rotation.z, camToBaseTransform_.transform.rotation.w), 
													 tf2::Vector3(camToBaseTransform_.transform.translation.x,
																			  camToBaseTransform_.transform.translation.y,
																			  camToBaseTransform_.transform.translation.z) ); 
	
	tf2::Quaternion basePRot;
	tf2::Vector3 basePTrans;
	
	switch(isHolonomic)
	{
	case true:
		basePTrans.setX( actArr_[indxAct][0] * deltaT );
		basePTrans.setY( actArr_[indxAct][1] * deltaT );
		basePTrans.setZ( actArr_[indxAct][2] * deltaT );		

		basePRot.setRPY(0, 0, 0);
		break;	
				
	default:
		if (actArr_[indxAct][2] == 0)
		{
			basePTrans.setX( actArr_[indxAct][0] * deltaT );
			basePTrans.setY( 0 );
			basePTrans.setZ( actArr_[indxAct][1] * deltaT );		
		}
		else
		{
			basePTrans.setX( (actArr_[indxAct][0]/actArr_[indxAct][2]) * sin(actArr_[indxAct][2]*deltaT) );
			basePTrans.setY( (actArr_[indxAct][0]/actArr_[indxAct][2]) * (1 - cos(actArr_[indxAct][2]*deltaT)) );
			basePTrans.setZ( actArr_[indxAct][1] * deltaT );
		}
		basePRot.setRPY(0, 0, actArr_[indxAct][2]*deltaT);
	}
	tf2::Transform baseP2base( basePRot, basePTrans );
	
	/*
	tf2::Vector3 pt3Base = cam2base * tf2::Vector3(point3.x, point3.y, point3.z);
	tf2::Vector3 pt3BaseP = baseP2base.inverse() * pt3Base;
	tf2::Vector3 pt3CamP = cam2base.inverse() * pt3BaseP;
	
	std::cout << "3D point (base frame) " << pt3Base.x() << ", " << pt3Base.y() << ", " << pt3Base.z() << std::endl;
	std::cout << "Action (base frame) " << actArr_[indxAct][0] << ", " <<actArr_[indxAct][1] << ", " << actArr_[indxAct][2] << std::endl;
	std::cout << "3D point (new base frame) " << pt3BaseP.x() << ", " << pt3BaseP.y() << ", " << pt3BaseP.z() << std::endl;
	
	std::cout << "New base frame origin " << baseP2base.getOrigin().x() << ", " << baseP2base.getOrigin().y() << ", " << baseP2base.getOrigin().z() << std::endl;
	
	std::cout << "New base frame orientation " << 0 << ", " << 0 << ", " << actArr_[indxAct][2]*deltaT << std::endl;
	
	std::cout << "3D point (new camera frame) " << pt3CamP.x() << ", " << pt3CamP.y() << ", " << pt3CamP.z() << std::endl;
	
	std::cout << std::endl << std::endl;
	*/
	tf2::Vector3 point3Tf = cam2base.inverse() * baseP2base.inverse() * cam2base * tf2::Vector3(point3.x, point3.y, point3.z);
	
	return pcl::PointXYZ(point3Tf.x(), point3Tf.y(), point3Tf.z());
}

// ***************************************************************************
void filter_point_class::populate_transition_model()
{
	voxTransModel_ = new std::discrete_distribution<int>[voxArrSize_];
	
	for (int i=0; i<voxArrSize_; i++)
	{
		double transWeights[voxArrSize_];
			
		for (int j=0; j<voxArrSize_; j++)
		transWeights[j] = norm_pdf( 0, sdevTrans_[0], man_dist_vox(i, j), false) *
											norm_pdf( 0, sdevTrans_[1], voxArr_[j].z, false);
			
		voxTransModel_[i] = std::discrete_distribution<int>(transWeights, transWeights+voxArrSize_);
	}
}

// ***************************************************************************
int filter_point_class::update_action()
{
	Eigen::VectorXd beliefMat;
	
	beliefMat = Eigen::VectorXd::Map(&voxBeliefDistr_.probabilities()[0], voxArrSize_);
	
	Eigen::VectorXd value = alphaMat_.transpose() * beliefMat;
	
	int actIndx;
	value.maxCoeff(&actIndx);
	
	//std::cout << value.maxCoeff(&actIndx) << std::endl;
	return actIndx;
}

// ***************************************************************************
void filter_point_class::update_belief() 
{
	//------ Belief can be zero to 1: better solution?
	// --------voxelDist = discrete_distribution<int>(obsWeight, obsWeight+nVoxels);
	//mt19937 generator;
	//generator.seed(seed);
	
	double voxObsWeight[voxArrSize_];
	int maxPts = 0;
	
	//voxObsWeight[voxArrSize_-1] = 1;
	
	//std::cout << "Observation Weight: " << "[";
	
	#pragma omp parallel for
	for (int i=0; i<(voxArrSize_-1); i++)
  { 
  	//std::cout << "Maximum Points: " <<  pixInt_ * pixInt_ << std::endl;
  	//std::cout << "Voxel Points: " << ptsVox_[i] << std::endl;
  		
  	voxObsWeight[i] = norm_pdf(0, sdevObsv_[0], (pixInt_ * pixInt_) - ptsVox_[i], false);
  	//std::cout << "Voxel Probability: " << voxObsWeight[i] << std::endl;
  	
  	#pragma omp critical(max_pts_update)
  	if(ptsVox_[i] > maxPts)
  		maxPts = ptsVox_[i];
  	
  	//std::cout << voxObsWeight[i] << ", ";
  }
  
  voxObsWeight[voxArrSize_-1] = norm_pdf(0, sdevObsv_[1], maxPts, false);
  //std::cout << voxObsWeight[voxArrSize_-1] << "]" << std::endl;
  //std::cout << "Boundary voxel Probability: " << voxObsWeight[voxArrSize_-1] << std::endl;
  //voxObsWeight[voxArrSize_-1] = norm_pdf(0, sdevObsv_[0], (pixInt_ * pixInt_) - nOutliers_, false);
	
	voxBeliefDistr_ = particle_filter(voxBeliefDistr_, voxTransModel_, voxObsWeight, voxPartArrSize_);	
}

// ***************************************************************************
std::discrete_distribution<int> 
filter_point_class::particle_filter(std::discrete_distribution<int>& initBeliefDist, 
																		std::discrete_distribution<int>* transModel,
																		double *obsWeight, int nParts)
{		
	double partWeight[nParts];
	int partState[nParts];
	
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator(seed);
			
	#pragma omp parallel for
	for (int i=0; i<nParts; i++)
	{
		int sampledState = initBeliefDist(generator);
		int nextState = transModel[sampledState](generator);
				
		partState[i] = nextState;
		partWeight[i] = obsWeight[nextState];
		
		//std::cout << partState[i] << ", " << partWeight[i] << std::endl;
	}
	
	std::discrete_distribution<int> partDistr(partWeight, partWeight+nParts);

	int updatedBelief[initBeliefDist.probabilities().size()] = { 0 };
	
	#pragma omp parallel for		
	for (int i=0; i<nParts; i++)
	{
		int sampledPart = partDistr(generator);
		
		#pragma omp atomic
		updatedBelief[partState[sampledPart]]++;
	}
	
	return std::discrete_distribution<int>(updatedBelief, updatedBelief+initBeliefDist.probabilities().size());
}
		
// ***************************************************************************
template<typename T>
void filter_point_class::extract_features(const T& imgPtr, const int imgWidth, const int imgHeight)
{
	//std::fill(voxArr_, voxArr_ + voxArrSize_, pcl::PointXYZ(0,0,0));
	std::fill(ptsVox_, ptsVox_ + voxArrSize_, 0);
	ptPiv_ = pcl::PointXYZ(0,0,0);
  
  double distVoxPiv = maxDist_;
  
  #pragma omp parallel for		
	for (int j=0; j<imgHeight; j++)
	{
    for (int i=0; i<imgWidth; i++)
	  {
		  if(!is_valid(imgPtr, i, j))
			continue;
			
			double distVox = get_pix_val(imgPtr, i, j);

      if((distVox > maxDist_) || (distVox < minDist_))
  	   continue;

			int indexVox = point2_to_voxel_indx(pcl::PointXYZ(i, j, distVox));

			#pragma omp atomic
			ptsVox_[indexVox] ++;
			
			#pragma omp critical(piv_pt_update)
			if(distVox < distVoxPiv)
			{
				distVoxPiv = distVox;
        ptPiv_ = point2_to_point3(pcl::PointXYZ(i, j, distVox));
			}
    }
  }
}

// *******************************************************************
