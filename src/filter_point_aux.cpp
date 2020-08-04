#include "filter_point.h"

// ***************************************************************************
bool filter_point_class::write_to_file(std::string filePath)
{
	Eigen::MatrixXd transMat(voxArrSize_, voxArrSize_);
  for (int i = 0; i < voxArrSize_; i++)
		transMat.row(i) = Eigen::VectorXd::Map(&voxTransModel_[i].probabilities()[0], voxArrSize_);
		
	std::string ns = ros::this_node::getNamespace();
	
	if ( !write_csv(filePath + ns + "trans" + ".csv", transMat) )
		return false;
	
	if ( !write_csv(filePath + ns + "rew" + ".csv", rewMat_) )
		return false;
	
	if ( !write_csv(filePath + ns + "alpha" + ".csv", alphaMat_) )
		return false;

	return true;
}

// ***************************************************************************
bool filter_point_class::read_from_file(std::string filePath)
{
	std::string ns = ros::this_node::getNamespace();
	
	voxTransModel_ = new std::discrete_distribution<int>[voxArrSize_];
	if ( !read_csv(filePath + ns + "trans" + ".csv", voxTransModel_, voxArrSize_, voxArrSize_) )
		return false;
	
	rewMat_.resize(voxArrSize_, actArrSize_);
	if ( !read_csv(filePath + ns + "rew" + ".csv", rewMat_, voxArrSize_, actArrSize_) )
		return false;
	
	alphaMat_.resize(voxArrSize_, actArrSize_);	
	if ( !read_csv(filePath + ns + "alpha" + ".csv", alphaMat_, voxArrSize_, actArrSize_) )
		return false;
}

// ***************************************************************************
bool filter_point_class::read_csv(std::string file, Eigen::MatrixXd& matOut, int nRows, int nCols)
{
	fout_->open(file, std::fstream::in);
	if (!fout_->is_open())
		return false;
		
	std::string cell;	
	for (int i=0; i<nRows; i++)
		for (int j=0; j<nCols; j++)
		{
			std::getline(*fout_, cell, ',');
			matOut(i,j) = stod(cell);
		}
	
	fout_->close();	
	return true;
}

// ***************************************************************************
bool filter_point_class::read_csv(std::string file, std::discrete_distribution<int>* matOut, int nRows, int nCols)
{
	fout_->open(file, std::fstream::in);
	if (!fout_->is_open())
		return false;
	
	std::vector<double> line(nCols);	
	std::string cell;	
	
	for (int i=0; i<nRows; i++)
	{
		for (int j=0; j<nCols; j++)
		{
			std::getline(*fout_, cell, ',');
			line[j] = stod(cell);
		}
		matOut[i] = std::discrete_distribution<int>(line.begin(), line.end());
	}
	
	fout_->close();	
	return true;
}

// ***************************************************************************
bool filter_point_class::write_csv(std::string file, Eigen::MatrixXd matIn)
{
	fout_->open(file, std::fstream::out | std::fstream::trunc);
	if (!fout_->is_open())
		return false;
		
	const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");
	*fout_ << matIn.format(CSVFormat);
	fout_->close();
	return true;
}

// ***************************************************************************
double filter_point_class::repulsive_potential(pcl::PointXYZ x_xobs)
{
	double dist = pcl::euclideanDistance (pcl::PointXYZ(0,0,0), x_xobs);
	
	if (dist >= repPotMaxDist_)
	return 0;
	
	return 0.5*repPotGain_*pow(1/dist - 1/repPotMaxDist_, 2);
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
int filter_point_class::point2_to_voxel(pcl::PointXYZ pointIn)
{
	pcl::PointXYZ pointOut;
	
	int xIndx = floor(double(pointIn.x) / double(pixInt_));
	int yIndx = floor(double(pointIn.y) / double(pixInt_));
	int zIndx = floor((pointIn.z - minDist_) / distInt_);
	
	return xIndx + voxGridWidth_*(yIndx + voxGridHeight_*zIndx);
}

// ***************************************************************************
pcl::PointXYZ filter_point_class::point2_to_point3(pcl::PointXYZ pointIn, bool direction)
{
	pcl::PointXYZ pointOut;
	
	double fx = camInfoP_[0];
	double cx = camInfoP_[2];
	double fy = camInfoP_[5];
	double cy = camInfoP_[6];
	
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
double filter_point_class::man_dist_vox(int indxVox1, int indxVox2)
{
	if( indxVox1 == (voxArrSize_ - 1) )
	return man_dist_to_bound(indxVox2);
	
	if( indxVox2 == (voxArrSize_ - 1) )
	return man_dist_to_bound(indxVox1);
	
	return abs( (voxArr_[indxVox2].x - voxArr_[indxVox1].x) / pixInt_ ) + 
				 abs( (voxArr_[indxVox2].y - voxArr_[indxVox1].y) /pixInt_ ) + 
				 abs( (voxArr_[indxVox2].z - voxArr_[indxVox1].z) /distInt_ );
}

// ***************************************************************************
double filter_point_class::man_dist_to_bound(int indxVox)
{	
	double dist[5];
	
	// right, left, front, upper, lower
	
	dist[0] = abs( voxArr_[indxVox].x / pixInt_ - voxGridWidth_ ); 
	dist[1] = abs( voxArr_[indxVox].x / pixInt_ + 1 );
	dist[2] = abs( (voxArr_[indxVox].z - minDist_) / distInt_ - voxGridDepth_ );
	dist[3] = abs( voxArr_[indxVox].y / pixInt_ + 1 );
	dist[4] = abs( voxArr_[indxVox].y / pixInt_ - voxGridHeight_); 
	
	return *std::min_element(dist, dist+5);
}

// ***************************************************************************
pcl::PointXYZ filter_point_class::indx_to_vox(int indxVox)
{
	pcl::PointXYZ outVox;
	
	outVox.z = floor(double(indxVox) / double(voxGridWidth_*voxGridHeight_));
	outVox.y = floor(double(indxVox - outVox.z*voxGridWidth_*voxGridHeight_) / double(voxGridWidth_));
	outVox.x = indxVox - outVox.z*voxGridWidth_*voxGridHeight_ - outVox.y*voxGridWidth_;
	
	return outVox;
}

// ***************************************************************************
double filter_point_class::norm_pdf(double mean, double sdev, double xIn, bool isFull)
{
    static const double inv_sqrt_2pi = 0.3989422804014327;
    double a = (xIn - mean) / sdev;
    
    //if( isFull || (xIn == mean) ) 
    if(isFull)
		return inv_sqrt_2pi / sdev * std::exp(-0.5f * a * a);
		else
		return 2 * inv_sqrt_2pi / sdev * std::exp(-0.5f * a * a);
}


// ***************************************************************************
int filter_point_class::random_index(double* belief, int& size)
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
bool filter_point_class::point_to_voxel(const pcl::PointXYZ& pt, int& indexPt, int& indexVox, double& distVox)
{
	//distVox = pcl::euclideanDistance (pcl::PointXYZ(0,0,0), pt);
	
	distVox = pt.z;
	
	if((distVox > maxDist_) || (distVox < minDist_))
	return false;
	
	//std::cout << "Detected valid points at distance: " << distVox << std::endl;
	int indexVoxX = floor(double(indexPt % imgWidth_) / double(pixInt_));
	int indexVoxY = floor(floor(double(indexPt)/double(imgWidth_)) / double(pixInt_));
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
void filter_point_class::publish_action(int actIndx)
{
	geometry_msgs::Vector3Stamped actMsg;
	
	actMsg.header.stamp = ros::Time::now();
	actMsg.header.frame_id = baseFrameId_;

	actMsg.vector.x = actArr_[actIndx][0];
	actMsg.vector.y = actArr_[actIndx][1];
	actMsg.vector.z = actArr_[actIndx][2];
	
	actPub_.publish(actMsg);
}

// ***************************************************************************
void filter_point_class::publish_points()
{
  std::vector<double> probVec;
	probVec = voxBeliefDistr_.probabilities();
		
	std::vector<double>::iterator domItr = std::max_element(probVec.begin(), probVec.end());
	int domIndx = domItr - probVec.begin();
	double domProb = *domItr;
		
	geometry_msgs::PointStamped ptMsg;
	ptMsg.header.stamp = ros::Time::now();
	ptMsg.header.frame_id = camFrameId_;
		
	pcl::PointXYZ pt3 = point2_to_point3(voxArr_[domIndx], true);

	ptMsg.point.x = pt3.x;
	ptMsg.point.y = pt3.y;
	ptMsg.point.z = pt3.z;

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
void filter_point_class::publish_force(double* repForce)
{
  geometry_msgs::Vector3Stamped forceMsg;

  forceMsg.header.stamp = ros::Time::now();
	forceMsg.header.frame_id = baseFrameId_;

  forceMsg.vector.x = 0; forceMsg.vector.y = 0; forceMsg.vector.z = 0;
  for(int i=0; i<(voxArrSize_-1); i++)
  {
    if(voxBeliefDistr_.probabilities()[i] > 0.0)
    {
      geometry_msgs::Vector3 repForceVox = get_rep_force(i, repPotMaxDist_, repPotGain_, voxBeliefDistr_.probabilities()[i]);
      forceMsg.vector.x += repForceVox.x;
      forceMsg.vector.y += repForceVox.y;
      forceMsg.vector.z += repForceVox.z;
    }
  }

  tf2::doTransform(forceMsg, forceMsg, camToBaseTransform_);
  repForce[0] = forceMsg.vector.x;
  repForce[1] = forceMsg.vector.y;
  repForce[2] = forceMsg.vector.z;

  forcePub_.publish(forceMsg);
}

// ***************************************************************************
geometry_msgs::Vector3 filter_point_class::get_rep_force(int indxVox, double horizon, double gain, double scale)
{
  geometry_msgs::Vector3 repVec;
  
  pcl::PointXYZ pt3 = point2_to_point3(voxArr_[indxVox], true);
  double obsDistSq = pow(pt3.x,2) + pow(pt3.y,2) + pow(pt3.z,2);
  double obsDist = sqrt(obsDistSq);

  double repVecMag = gain * (1/obsDist - 1/horizon) * (1/obsDistSq);

  repVec.x = -pt3.x / obsDist * repVecMag * scale;
  repVec.y = -pt3.y / obsDist * repVecMag * scale;
  repVec.z = -pt3.z / obsDist * repVecMag * scale;
  
  return repVec;
}
		
// ***************************************************************************
void filter_point_class::publish_viz(double* repAct, std::string field)
{
	visualization_msgs::MarkerArray markerArrMsg;
	
	if (field == "all" || field == "voxels")
	{
		visualization_msgs::Marker markerMsg;
	
		markerMsg.header.stamp = ros::Time::now();
		markerMsg.action = visualization_msgs::Marker::ADD;
		markerMsg.lifetime = ros::Duration(0);
		markerMsg.frame_locked = true;	
		
		markerMsg.ns = "filter_point_voxels";
		markerMsg.header.frame_id = camFrameId_;
		markerMsg.type = visualization_msgs::Marker::CUBE_LIST;
		
		geometry_msgs::Pose geoPose;
		geoPose.orientation.w = 1;
		markerMsg.pose = geoPose;
		
		geometry_msgs::Vector3 scale;
		scale.x = 0.05;
		scale.y = 0.05;
		scale.z = 0.05;
		markerMsg.scale = scale;
		
		for (int i=0; i<voxArrSize_; i++)
		{
			markerMsg.id = i;
			
			geometry_msgs::Point pt;
			pcl::PointXYZ pt3 = point2_to_point3(voxArr_[i], true);
			pt.x = pt3.x;
			pt.y = pt3.y;
			pt.z = pt3.z;
			markerMsg.points.push_back(pt);
			
			double shade;
			if(voxBeliefDistr_.probabilities()[i] == 0)
			shade = 0;
			else
			shade = 1;
			
			std_msgs::ColorRGBA color;
			color.r = shade; 
			color.g = 0; 
			color.b = 1 - color.r; 
			color.a = 1;
			markerMsg.colors.push_back(color);
		}
		
		markerArrMsg.markers.push_back(markerMsg);
	}
	
	if (field == "all" || field == "actions")
	{
		visualization_msgs::Marker markerMsg;
	
		markerMsg.header.stamp = ros::Time::now();
		markerMsg.action = visualization_msgs::Marker::ADD;
		markerMsg.lifetime = ros::Duration(0);
		markerMsg.frame_locked = true;	
		
		markerMsg.ns = "filter_point_actions";
		markerMsg.header.frame_id = baseFrameId_;
		markerMsg.type = visualization_msgs::Marker::ARROW;
		
		geometry_msgs::Pose geoPose;
		geoPose.orientation.w = 1;
		markerMsg.pose = geoPose;
		
		markerMsg.id = 0;
		
		geometry_msgs::Point geoPt;
		geoPt.x = 0; geoPt.y = 0; geoPt.z = 0;
		markerMsg.points.push_back(geoPt);
		
		geoPt.x = repAct[0];
		geoPt.y = repAct[1];
		geoPt.z = repAct[2];
		markerMsg.points.push_back(geoPt);

		markerMsg.color.r = 1; 
		markerMsg.color.g = 0; 
		markerMsg.color.b = 0; 
		markerMsg.color.a = 1;
		
		markerMsg.scale.x = 0.03;
		markerMsg.scale.y = 0.06;
		markerMsg.scale.z = 0.09;	
		
		markerArrMsg.markers.push_back(markerMsg);
	}
	
	if (markerArrMsg.markers.size() > 0)
	vizPub_.publish(markerArrMsg);

  if (field == "all" || field == "belief")
  {
    std_msgs::Float64MultiArray belief;

    std_msgs::MultiArrayDimension dim;
    dim.label = "length";
    dim.size = voxBeliefDistr_.probabilities().size();
    dim.stride = dim.size;

    belief.layout.dim.push_back(dim);
    belief.data = voxBeliefDistr_.probabilities();

    beliefPub_.publish(belief);
  }
}

// ***************************************************************************
void filter_point_class::publish_compute_time(ros::Time& tic)
{
  ros::Duration elapsed = ros::Time::now() - tic;
  std_msgs::Float32 elapsedSec; 
  elapsedSec.data = elapsed.toSec(); 

	//ROS_INFO("Time Elapsed: %f", elapsedSec.data);
  computeTimePub_.publish(elapsedSec);
}
		
// ***************************************************************************
void filter_point_class::display(std::string field, int precision)
{
	std::cout << std::setprecision(precision) << std::endl;
	std::cout << "<===============================================>" << std::endl; 
	std::cout << "Number of possible states: " << voxArrSize_ << std::endl;
	std::cout << "Number of possible actions: " << actArrSize_ << std::endl;
	
	if(field == "all" || field == "states")
	{
		std::cout << "All possible states: [";
		for (int i=0; i<(voxArrSize_-1); i++)
			std::cout << voxArr_[i] << ", ";
			
		std::cout << voxArr_[voxArrSize_-1] << "]" << std::endl;
		std::cout << "Transition Model: " << std::endl;
		for (int i=0; i<voxArrSize_; i++)
		{
			for (int j=0; j<voxArrSize_; j++)
				std::cout << voxTransModel_[i].probabilities()[j] << "\t";
			std::cout << std::endl << std::endl;
		}
	}

	if(field == "all" || field == "observations")
	{
		std::cout << "Number of Voxel Points: [";
		for (int i=0; i<(voxArrSize_-1); i++)
			std::cout << ptsVox_[i] << ", ";

		std::cout << ptsVox_[voxArrSize_-1] << "]" << std::endl;
	}
	
	if(field == "all" || field == "beliefs")
	{
		std::cout << "Belief Vector: [";
		for (int i=0; i<(voxArrSize_-1); i++)
			std::cout << voxBeliefDistr_.probabilities()[i] << ", ";
		std::cout << voxBeliefDistr_.probabilities()[voxArrSize_-1] << "]" << std::endl;
	}
	
	if(field == "all" || field == "rewards")
	{
		std::cout << "Reward Model: " << std::endl;
		for (int i=0; i<voxArrSize_; i++)
		{
			std::cout << point2_to_point3(voxArr_[i]) << "==>" << apply_action(i, 0, lookaheadT_) << std::endl;
			
			for (int j=0; j<actArrSize_; j++)
				std::cout << rewMat_(i, j) << "\t";
			std::cout << std::endl << std::endl;
		}
	}
	
	if(field == "all" || field == "actions")
	{
		std::cout << "Possible actions" << std::endl;
		for (int i=0; i<actArrSize_; i++)
			std::cout << i << "==> [" << actArr_[i][0] << ", " << actArr_[i][1] << ", " << actArr_[i][2] << "]" << std::endl;
	}
	
	if(field == "all" || field == "alphas")
	{
		std::cout << "Alpha Vectors" << std::endl;
		for (int i=0; i<voxArrSize_; i++)
		{
			for (int j=0; j<actArrSize_; j++)
				std::cout << alphaMat_(i, j) << "\t";
			std::cout << std::endl << std::endl;
		}
	}
	std::cout << std::endl << std::endl;
};

// *******************************************************************
