
#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// ------------------------------------------------------



// ------------------------------------------------------
class goal_to_vel_class
{

private:
	ros::NodeHandle* nh_;
	
	// Publishers, Subscribers, Transforms, Timers
	ros::Publisher twistOutPub_;
	
	ros::Subscriber goalSub_;
	ros::Subscriber repSub_;
	
	ros::Subscriber poseSub_;
	
	ros::ServiceClient flModClient_;
	
	tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener* tfListenerPtr_;
  
  ros::Timer timer;
	
	// Params
	std::vector<float> maxVel_; //x,z,yaw
	std::string worldFrameId_;
  std::string baseFrameId_;
	float pubRate_;
	bool isHolonomic_;
	float foreVelDeadband_;
	float timeOut_;
	float successRadius_;
  float attPotParaBound_;
  float attGain_;
  std::string mode_;
  bool display_;
  bool worldFrameOut_;
	
	// Local Variables
	geometry_msgs::Point goalPoint_;
	geometry_msgs::Vector3 repVec_;
	geometry_msgs::Pose pose_;
	
	uint8_t isInitialized_;
	bool startCount_;
	int timeElapsedLastRepVec_;
	
public:

	// ************************************************
	goal_to_vel_class(ros::NodeHandle* nh)
	{
		nh_ = nh;

		while( !nh->getParam("rep_vec_timeout_secs", timeOut_) );
		while( !nh->getParam("process_rate_secs", pubRate_) );
		while( !nh->getParam("att_vel", maxVel_) );
		while( !nh->getParam("success_radius", successRadius_) );
		while( !nh->getParam("holonomic", isHolonomic_) );
		while( !nh->getParam("yaw_err_bound_nonzero_fore_vel_in_rad", foreVelDeadband_) );
		while( !nh->getParam("pose_frame_id", worldFrameId_) );
    while( !nh->getParam("base_frame_id", baseFrameId_) );
    while( !nh->getParam("parabolic_attractor_bound", attPotParaBound_) ); // only in apf mode
    while( !nh->getParam("attractor_gain", attGain_) ); // only in apf mode
    while( !nh->getParam("mode", mode_) ); // qmdp, apf
    while( !nh->getParam("display_cout", display_) );
    while( !nh->getParam("pose_frame_output", worldFrameOut_) );
		
		ROS_INFO("%s: Parameters retrieved from parameter server", nh->getNamespace().c_str());
		
		twistOutPub_ = nh->advertise<geometry_msgs::TwistStamped>("twist_out", 10);
		
		goalSub_ = nh->subscribe("goal_pt_in", 1, &goal_to_vel_class::goal_cb, this);
		repSub_ = nh->subscribe("rep_vec_in", 1, &goal_to_vel_class::rep_cb, this);
		
		poseSub_ = nh->subscribe("pose_in", 1, &goal_to_vel_class::pose_cb, this);
		
		tfListenerPtr_ = new tf2_ros::TransformListener(tfBuffer_);
		
		isInitialized_ = 0x00;
		ROS_INFO("%s: Waiting for input goal and first pose message ...", nh_->getNamespace().c_str());
		while( (isInitialized_ & 0x03) != 0x03 )
			ros::spinOnce();
		
		timer = nh->createTimer(ros::Duration(pubRate_), &goal_to_vel_class::timer_cb, this);		
		
		clear_msg(repVec_);
		startCount_ = true;
		timeElapsedLastRepVec_ = 0;
	}

	// ************************************************
	void pose_cb(const geometry_msgs::PoseStamped& msg)
	{
		if( (isInitialized_ & 0x01) != 0x01 )
			isInitialized_ |= 0x01;
		
		pose_ = msg.pose;
	}
	
	// ************************************************
	geometry_msgs::Vector3 transform_goal_to_base_flat(geometry_msgs::Point goalIn, geometry_msgs::Pose poseIn, std::string mode)
	{
		tf2::Transform baseFlatToWorld = base_flat_to_world(poseIn);
		
    if(display_)
    {
			std::cout << "Base flat to world transform: " << tf2::toMsg(baseFlatToWorld).translation.x << ", " 
																										<< tf2::toMsg(baseFlatToWorld).translation.y << ", " 
																										<< tf2::toMsg(baseFlatToWorld).translation.z << ", " 
																										<< tf2::toMsg(baseFlatToWorld).rotation.x << ", " 
																										<< tf2::toMsg(baseFlatToWorld).rotation.y << ", " 
																										<< tf2::toMsg(baseFlatToWorld).rotation.z << ", " 
																										<< tf2::toMsg(baseFlatToWorld).rotation.w << std::endl;
    }
		
		
		tf2::Vector3 goalInBaseFlat = baseFlatToWorld.inverse() * tf2::Vector3(goalIn.x, goalIn.y, goalIn.z);
		
		geometry_msgs::Vector3 attVec = toMsg(goalInBaseFlat);

    if(mode == "qmdp")
    {
      attVec = get_unit_vector(attVec);
			attVec.x *= maxVel_[0];
			attVec.y *= maxVel_[0];
			attVec.z *= maxVel_[1];
    }
    else if(mode == "apf")
    {
      if(get_magnitude(attVec) > attPotParaBound_) // conical attractive field
      {
        attVec = get_unit_vector(attVec);
        attVec.x *= (attPotParaBound_*attGain_);
        attVec.y *= (attPotParaBound_*attGain_);
        attVec.z *= (attPotParaBound_*attGain_);
      }
      else // parabolic attractive field
      {
        attVec.x *= attGain_;
        attVec.y *= attGain_;
        attVec.z *= attGain_;
      }
    } 
		
		return attVec;
	}
	
	// *************************************************
	geometry_msgs::Vector3 transform_rep_vec_to_base_flat(geometry_msgs::Vector3 repVecIn, geometry_msgs::Pose poseIn)
	{
		tf2::Transform baseFlatToWorld = base_flat_to_world(poseIn);
		
		tf2::Vector3 repVecTf = baseFlatToWorld.inverse().getBasis() * tf2::Vector3(repVecIn.x, repVecIn.y, repVecIn.z);

		return tf2::toMsg(repVecTf);
	}
	
	// ************************************************
	void goal_cb(const geometry_msgs::PointStamped& msg)
	{
		try
		{
			geometry_msgs::TransformStamped transform = tfBuffer_.lookupTransform(worldFrameId_, msg.header.frame_id, ros::Time(0)); 
			
			geometry_msgs::PointStamped msgWorld;
			
			tf2::doTransform(msg, msgWorld, transform); 
			
			goalPoint_ = msgWorld.point;
			
			if( (isInitialized_ & 0x02) != 0x02 )
			isInitialized_ |= 0x02;
    }
    catch (tf2::TransformException &ex)
    { 
      ROS_WARN_THROTTLE(1, "%s",ex.what());
      goalPoint_.x = 0;
      goalPoint_.y = 0;
      goalPoint_.z = 0;
    }
    
    if(display_)
      std::cout << "Received Goal Point: " << goalPoint_.x << ", " << goalPoint_.y << "," << goalPoint_.z << std::endl;
	}
	
	// ************************************************
	void rep_cb(const geometry_msgs::Vector3Stamped& msg)
	{		
		geometry_msgs::Vector3Stamped msgBase;
		try
		{
			geometry_msgs::TransformStamped transform = tfBuffer_.lookupTransform(worldFrameId_, msg.header.frame_id, ros::Time(0)); 
			
			tf2::doTransform(msg, msgBase, transform);    	
    }
    catch (tf2::TransformException &ex)
    { 
      ROS_WARN_THROTTLE(1, "%s",ex.what());
      return;
    }
		
		if(startCount_)
		{
			clear_msg(repVec_);
			startCount_ = false;
		}
		
		if (get_magnitude(msgBase.vector) > get_magnitude(repVec_))
			repVec_ = msgBase.vector;
		
    if(display_)	
		  std::cout << "Received Rep Vector: " << repVec_.x << ", " << repVec_.y << "," << repVec_.z << std::endl;
	}
	
	// ************************************************
	void timer_cb(const ros::TimerEvent&)
	{
		if (!isInitialized_)
		{
			ROS_WARN_THROTTLE(1, "%s : Waiting for first lookahead point ...", nh_->getNamespace().c_str());
			return;
		}
		
		static float timeElapsedLastRepVec = 0;
		
		if(startCount_)
			timeElapsedLastRepVec += pubRate_; 
		else
			timeElapsedLastRepVec = 0;
			
		if (timeElapsedLastRepVec > timeOut_)
		{
			ROS_WARN_THROTTLE(1, "%s: Repulsive vector timeout", nh_->getNamespace().c_str());
			clear_msg(repVec_);
		}
		
    if(display_)
    {
		  std::cout << "Rep Vec Available: " << startCount_ << std::endl;
		  std::cout << "Time Elapsed: " << timeElapsedLastRepVec << std::endl;
    }
		
		geometry_msgs::Pose currentPose = pose_;
		
		geometry_msgs::Vector3 attVecFlat = transform_goal_to_base_flat(goalPoint_, currentPose, mode_);
		geometry_msgs::Vector3 repVecFlat = transform_rep_vec_to_base_flat(repVec_, currentPose);
		
		if ( ( point_dist_sq(currentPose.position, goalPoint_) < pow(successRadius_,2) ) && isEmpty(repVecFlat) )
		{
			clear_msg(attVecFlat);
			clear_msg(repVecFlat);
		}

		geometry_msgs::TwistStamped twistOutMsg;
		
		clear_msg(twistOutMsg);

    if(mode_ == "apf")
    {
      twistOutMsg.twist.linear = get_unit_vector(add_vecs(attVecFlat, repVecFlat));
      twistOutMsg.twist.linear.x *= maxVel_[0];
      twistOutMsg.twist.linear.y *= maxVel_[0];
      twistOutMsg.twist.linear.z *= maxVel_[1];
    }
    else if(mode_ == "qmdp")
     twistOutMsg.twist.linear = add_vecs(attVecFlat, repVecFlat);	

    if(display_)
    {
      std::cout << "Holonomic seperate attractive and repulsive messages"	<< std::endl;
		  std::cout << attVecFlat.x << ", " << attVecFlat.y << "," << attVecFlat.z << std::endl;	
		  std::cout << repVecFlat.x << ", " << repVecFlat.y << "," << repVecFlat.z << std::endl;
		
		  std::cout << "Holonomic merged attractive and repulsive messages"	<< std::endl;
		  std::cout << twistOutMsg.twist.linear.x << ", " << twistOutMsg.twist.linear.y << "," << twistOutMsg.twist.linear.z << std::endl;
    }
		
		bool nonHolSuccess = true;
		if (!isHolonomic_)
			nonHolSuccess = convert_to_nonholonomic(twistOutMsg.twist.linear, twistOutMsg.twist);
		if ( !nonHolSuccess )
		{
			ROS_WARN_THROTTLE(1, "%s: Non-Holonomic Conversion Unsuccessful", nh_->getNamespace().c_str());
			return;
		}
		
    if(display_)
    {
      std::cout << "Non-Holonomic merged attractive and repulsive messages"	<< std::endl;
		  std::cout << twistOutMsg.twist.linear.x << ", " << twistOutMsg.twist.linear.y << "," << twistOutMsg.twist.linear.z << std::endl << std::endl;
    }

    twistOutMsg.header.stamp = ros::Time::now();

    if(worldFrameOut_)
    {
		  twistOutMsg.twist = base_flat_to_world(twistOutMsg.twist, currentPose);
      twistOutMsg.header.frame_id = worldFrameId_;
    }
    else
    {
      twistOutMsg.header.frame_id = baseFrameId_;
    }
		
		twistOutPub_.publish(twistOutMsg);
		
		startCount_ = true;
	}
	
	// ************************************************
	geometry_msgs::Twist base_flat_to_world(geometry_msgs::Twist twistIn, geometry_msgs::Pose poseIn)
	{
		tf2::Vector3 linVel(twistIn.linear.x, twistIn.linear.y, twistIn.linear.z);
		
		linVel = get_yaw_transform(poseIn.orientation) * linVel;
		
		geometry_msgs::Twist twistOut;
		twistOut.linear = toMsg(linVel);
		twistOut.angular = twistIn.angular;
		
		return twistOut;
	}
	
	// ************************************************
	tf2::Transform get_yaw_transform(geometry_msgs::Quaternion imuOrientIn)
	{
		tf2::Quaternion imuOrientOut;
		tf2::fromMsg(imuOrientIn, imuOrientOut);
		
		double roll, pitch, yaw;
		tf2::Matrix3x3(imuOrientOut).getRPY(roll, pitch, yaw);
		
		roll = 0; pitch = 0;
		
		imuOrientOut.setRPY(roll, pitch, yaw);
		
		return tf2::Transform(imuOrientOut);
	}
	
	// ************************************************
	bool convert_to_nonholonomic(geometry_msgs::Vector3 velIn, geometry_msgs::Twist& velOut) // In: Base frame, Out: World Frame
	{
		float pi = 3.14159265358979323846;
		
    float yawErr = atan2(velIn.y, velIn.x);
    
    velOut.linear.x = velIn.x;
    if ( abs(yawErr) > foreVelDeadband_ )
    	velOut.linear.x = 0;
    
    velOut.linear.y = 0;
    velOut.linear.z = velIn.z;
    
    velOut.angular.x = 0;
    velOut.angular.y = 0;
    velOut.angular.z = yawErr / pi * maxVel_[2];
    
    return true;
	}
	
	// ************************************************
	tf2::Transform base_flat_to_world(geometry_msgs::Pose poseIn)
	{
		tf2::Quaternion rotation;
		tf2::fromMsg(poseIn.orientation, rotation);
		
		double roll, pitch, yaw;
		tf2::Matrix3x3(rotation).getRPY(roll, pitch, yaw);
		roll = 0; pitch = 0;
		
		rotation.setRPY(roll, pitch, yaw);
		
		tf2::Vector3 origin(poseIn.position.x, poseIn.position.y, poseIn.position.z);
		
		return tf2::Transform(rotation, origin);
	}
	
	// ************************************************
	tf2::Transform get_base_inclination_transform(geometry_msgs::Quaternion imuOrientIn, std::string imuFrameId, std::string baseFrameId)
	{
		tf2::Quaternion baseOrientOut;
		tf2::fromMsg(imuOrientIn, baseOrientOut);
		
		//tf2::Quaternion baseToImu;
		//fromMsg(baseToImu_, baseToImu);
		
		//baseOrientOut *= baseToImu; 
		
		double roll, pitch, yaw;
		tf2::Matrix3x3(baseOrientOut).getRPY(roll, pitch, yaw);
		yaw = 0;
		
		baseOrientOut.setRPY(roll, pitch, yaw);

		return tf2::Transform(baseOrientOut);
	}
	
	// ************************************************
	bool find_max_mag_index(std::vector<geometry_msgs::Vector3> vecsIn, int& maxMagIndx)
	{	
		maxMagIndx = 0;
		float maxMagSq = 0;
		
		if (vecsIn.size() == 0)
		return false;
		
		for (int i=0; i<vecsIn.size(); i++)
		{
			float vecMagSq = pow(vecsIn[i].x,2) + pow(vecsIn[i].y,2) + pow(vecsIn[i].z,2);
			if ( vecMagSq > maxMagSq )
			{
				maxMagSq = vecMagSq;
				maxMagIndx = i;
			}
		}
		
		return true;
	}
	
	// ************************************************
	void clear_msg(geometry_msgs::TwistStamped& msg)
	{
		msg.twist.linear.x = 0;
		msg.twist.linear.y = 0;
		msg.twist.linear.z = 0;
		
		msg.twist.angular.x = 0;
		msg.twist.angular.y = 0;
		msg.twist.angular.z = 0;
	}
	
	// ************************************************
	void clear_msg(geometry_msgs::Vector3& msg)
	{
		msg.x = 0; msg.y = 0;	msg.z = 0;
	}
	
	// ************************************************
	float get_magnitude(geometry_msgs::Vector3& vecIn)
	{
		return sqrt(pow(vecIn.x,2) + pow(vecIn.y,2) + pow(vecIn.z,2));
	}
	
	// ************************************************
	geometry_msgs::Vector3 add_vecs(const geometry_msgs::Vector3& vec1, const geometry_msgs::Vector3& vec2)
	{
		geometry_msgs::Vector3 vecRes;
		vecRes.x = vec1.x + vec2.x;
		vecRes.y = vec1.y + vec2.y;
		vecRes.z = vec1.z + vec2.z;
		
		return vecRes;
	}

	// ************************************************
	geometry_msgs::Vector3 get_unit_vector(geometry_msgs::Vector3 vecIn)
	{
		float vecInMag = sqrt(pow(vecIn.x,2) + pow(vecIn.y,2) + pow(vecIn.z,2)); 

    geometry_msgs::Vector3 vecOut;
    vecOut.x = 0; vecOut.y = 0; vecOut.z = 0;

    if(vecInMag == 0.0)
      return vecOut;
		
		vecOut.x = vecIn.x / vecInMag;
		vecOut.y = vecIn.y / vecInMag;
		vecOut.z = vecIn.z / vecInMag;
		
		return vecOut;
	}
	
	// ************************************************
	float point_dist_sq(geometry_msgs::Point pt1, geometry_msgs::Point pt2)
	{
		float xDistSq = pow(pt2.x - pt1.x, 2);
		float yDistSq = pow(pt2.y - pt1.y, 2);
		float zDistSq = pow(pt2.z - pt1.z, 2);
		
		return xDistSq+yDistSq+zDistSq;
	}
	
	// ************************************************
	bool isEmpty(geometry_msgs::Vector3 msg)
	{
		if (msg.x == 0 && msg.y == 0 && msg.z == 0)
			return true;
			
		return false;
	}
	
	// ************************************************
	/*
	bool change_drone_pose_mode(char mode)
	{	
		drone_pose::flightModeSrv flModSrv;
		flModSrv.request.setGet = 0;
		flModSrv.request.flightMode = 'V';
		
		flModClient_.call(flModSrv);
			
		if(flModSrv.response.flightMode != 'V')
		{		
			ROS_WARN_THROTTLE(1, "%s: Could not change flight mode. Trying again ...", nh_->getNamespace().c_str());
			return false;
		}
		
		return true;
	}
	*/
};


// Main
int main(int argc, char **argv)
{
	ros::init(argc, argv, "goal_to_vel_node");
	ros::NodeHandle nh(ros::this_node::getName());

	goal_to_vel_class goalToVel(&nh);
	
	ros::spin();
	return 0;
}
