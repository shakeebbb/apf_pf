#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <random>
#include <chrono>

using namespace std;

int main(int argc, char** argv){
  ros::init(argc, argv, "loc_noiser");

  ros::NodeHandle nh("loc_noiser");

  double rate;
  string targetFrame, sourceFrame;
  vector<double> mean, sdev;

  while(!nh.getParam("mean_position", mean));
  while(!nh.getParam("standard_deviation", sdev));
  while(!nh.getParam("rate", rate));
  while(!nh.getParam("source_frame_id", sourceFrame));
  while(!nh.getParam("target_frame_id", targetFrame));
    
  tf2_ros::TransformBroadcaster br;

  std::normal_distribution<double> distribution_x( mean[0], sdev[0]);
  std::normal_distribution<double> distribution_y( mean[1], sdev[1]);
  std::normal_distribution<double> distribution_z( mean[2], sdev[2]);

  ros::Rate loop_rate(rate);
  while(ros::ok())
  {
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	  std::default_random_engine generator(seed);

    geometry_msgs::TransformStamped transformStamped;
  
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = targetFrame;
    transformStamped.child_frame_id = sourceFrame;

     transformStamped.transform.translation.x = distribution_x(generator);
     transformStamped.transform.translation.y = distribution_y(generator);
     transformStamped.transform.translation.z = distribution_z(generator);


     tf2::Quaternion q;
     q.setRPY(0, 0, 0);
     transformStamped.transform.rotation.x = q.x();
     transformStamped.transform.rotation.y = q.y();
     transformStamped.transform.rotation.z = q.z();
     transformStamped.transform.rotation.w = q.w();

     br.sendTransform(transformStamped);
     ros::spinOnce();

     loop_rate.sleep();
  }

  return 0;
};
