#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/PCLPointCloud2.h"
#include "pcl/point_cloud.h"
#include "pcl/point_representation.h"
#include "pcl_ros/point_cloud.h"

using namespace std;

void pt_cloud_cb(const pcl::PointCloud<pcl::PointXYZ>&);
ros::Publisher ptCloudPub;
	
int main(int argc, char** argv)
{
	ros::init(argc, argv, "noisy_cloud_node");
	ros::NodeHandle nh;
	
	ros::Subscriber ptCloudSub = nh.subscribe("noise_cloud_node/pt_cloud_in", 100, pt_cloud_cb);
	ptCloudPub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("noise_cloud_node/pt_cloud_out", 100);
	
	ros::spin();

	return 0;
}

void pt_cloud_cb(const pcl::PointCloud<pcl::PointXYZ>& msg)
{
	pcl::PointCloud<pcl::PointXYZ> ptCloud;
	ptCloud = msg;
	
	int count = 0;
	for (int i=0; i<ptCloud.size(); i++)
	{
		if(isnan(ptCloud[i].x))
		continue;
		
		if(count <5)
		count++;
		else
		{			
			srand(i);
			
			ptCloud[i].x += (float(rand() % 3) / 5) - 0.3;
			
			srand(i+3000);
			ptCloud[i].y += (float(rand() % 3) / 5) - 0.3;
			
			srand(i+5000);
			ptCloud[i].z += (float(rand() % 3) / 5) - 0.3;
			
			count = 0;
		}
	}
	
	ptCloudPub.publish(ptCloud);
}
