#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "std_msgs/Float64MultiArray.h"
#include "cv_bridge/cv_bridge.h"
#include "pcl_ros/point_cloud.h"

using namespace std;

ros::Publisher imgPub_;

std::vector<double> belief_={0};
double pixInt_;
double distInt_;
sensor_msgs::CameraInfo camInfo_;
sensor_msgs::CameraInfo beliefCamInfo_;
uint8_t isInitialized_ = 0x00;

void img_cb(const sensor_msgs::ImageConstPtr&);
void cam_info_cb(const sensor_msgs::CameraInfo&);
void belief_cam_info_cb(const sensor_msgs::CameraInfo&);
void belief_cb(const std_msgs::Float64MultiArray&);

pcl::PointXYZ project_to_3d(pcl::PointXYZ, sensor_msgs::CameraInfo, bool);
	
int main(int argc, char** argv)
{
	ros::init(argc, argv, "img_marker_node");
	ros::NodeHandle nh(ros::this_node::getName());
	
	ros::Subscriber imgSub = nh.subscribe("img_in", 1, img_cb);
  ros::Subscriber camInfoSub = nh.subscribe("cam_info_in", 1, cam_info_cb);

  ros::Subscriber beliefCamInfoSub = nh.subscribe("belief_cam_info_in", 1, belief_cam_info_cb);
  ros::Subscriber beliefSub = nh.subscribe("belief_in", 1, belief_cb);

	imgPub_ = nh.advertise<sensor_msgs::Image>("img_out", 10);
	
  while(!nh.getParam("pixel_interval", pixInt_));
  while(!nh.getParam("distance_interval", distInt_));

	ros::spin();

	return 0;
}


void img_cb(const sensor_msgs::ImageConstPtr& msg)
{
  if((isInitialized_ & 0x03) != 0x03)
  {
   ROS_WARN("%s", "Waiting");
   return;
  }

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  int gridWidth = ceil(double(msg->width) / pixInt_);
  int gridHeight = ceil(double(msg->height) / pixInt_);
  int gridSize = gridWidth*gridHeight;

  for(int i=0; i<(belief_.size()-1); i++)
  {
    if(belief_[i] > 0.0)
    {
      int indx = i;
      
      int voxZ = floor( double(indx) / double(gridSize) );
      int voxX = ( indx % gridSize ) % gridWidth;
      int voxY = floor( double( indx % gridSize ) / double(gridWidth) );

      int pixX = voxX * pixInt_;
      int pixY = voxY * pixInt_;
      double pixZ = voxZ * distInt_ + 0.1;

      pcl::PointXYZ point3a = project_to_3d(pcl::PointXYZ(pixX,pixY,pixZ), beliefCamInfo_, true);
      pcl::PointXYZ point2a = project_to_3d(point3a, camInfo_, false);

      pcl::PointXYZ point3b = project_to_3d(pcl::PointXYZ(pixX+pixInt_,pixY+pixInt_,pixZ), beliefCamInfo_, true);
      pcl::PointXYZ point2b = project_to_3d(point3b, camInfo_, false);

      rectangle(cv_ptr->image, cv::Point(point2a.x, point2a.y), cv::Point(point2b.x, point2b.y), CV_RGB(0,0,255*belief_[indx]), 5);
    }
  }

  imgPub_.publish(cv_ptr->toImageMsg());
}

void belief_cb(const std_msgs::Float64MultiArray& msg)
{
  belief_ = msg.data;
}

void cam_info_cb(const sensor_msgs::CameraInfo& msg)
{
  camInfo_ = msg;
  isInitialized_ |= 0x01;
}

void belief_cam_info_cb(const sensor_msgs::CameraInfo& msg)
{
  beliefCamInfo_ = msg;
  isInitialized_ |= 0x02;
}

pcl::PointXYZ project_to_3d(pcl::PointXYZ pointIn, sensor_msgs::CameraInfo camInfo, bool direction)
{
	pcl::PointXYZ pointOut;
	
	double fx = camInfo.K[0];
	double cx = camInfo.K[2];
	double fy = camInfo.K[5];
	double cy = camInfo.K[6];
	
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
