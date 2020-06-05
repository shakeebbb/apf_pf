#include "filter_point.h"

using namespace std;
	
int main(int argc, char** argv)
{
	ros::init(argc, argv, "filter_point_node");
	ros::NodeHandle nh(ros::this_node::getName());
	
	// Pixel Interval, Distance Interval, Min Distance, Max Distance
	
	filter_point_class filterPoint(&nh);
	
	ros::spin();

	return 0;
}
