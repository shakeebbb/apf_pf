#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/PCLPointCloud2.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"

using namespace std;

class particleGridFilter
{		

		// Array of all possible grid combinations, each grid is an integer
		// Array of all possible evidence combinations, each observation is an integer
		uint32_t** allPossibleGrids; 
		uint32_t** allPossibleEvidence; 
		
		float* transitionModel;
		float* evidenceModel;
		
		float voxelSize;
		float sensingHorizon;
		int nVoxels;
		
	public:
		particleGridFilter(float vs, float rng) // voxel size, range
		{
			voxelSize = vs;
			sensingHorizon = rng;
			nVoxels = pow(floor(rng/vs), 3);
			
			uint16_t 
			allPossibleGrids = new uint32_t*[2^nVoxels]; // 2 possible voxel values, one bit representing one voxel
			
			for (uint16_t i=0; i<(2^nVoxels); i++)
			{
				allPossibleGrids[i] = new uint32_t[ceil(nVoxels/16)]; // Each 16 bits is an unsigned integer
			}
			
			allPossibleEvidence = new unsigned int*[2^(2*nVoxels)]; // 4 possible evidence about a voxel, two bits representing one voxel
			for (unsigned int i=0; i<(2^(2*nVoxels)); i++)
			{
				allPossibleEvidence[i] = new unsigned int[ceil(2*nVoxels/16)]; // Each 16 bits is an unsigned integer
			}
		}
		
		
		vector<vector<int>> get_possible_arrays(int arrayLength, vector<int> possibleValues) // number of voxels, possible voxel values
		{
			vector<vector<int>> allPossibleArrays; // Vector of all possible grid combinations		
			vector<vector<int>> allPossibleValues; // nVoxels vectors, each of length size(possibleVoxels)
			
			int* indices = new int[arrayLength]; 
			
			for(int i = 0; i < arrayLength; i++)
			{
				allPossibleValues.push_back(possibleValues);
				indices[i] = 0;
			} 
  
   		while(1) 
   			{ 
		 		 	vector<int> arrayInstance;
		    
		      for (int i = 0; i < arrayLength; i++) 
		      {
		      	arrayInstance.push_back(allPossibleValues[i][indices[i]]);
		      	//cout << allPossibleVoxels[i][indices[i]] << " "; 
        	}
        	//cout << endl; 	
        	//getchar();	       
		      allPossibleArrays.push_back(gridInstance);

		      int next = nVoxels - 1; 
		      while (next >= 0 && (indices[next] + 1 >= allPossibleValues[next].size())) 
		      	next--; 
		      	
		      	
		      //cout << "Next: " << next << endl;
		
		      if (next < 0) 
		      	return allPossibleArrays; 
		
		      indices[next]++; 
	 
		      for (int i = next + 1; i < arrayLength; i++) 
		      	indices[i] = 0; 
    		}
		}
};

void pointCb(const sensor_msgs::PointCloud2&);

int main(int argc, char** argv)
{

ros::init(argc, argv, "filter_point");
ros::NodeHandle nh;

ros::Subscriber pointSub = nh.subscribe("/royale_camera_driver/point_cloud", 100, pointCb);

particleGridFilter pgf(0.05, 0.15);
	
ros::spin();

return 0;
}


void pointCb(const sensor_msgs::PointCloud2& msg)
{
	cout << "Height: " << msg.height << endl;
	cout << "Width: " << msg.width << endl;
	
	cout << "Fields Size: " << msg.fields.size() << endl;
	
	cout << "Is_bigendian: " << msg.is_bigendian << endl;
	
	cout << "Point Step: " << msg.point_step << endl;
	cout << "Row Step: " << msg.row_step << endl;
	
	cout << "Data Size: " << msg.data.size() << endl;
	
	cout << "Is_dense: " << msg.is_dense << endl;
	
	pcl::PCLPointCloud2 pclMsg;
	pcl::PointCloud<pcl::PointXYZ> pt_cloud;
	
	pcl_conversions::toPCL (msg, pclMsg);
	pcl::fromPCLPointCloud2 (pclMsg, pt_cloud);
	
	cout << "Pt_cloud Size: " << pt_cloud.size() << endl << endl;
	
}
