#include "ros/ros.h"
#include <random>
#include <chrono>
#include "sensor_msgs/PointCloud2.h"
#include "pcl/PCLPointCloud2.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/common/distances.h"
#include "pcl/point_representation.h"

using namespace std;

class particleGridFilter
{		
		// Array of all possible grid combinations, each grid is an integer
		// Array of all possible evidence combinations, each observation is an integer
		float* allPossibleDistances; 
		
		float** transitionModel;
		
		float discretizationinterval;
		float minDistance;
		float maxDistance;
		int nDistances;
		int nVoxels;
		
		// Observation Features, Changing Every Iteration
		pcl::PointXYZ* allPossibleVoxels;
		int* nVoxelPoints;
		int nearestVoxel;
		
	public:
		// *******************************************************************
		particleGridFilter(float discIntZ, int discIntXY, float minDist, float maxDist, int imgHeight, int imgWidth) // Discretization Interval, Sensing Horizon
		{
			discretizationinterval = discIntZ;
			minDistance = minDist;
			maxDistance = maxDist;
			
			nDistances = ceil((maxDistance - minDistance)/discretizationinterval + 1);
			allPossibleDistances = new float[nDistances];
			
			for (int i=0; i<nDistances; i++)
			allPossibleDistances[i] = minDistance + (i*discretizationinterval);
			
			repPoints = new float*[nDistances];
			for (int i=0; i<nDistances; i++)
			repPoints[i] = new float[5];
			
			nearestPoint = new float[5];
			
			nVoxels = ceil(imgWidth/discIntXY + 1);
			allPossibleVoxels = new pcl::PointXYZ[nVoxels];
			nVoxelPoints = new int[nVoxels];
			
			allPossibleVoxels = new
		}
		
		// *******************************************************************
		~particleGridFilter()
		{
			delete allPossibleDistances;
			
			for (int i=0; i<nDistances; i++)
			delete transitionModel[i];
			delete transitionModel;
			
			for (int i=0; i<nDistances; i++)
			delete repPoints[i];
			delete repPoints;
			
			delete nearestPoint;
			
			delete allPossibleVoxels;
			
			delete nVoxelPoints;
		}
		
		// *******************************************************************
		void populate_transition_model()
		{
			transitionModel = new float*[nDistances];
			for (int i=0; i<nDistances; i++)
			{
				transitionModel[i] = new float[nDistances];
				for (int j=0; j<nDistances; j++)
				{
					if (j >= i)
						transitionModel[i][j] = 1 - abs(allPossibleDistances[j] - allPossibleDistances[i])/(maxDistance - minDistance);
					else
						transitionModel[i][j] = transitionModel[j][i];
				}
			}
		}
		
		// *******************************************************************
		// Current State Index ---> Next State index
		// *******************************************************************
		int generative_model(int index) 
		{
			discrete_distribution<int> distObject(transitionModel[index], transitionModel[index]+nDistances);
			//for (double x:distObject.probabilities()) std::cout << x << " " << endl;
			
			unsigned seed = chrono::system_clock::now().time_since_epoch().count();
			
			//mt19937 generator;
			//generator.seed(seed);
			
			default_random_engine generator(seed);
			
			return distObject(generator);	
		}
		
		// *******************************************************************
		// Current Belief ---> Randomly Picked State index
		// *******************************************************************
		int random_distance(float* belief)
		{
			discrete_distribution<int> distObject(belief, belief+nDistances);
			
			unsigned seed = chrono::system_clock::now().time_since_epoch().count();
			
			default_random_engine generator(seed);
			
			return distObject(generator);	
		} 
		
		// *******************************************************************
		// Next State Index, PCL Points, Number of Points in PCL at (sp +- epsilon), 
		// Distance of sp to the Camera Closest Point in PCL  ---> Weight of the State-Obsv Pair
		// *******************************************************************
		void evidence_weight(int distIndex, float& densityWeight, float& nearnessWeight)
		{	
			if (repPoints[distIndex][3] == -1 && repPoints[distIndex][4] == -1)
			{
				densityWeight = 0;
				nearnessWeight = 0;
			}
			else
			{			
				densityWeight = repPoints[distIndex][4]; 
				nearnessWeight = ( 1 - abs(repPoints[distIndex][3] - nearestPoint[3])/(maxDistance - minDistance) );
			}
			
			cout << "Density Weight: " << densityWeight << endl;
			cout << "Nearness Weight: " << nearnessWeight << endl;
		}
		
		// *******************************************************************
		// PCL, epsilon --> Number of Points at (s+- epsilon) of each state, 
		// Closest Point Distance 
		// *******************************************************************
		void extract_features(pcl::PointCloud<pcl::PointXYZ>& ptCloud, float epsilon)
		{
			pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
			kdTree.setInputCloud(ptCloud.makeShared());
			
			fill(repPoints[0], repPoints[0] + nDistances * 5, -1);
			fill(nearestPoint, nearestPoint + 5, -1);
			nearestPoint[3] = maxDistance;
			
			vector<int> neighbors;
  		vector<float> radii;
  			
  		int pointState;
  		float pointDistance;
  		
			for (int i=0; i<ptCloud.size(); i++)
			{
				//cout << i << endl;
				if(!isValid(ptCloud[i]))
				continue;

  			point_state_dist(ptCloud[i], pointState, pointDistance);
  			
  			//cout << ptCloud[i].x << endl;

				if (kdTree.radiusSearch(ptCloud[i], epsilon, neighbors, radii) > 0)
  			{
  				//cout << ptCloud[i].y << endl;
  				if(neighbors.size() > repPoints[pointState][1])
  				{
						repPoints[pointState][0] = ptCloud[i].x;
						repPoints[pointState][1] = ptCloud[i].y;
						repPoints[pointState][2] = ptCloud[i].z;
						repPoints[pointState][3] = pointDistance; 
						repPoints[pointState][4] = neighbors.size();
						
						if(repPoints[pointState][3] < nearestPoint[3])
						{
							nearestPoint[0] = ptCloud[i].x;
							nearestPoint[1] = ptCloud[i].y;
							nearestPoint[2] = ptCloud[i].z;
							nearestPoint[3] = pointDistance; 
							nearestPoint[4] = neighbors.size();
						}
  				}
  			}
			}
		}
		
		// *******************************************************************
		// PCL Point --> State Index To Which The Point Belongs
		// *******************************************************************
		void point_state_dist(pcl::PointXYZ& inPoint, int& pointState, float& pointDistance)
		{
			pointDistance = pcl::euclideanDistance (pcl::PointXYZ(0,0,0), inPoint);
			pointState = ceil( (pointDistance - minDistance) / discretizationinterval );
			
			if(pointState >= nDistances)
			pointState = nDistances-1;
			if(pointState < 0)
			pointState = 0;
		}
		
		// *******************************************************************
		bool isValid(pcl::PointXYZ& point)
		{
			return !(isnan(point.x) || isnan(point.y) || isnan(point.z));
		}
		
		// *******************************************************************
		void display(int precision)
		{
			cout << setprecision(precision) << endl;
			cout << "<===============================================>" << endl; 
			cout << "Number of possible states: " << nDistances << endl;
			cout << "All possible states: [";
			for (int i=0; i<(nDistances-1); i++)
			{
				cout << allPossibleDistances[i] << ", ";
			}
			cout << allPossibleDistances[nDistances-1] << "]" << endl;
			cout << "Transition Model: " << endl;
			for (int i=0; i<nDistances; i++)
			{
				for (int j=0; j<nDistances; j++)
				{
					cout << transitionModel[i][j] << "\t";
				}
				cout << endl;
			}
			
			ROS_INFO("%f", transitionModel[1][2]);
			cout << endl << endl;
		}
};

// *******************************************************************
void pointCb(const sensor_msgs::PointCloud2&);

// *******************************************************************

particleGridFilter pgf(0.05, 0.00, 0.35);
	
int main(int argc, char** argv)
{
	ros::init(argc, argv, "filter_point");
	ros::NodeHandle nh;

	ros::Subscriber pointSub = nh.subscribe("/royale_camera_driver/point_cloud", 100, pointCb);
	
	pgf.populate_transition_model();
	pgf.display(2);

	//particleGridFilter pgf(0.05, 0.15, 0.35);
	//pgf.populate_transition_model();
	//pgf.display(2);
	
	//for (int i=0; i<3500; i=i+50)
	//cout << "Observation Weight: " << pgf.evidence_weight(1, 38304, i, 0.15) << endl;
	
	/*
	int count[5];
	count[0] = 0;
	count[1] = 0;
	count[2] = 0;
	count[3] = 0;
	count[4] = 0;
	count[5] = 0;
	for (int i=0; i<500; i++)
	{
		int k = pgf.generative_model(1);
		//cout << "Current Index: " << 1 << endl;
		//cout << "Next Index: " << k << endl << endl;
		count[k] ++;
	}
	for (int i=0; i<5; i++)
	{
		for (int j=0; j<count[i]; j++)
		cout << "*";
		cout << endl;
	}
	cout << endl;
	*/
	
	ros::spin();

	return 0;
}


// *******************************************************************
void pointCb(const sensor_msgs::PointCloud2& msg)
{
/*
	cout << "Height: " << msg.height << endl;
	cout << "Width: " << msg.width << endl;
	
	cout << "Fields Size: " << msg.fields.size() << endl;
	
	cout << "Is_bigendian: " << int(msg.is_bigendian) << endl;
	
	cout << "Point Step: " << msg.point_step << endl;
	cout << "Row Step: " << msg.row_step << endl;
	
	cout << "Data Size: " << msg.data.size() << endl;
	
	cout << "Is_dense: " << int(msg.is_dense) << endl;
*/
	
	pcl::PCLPointCloud2 pclMsg;
	pcl::PointCloud<pcl::PointXYZ> ptCloud;
	
	pcl_conversions::toPCL (msg, pclMsg);
	pcl::fromPCLPointCloud2 (pclMsg, ptCloud);
	
	cout << "Pt_cloud Size: " << ptCloud.size() << endl << endl;
	
	pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
	kdTree.setInputCloud(ptCloud.makeShared());
	float epsilon = 0.15;
	vector<int> neighbors;
	vector<float> radii;
			
	for(int i=0; i<ptCloud.size(); i++)
	{
		if(!isnan(ptCloud[i].x))
		{
			kdTree.radiusSearch(ptCloud[i], epsilon, neighbors, radii);
		}
		//cout << i <<  endl;
	}
	cout << "done" <<  endl;
	
	//BOOST_FOREACH (const pcl::PointXYZ& pt, ptCloud)
   // printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
	
	//pgf.extract_features(ptCloud, 0.15);
	
	//float densityWeight, nearnessWeight;
	//pgf.evidence_weight(1, densityWeight, nearnessWeight);
	
	//cout << "Density Weight: " << densityWeight << endl;
	//cout << "Nearness Weight: " << nearnessWeight << endl;
	
}
