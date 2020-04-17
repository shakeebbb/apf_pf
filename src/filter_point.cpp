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
#include <pcl_ros/point_cloud.h>

using namespace std;

class particleGridFilter
{		
		// Array of all possible grid combinations, each grid is an integer
		// Array of all possible evidence combinations, each observation is an integer
		float* allPossibleDistances; 
		
		float** transitionModel;
		
		float discretizationinterval;
		int discretizationPixels;
		float minDistance;
		float maxDistance;
		int nDistances;
		int nVoxels;
		
		// Observation Features, Changing Every Iteration
		pcl::PointXYZ* allPossibleVoxels;
		int* ptsVoxel;
		float* distVoxel;
		float refVoxelDistance;
		int imageWidth;
		int imageHeight;
		int voxelGridWidth;
		int voxelGridHeight;
		int nOutliers;
		
		// ROS Related
		//ros::Publisher ptCloudPub;
		
	public:
		// *******************************************************************
		particleGridFilter(float discIntZ, int discIntXY, float minDist, float maxDist, int imgHeight, int imgWidth) // Discretization Interval, Sensing Horizon
		{
			//ptCloudPub = nh->advertise<sensor_msgs::PointCloud2>("cloud_out", 100);
		
			discretizationinterval = discIntZ;
			discretizationPixels = discIntXY;
			minDistance = minDist;
			maxDistance = maxDist;
			imageWidth = imgWidth;
			imageHeight = imgHeight;
			
			nDistances = ceil((maxDistance - minDistance)/discretizationinterval + 1);
			allPossibleDistances = new float[nDistances];
			
			for (int i=0; i<nDistances; i++)
			allPossibleDistances[i] = minDistance + (i*discretizationinterval);
			
			voxelGridWidth = ceil(float(imgWidth)/float(discIntXY));
			voxelGridHeight = ceil(float(imgHeight)/float(discIntXY));
			
			nVoxels = voxelGridWidth*voxelGridHeight*nDistances + 1;
			cout << voxelGridWidth << endl;
			cout << voxelGridHeight << endl;
			cout << nDistances << endl;
			cout << nVoxels << endl;
			getchar();
			allPossibleVoxels = new pcl::PointXYZ[nVoxels];
			ptsVoxel = new int[nVoxels];
			
			distVoxel = new float[nVoxels];
			
			refVoxelDistance = 0;
			nOutliers = 50;
		}
		
		// *******************************************************************
		~particleGridFilter()
		{
			delete allPossibleDistances;
			
			for (int i=0; i<nDistances; i++)
			delete transitionModel[i];
			delete transitionModel;
			
			delete allPossibleVoxels;
			
			delete ptsVoxel;
			
			delete distVoxel;
		}
		
		// *******************************************************************
		void populate_transition_model()
		{
			transitionModel = new float*[nVoxels];
			for (int i=0; i<nVoxels; i++)
			{
				transitionModel[i] = new float[nVoxels];
				for (int j=0; j<nVoxels; j++)
				{
					int indexVoxZ1 = floor(float(i) / float(voxelGridWidth*voxelGridHeight));
					int indexVoxY1 = floor(float(i - indexVoxZ1*voxelGridWidth*voxelGridHeight) / float(voxelGridWidth));
					int indexVoxX1 = i - indexVoxZ1*voxelGridWidth*voxelGridHeight - indexVoxY1*voxelGridWidth;	
							
					int indexVoxZ2 = floor(float(j) / float(voxelGridWidth*voxelGridHeight));
					int indexVoxY2 = floor(float(j - indexVoxZ2*voxelGridWidth*voxelGridHeight) / float(voxelGridWidth));
					int indexVoxX2 = j - indexVoxZ2*voxelGridWidth*voxelGridHeight - indexVoxY2*voxelGridWidth;
					
					int indexVoxZC = 0;
					int indexVoxYC = voxelGridHeight/2;
					int indexVoxXC = voxelGridWidth/2;					
					
					int manDistMax = abs(voxelGridWidth) + abs(voxelGridHeight) + abs(nDistances);
					
					if (i == (nVoxels-1))
					{
						int manDist = abs(indexVoxX2 - indexVoxXC) + abs(indexVoxY2 - indexVoxYC) + abs(indexVoxZ2 - indexVoxZC);
						transitionModel[i][j] = float(manDist) / float(manDistMax); 
					}
					else if (j == (nVoxels-1))
					{
						int manDist = abs(indexVoxX1 - indexVoxXC) + abs(indexVoxY1 - indexVoxYC) + abs(indexVoxZ1 - indexVoxZC);
						transitionModel[i][j] = float(manDist) / float(manDistMax); // Can be made less conservative manDist << manDistMax
					}
					else if (j >= i)
					{	
						//cout << "(i, j) = " << i << ", " << j << endl;
						//cout << "(Width, Height) = " << voxelGridWidth << ", " << voxelGridHeight << endl;						
						//cout << "(vx, vy, vz) = " << indexVoxX1 << ", " << indexVoxY1 << ", " << indexVoxZ1 << endl;
						//cout << "(vx, vy, vz) = " << indexVoxX2 << ", " << indexVoxY2 << ", " << indexVoxZ2 << endl;
						int manDist = abs(indexVoxX2 - indexVoxX1) + abs(indexVoxY2 - indexVoxY1) + abs(indexVoxZ2 - indexVoxZ1);
						//cout << manDist << endl;
						//cout << manDistMax << endl;
						//getchar();

						transitionModel[i][j] = 1 - float(manDist) / float(manDistMax);
						//abs(allPossibleDistances[j] - allPossibleDistances[i])/(maxDistance - minDistance);
					}
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
			discrete_distribution<int> distObject(transitionModel[index], transitionModel[index]+nVoxels);
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
		int random_voxel(float* belief)
		{
			discrete_distribution<int> distObject(belief, belief+nVoxels);
			
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
		/*
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
			*/
		}
		
		// *******************************************************************
		// PCL, epsilon --> Number of Points at (s+- epsilon) of each state, 
		// Closest Point Distance 
		// *******************************************************************
		void extract_features(pcl::PointCloud<pcl::PointXYZ>& ptCloud)
		{
			fill(allPossibleVoxels, allPossibleVoxels + nVoxels, pcl::PointXYZ(0,0,0));
			fill(ptsVoxel, ptsVoxel + nVoxels, 0);
			fill(distVoxel, distVoxel + nVoxels, 0);
			refVoxelDistance = maxDistance;
  			
  		int indexVox;
  		float distVox;
  		int nValidPts = 0;
  		
			for (int i=0; i<ptCloud.size(); i++)
			{
				if(!is_valid(ptCloud[i]))
				continue;
				
				nValidPts++;
  			if(!point_to_voxel(ptCloud[i], i, indexVox, distVox))
  			continue;
  			
  			//cout << i << endl;
  			//cout << "Image Width: " << imageWidth << endl;
  			//cout << "Point Index: " << i << endl;
  			//cout << "Point X: " << ptCloud[i].x << endl;
  			//cout << "Point Y: " << ptCloud[i].y << endl;
  			//cout << "Point Z: " << ptCloud[i].z << endl;
  			//cout << "Voxel Index: " << indexVox << endl;
  			//cout << "Voxel Distance: " << distVox << endl;
  			//getchar();
  			
  			//if(indexVox >= nVoxels)
  			//{	
  				//cout << voxelGridWidth*voxelGridHeight*nDistances << endl;
  				//cout << nVoxels << endl;
  				//cout << indexVox << endl << endl;
  				//getchar();
  			//}
  			ptsVoxel[indexVox] ++;
  			//cout << "Points in Voxel " << indexVox << " = " << ptsVoxel[indexVox] << endl;
  			allPossibleVoxels[indexVox] = ptCloud[i];
  			distVoxel[indexVox] = distVox;
  				
  			if(distVoxel[indexVox] < refVoxelDistance)
  			refVoxelDistance = distVoxel[indexVox];
  		}
  		
  		//allPossibleVoxels[nVoxels-1] = pcl::PointXYZ(0,0,0);
  		//ptsVoxel[nVoxels-1] = nOutliers;
  		//distVoxel[nVoxels-1] = 0;
		}
		
		// *******************************************************************
		// PCL Point --> State Index To Which The Point Belongs
		// *******************************************************************
		bool point_to_voxel(pcl::PointXYZ& pt, int& indexPt, int& indexVox, float& distVox)
		{
			//distVox = abs(pt.x) + abs(pt.y) + abs(pt.z);
			
			distVox = pcl::euclideanDistance (pcl::PointXYZ(0,0,0), pt);
			
			if((distVox > maxDistance) || (distVox < minDistance))
			return false;
			
			int indexVoxX = floor(float(indexPt % imageWidth) / float(discretizationPixels));
			int indexVoxY = floor(floor(float(indexPt)/float(imageWidth)) / float(discretizationPixels));
			int indexVoxZ = floor((distVox - minDistance) /  discretizationinterval);
			/*
			cout << "Voxel Grid Size: " << voxelGridWidth << ", "
			  													<< voxelGridHeight << ", "
			  													<< nDistances << endl;
			cout << "Voxel Grid Point: " << indexVoxX << ", "
			  													<< indexVoxY << ", "
			  													<< indexVoxZ << endl;
			*/
			if(indexVoxX > ceil(imageWidth/discretizationPixels))
			cout << indexVoxX << endl;
			if(indexVoxY > ceil(imageHeight/discretizationPixels))
			cout << indexVoxY << endl;
			if(indexVoxZ > nDistances)
			cout << indexVoxZ << endl;
			
			indexVox = indexVoxX + voxelGridWidth*(indexVoxY + voxelGridHeight*indexVoxZ);
			
			return true;
		
			//pointDistance = pcl::euclideanDistance (pcl::PointXYZ(0,0,0), inPoint);
			//pointState = ceil( (pointDistance - minDistance) / discretizationinterval );
			
			//if(pointState >= nDistances)
			//pointState = nDistances-1;
			//if(pointState < 0)
			//pointState = 0;
		}
		
		// *******************************************************************
		bool is_valid(pcl::PointXYZ& point)
		{
			return !(isnan(point.x) || isnan(point.y) || isnan(point.z));
		}
		
		// *******************************************************************
		void publish_voxels(ros::Publisher& msgPub)
		{
			pcl::PointCloud<pcl::PointXYZ> ptCloudMsg;
			ptCloudMsg.header.frame_id = "royale_camera_optical_frame";
			ptCloudMsg.height = 1;
			ptCloudMsg.width = nVoxels;
			
			for (int i=0; i<nVoxels; i++)
			{
				ptCloudMsg.points.push_back(allPossibleVoxels[i]);
			}
			pcl_conversions::toPCL(ros::Time::now(), ptCloudMsg.header.stamp);
			msgPub.publish(ptCloudMsg);
			
			//cout << "Points Each Voxel: " << endl;
			//for (int i=0; i<nVoxels; i++)
			//{	
			//	if(ptsVoxel[i] > 0)
			//	cout << ptsVoxel[i] << ", ";
			//}
			//cout << endl;
			//ptCloudMsg.row_step = ceil(imageWidth/discretizationPixels);
		}
		
		// *******************************************************************
		void display(int precision)
		{
			cout << setprecision(precision) << endl;
			cout << "<===============================================>" << endl; 
			cout << "Number of possible states: " << nVoxels << endl;
			cout << "All possible states: [";
			for (int i=0; i<(nVoxels-1); i++)
			{
				cout << allPossibleVoxels[i] << ", ";
			}
			cout << allPossibleVoxels[nVoxels-1] << "]" << endl;
			cout << "Transition Model: " << endl;
			for (int i=0; i<nVoxels; i++)
			{
				for (int j=0; j<nVoxels; j++)
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

ros::Publisher ptCloudPub;

// *******************************************************************

particleGridFilter pgf(0.1, 15, 0.1, 2.00, 171, 224);
	
int main(int argc, char** argv)
{
	ros::init(argc, argv, "filter_point");
	ros::NodeHandle nh;

	ros::Subscriber pointSub = nh.subscribe("/royale_camera_driver/point_cloud", 100, pointCb);
	ptCloudPub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("cloud_out", 100);
	
	pgf.populate_transition_model();
	//pgf.display(2);

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

	cout << "Frame Id: " << msg.header.frame_id << endl;
	cout << "Height: " << msg.height << endl;
	cout << "Width: " << msg.width << endl;
	
	cout << "Fields Size: " << msg.fields.size() << endl;
	
	cout << "Is_bigendian: " << int(msg.is_bigendian) << endl;
	
	cout << "Point Step: " << msg.point_step << endl;
	cout << "Row Step: " << msg.row_step << endl;
	
	cout << "Data Size: " << msg.data.size() << endl;
	
	cout << "Is_dense: " << int(msg.is_dense) << endl;

	ros::Time begin = ros::Time::now();
	
	pcl::PCLPointCloud2 pclMsg;
	pcl::PointCloud<pcl::PointXYZ> ptCloud;
	
	pcl_conversions::toPCL (msg, pclMsg);
	pcl::fromPCLPointCloud2 (pclMsg, ptCloud);
	
	cout << "Pt_cloud Size: " << ptCloud.size() << endl << endl;
	
	//BOOST_FOREACH (const pcl::PointXYZ& pt, ptCloud)
   // printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
	
	pgf.extract_features(ptCloud);
	pgf.publish_voxels(ptCloudPub);
	
	for (int i=0; i < 1000; i++)
	pgf.generative_model(5);
	
	ros::Duration elapsed = ros::Time::now() - begin;
	ROS_INFO("Time Elapsed: %f", elapsed.toSec());
	
	//float densityWeight, nearnessWeight;
	//pgf.evidence_weight(1, densityWeight, nearnessWeight);
	
	//cout << "Density Weight: " << densityWeight << endl;
	//cout << "Nearness Weight: " << nearnessWeight << endl;
	
}
