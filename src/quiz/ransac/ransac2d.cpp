/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

using namespace std;
using namespace pcl;

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

unordered_set<int> RansacPlane(PointCloud<PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	unordered_set<int> inliersResult;
	srand(time(NULL));

	auto startTime = chrono::steady_clock::now();
	unordered_set<int> tempInliers;
	// Get the cloud point size
	int cloud_size = cloud->points.size();

	// For max iterations 
	while (maxIterations--)
	{
		// Clear our the set before performing any operation
		tempInliers.clear();
		unordered_set<int> sample_index;
		// Get two points index for fitting a line
		// Randomly sample subset and fit line
		sample_index.insert(rand()%cloud_size);
		sample_index.insert(rand()%cloud_size);
		sample_index.insert(rand()%cloud_size);

		// We want to make sure all 3 points are unique and are not the same. 
		if (sample_index.size() != 3)
		{
			maxIterations++;
			continue;
		}

		// Time to fit the line and calculate distance
		unordered_set<int>::iterator itr = sample_index.begin();
		
		PointXYZ pt1 = cloud->points[*itr++];
		PointXYZ pt2 = cloud->points[*itr++];
		PointXYZ pt3 = cloud->points[*itr];

		//Plane formula
		// Ax+By+Cz+D = 0
		//Distance d = |Ax+By+Cz+D|/sqrt(A^2+B^2+C^2)
		//A = (pt2.y - pt1.y)*(pt3.z - pt1.z) - (pt2.z - pt1.z)*(pt3.y - pt1.y)
		//B = (pt2.z - pt1.z)*(pt3.x - pt1.x) - (pt2.x - pt1.x)*(pt3.z - pt1.z)
		//C = (pt2.x - pt1.x)*(pt3.y - pt1.y) -  (pt2.y - pt1.y)*(pt3.x - pt1.x) 

		float a = (pt2.y - pt1.y)*(pt3.z - pt1.z) - (pt2.z - pt1.z)*(pt3.y - pt1.y);
		float b = (pt2.z - pt1.z)*(pt3.x - pt1.x) - (pt2.x - pt1.x)*(pt3.z - pt1.z);
		float c = (pt2.x - pt1.x)*(pt3.y - pt1.y) -  (pt2.y - pt1.y)*(pt3.x - pt1.x);
		float d = -(a * pt1.x + b * pt1.y + c * pt1.z);


		// Run through all the points in the cloud to fit the line
		for (int i=0; i < cloud_size; i++)
		{
			PointXYZ pt = cloud->points[i];
			float dist = abs(a*pt.x + b*pt.y + c*pt.z + d)/sqrt (a*a + b*b + c*c);

			// Measure distance between every point and fitted line
			// If distance is smaller than threshold count it as inlier
			if (dist < distanceTol) 
			{
				tempInliers.insert(i);
			}
		}

		// Return indicies of inliers from fitted line with most inliers
		if (tempInliers.size() > inliersResult.size())
		{
			inliersResult = tempInliers;
		}
		
	}

	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac took " << elapsedTime.count() << " milliseconds" << std::endl;
	
	return inliersResult;

}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	auto startTime = std::chrono::steady_clock::now();
	
	// TODO: Fill in this function
	unordered_set<int> tempInliers;
	// Get the cloud point size
	int cloud_size = cloud->points.size();

	// For max iterations 
	while (maxIterations--)
	{
		// Clear our the set before performing any operation
		tempInliers.clear();
		// Get two points index for fitting a line
		// Randomly sample subset and fit line
		int pt1_index = rand()%cloud_size;
		int pt2_index = rand()%cloud_size; 

		// We want to make sure that pt1_index and pt2_index are not the same. If they 
		// are the same we will be a picking only one point and not be able to fit a line
		if (pt1_index == pt2_index)
		{
			maxIterations++;
			continue;
		}

		// Time to fit the line and calculate distance
		PointXYZ pt1 = cloud->points[pt1_index];
		PointXYZ pt2 = cloud->points[pt2_index];

		//Line formula
		// Ax+By+C = 0
		//Distance d = |Ax+By+C|/sqrt(A^2+B^2)
		//A = pt1.y - pt2.y
		//B = pt1.x - pt2.x
		//C = pt1.x * pt2.y - pt2.x * pt1.y

		float a = (pt1.y - pt2.y);
		float b = (pt2.x - pt1.x);
		float c = (pt1.x * pt2.y - pt2.x * pt1.y);


		// Run through all the points in the cloud to fit the line
		for (int i=0; i < cloud_size; i++)
		{
			PointXYZ pt = cloud->points[i];
			float dist = abs(a*pt.x + b*pt.y + c)/sqrt (a*a + b*b);

			// Measure distance between every point and fitted line
			// If distance is smaller than threshold count it as inlier
			if (dist < distanceTol) 
			{
				tempInliers.insert(i);
			}
		}

		// Return indicies of inliers from fitted line with most inliers
		if (tempInliers.size() > inliersResult.size())
		{
			inliersResult = tempInliers;
		}
		
	}

	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac took " << elapsedTime.count() << " milliseconds" << std::endl;
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.4);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
