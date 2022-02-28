/* Author: Hari Perla */

#ifndef _RANSAC_PLANE_H
#define _RANSAC_PLANE_H

#include <pcl/common/common.h>
#include <unordered_set>

template<typename PointT>
class RANSACPLANE{
	public:
	static std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
	{
		std::unordered_set<int> inliersResult;
		srand(time(NULL));

		auto startTime = std::chrono::steady_clock::now();
		std::unordered_set<int> tempInliers;
		// Get the cloud point size
		int cloud_size = cloud->points.size();

		// For max iterations 
		while (maxIterations--)
		{
			// Clear our the set before performing any operation
			tempInliers.clear();
			std::unordered_set<int> sample_index;
			// Get two points index for fitting a line
			// Randomly sample subset and fit line
			sample_index.insert(rand()%cloud_size);
			sample_index.insert(rand()%cloud_size);
			sample_index.insert(rand()%cloud_size);

			// We want to make sure all 3 points are not the same. 
			if (sample_index.size() != 3)
			{
				maxIterations++;
				continue;
			}

			// Time to fit the line and calculate distance
			std::unordered_set<int>::iterator itr = sample_index.begin();
			
			PointT pt1 = cloud->points[*itr++];
			PointT pt2 = cloud->points[*itr++];
			PointT pt3 = cloud->points[*itr];

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
				PointT pt = cloud->points[i];
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
		std::cout << "Ransac plane took " << elapsedTime.count() << " milliseconds" << std::endl;
		
		return inliersResult;

	};
};

#endif