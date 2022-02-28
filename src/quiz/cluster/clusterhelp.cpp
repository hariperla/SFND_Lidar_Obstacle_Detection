/* Author: Hari Perla */

#include "clusterhelp.h"

void proximityClusterHelp(KdTree* tree, const std::vector<std::vector<float>> &points,std::vector<int> &cluster,int id,
				   std::vector<bool> &isProcessed,float distanceTol)
{
	isProcessed[id] = true; // Mark the point as processed

	cluster.push_back(id); // Add point to the cluster

	std::vector<int> nearbyIDs = tree->search(points[id],distanceTol); // Get nearby points

	// Iterate through each point
	for (int point_idx : nearbyIDs)
	{
		if (!isProcessed[point_idx])
		{
			proximityClusterHelp(tree, points, cluster, point_idx, isProcessed, distanceTol);
		}
	}
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{
	// TODO: Fill out this function to return list of indices for each cluster
	std::vector<std::vector<int>> clusters;
	std::vector<bool> pointProcessed(points.size(),false); // Initialize points prcessed to false and update after each iteration

	// Iterate through each point and add clusters
	for (int i = 0; i < points.size(); i++)
	{
		if (pointProcessed[i])
		{
			continue;
		}
		std::vector<int> cluster; // Create a cluster
		proximityClusterHelp(tree, points, cluster, i, pointProcessed, distanceTol); // Recursive clustering

		clusters.push_back(cluster); // Add the new found cluster to the clusters
	}
	return clusters;
}