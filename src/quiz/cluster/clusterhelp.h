/* Author: Hari Perla */

#ifndef _CLUSTER_HELP_H
#define _CLUSTER_HELP_H

#include "kdtree.h"

void proximityClusterHelp(KdTree* tree, const std::vector<std::vector<float>> &points,std::vector<int> &cluster,int id,
				          std::vector<bool> &isProcessed,float distanceTol);

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol);

#endif