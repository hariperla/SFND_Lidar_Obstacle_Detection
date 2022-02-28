// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>

#include "../src/quiz/ransac/ransacplane.h"
#include "../src/quiz/cluster/clusterhelp.cpp"

using namespace std;


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> voxGrid;
    typename pcl::PointCloud<PointT>::Ptr filtCloud(new pcl::PointCloud<PointT>);
    voxGrid.setInputCloud(cloud);
    voxGrid.setLeafSize(filterRes,filterRes,filterRes);
    voxGrid.filter(*filtCloud);

    typename pcl::PointCloud<PointT>::Ptr roiCloud(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> roi(true);
    roi.setMin(minPoint);
    roi.setMax(maxPoint);
    roi.setInputCloud(filtCloud);
    roi.filter(*roiCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return roiCloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  typename pcl::PointCloud<PointT>::Ptr objCloud (new pcl::PointCloud<PointT> ());
  typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());

  for (int index : inliers->indices)
    planeCloud->points.push_back(cloud->points[index]);

  // Create extraction object
  pcl::ExtractIndices<PointT> extract;
  
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*objCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(objCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold, bool useDefault)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    // TODO:: Fill in this function to find inliers for the cloud.
    // To switch between pcl library segmentation and clustering vs written

    if (useDefault)
    {
        // Create segmentation object
        pcl::SACSegmentation<PointT> seg;
        
        // Create other model parameters
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(maxIterations);
        seg.setDistanceThreshold(distanceThreshold);

        // Segment the plane/road vs cars
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
    }
    else
    {
        std::unordered_set<int> inliersResult = RANSACPLANE<PointT>::RansacPlane(cloud, maxIterations, distanceThreshold);

        for (int index:inliersResult)
        {
            inliers->indices.push_back(index);
        }
    }
    
    if (inliers->indices.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize, bool useDefault)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    if (useDefault)
    {
        typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
        tree->setInputCloud(cloud);

        std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
        std::vector<pcl::PointIndices> clusterIndices;

        pcl::EuclideanClusterExtraction<PointT> eucClus;
        eucClus.setClusterTolerance(clusterTolerance);
        eucClus.setMinClusterSize(minSize);
        eucClus.setMaxClusterSize(maxSize);
        eucClus.setSearchMethod(tree);
        eucClus.setInputCloud(cloud);
        eucClus.extract(clusterIndices);
    
        for(pcl::PointIndices getIndices: clusterIndices)
        {
            typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

            for (int index: getIndices.indices)
            {
                cloudCluster->points.push_back (cloud->points[index]);
            }

            cloudCluster->width = cloudCluster->points.size();
            cloudCluster->height = 1;
            cloudCluster->is_dense = true;

            clusters.push_back(cloudCluster);
        }
    }
    else
    {
        KdTree *tree = new KdTree;

        // This is a float of vectors of cluster points
        std::vector<std::vector<float>> points;

        for (int i=0; i <= cloud->points.size(); i++)
        {
            std::vector<float> point = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
            // Insert points in the tree and onto the points vector for running through the cluster function
            tree->insert(point,i);
            points.push_back(point);
        }

        // Obtain the clusters from the tree
        const std::vector<std::vector<int>> treeClusters = euclideanCluster(points, tree, clusterTolerance);

        // Loop through the tree and find all the points within our distance threshold
        for (std::vector<int> clusterIDs:treeClusters)
        {
            // Grab clusters within our min and max range
            if ((clusterIDs.size() >= minSize) && (clusterIDs.size() <= maxSize))
            {
                typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

                //Loop through cluster id's and add points to the cloud cluster
                for (int index:clusterIDs)
                {
                    cloudCluster->points.push_back(cloud->points[index]);
                }

                cloudCluster->width = cloudCluster->points.size();
                cloudCluster->height = 1;
                cloudCluster->is_dense = true;

                clusters.push_back(cloudCluster);
            }
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template <typename PointT>
BoxQ ProcessPointClouds<PointT>::RotatingBoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    BoxQ box;

    // To fit the smallest box for given cluster, we need to use PCA.
    // Use PCA to get the pricinpal components for the cluster/cloud
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);

    // Using Codex Technicanum idea of rotating bounding box to get the smallest box around the points
    typename pcl::PointCloud<PointT>::Ptr cloudPCAprojection (new pcl::PointCloud<PointT>);
    pcl::PCA<PointT> pca;
    pca.setInputCloud(cluster);
    pca.project(*cluster, *cloudPCAprojection);

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix3f eigenVectorsPCA = pca.getEigenVectors();
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());

    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *cloudPCAprojection, projectionTransform);

    // Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations 
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();   

    // Set bounding box parameters
    box.bboxQuaternion = bboxQuaternion;
    box.bboxTransform = bboxTransform;
    box.cube_length = maxPoint.x - minPoint.x;
    box.cube_width = maxPoint.y - minPoint.y;
    box.cube_height = maxPoint.z - minPoint.z;

    return box;   
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}