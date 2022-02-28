/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void demoCode(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZI> pointProcessorI;

    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    // Call the filter function to downsample the huge pcd file
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtData = pointProcessorI.FilterCloud(inputCloud, 0.3, 
                                                    Eigen::Vector4f(-22,-6,-3.5,1.0), Eigen::Vector4f(25,6.5,3,1.0));
    // render the pcd file
    renderPointCloud(viewer, filtData, "Filtered Cloud Data");

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudSegment = pointProcessorI.SegmentPlane(filtData,30,0.3,false);
    renderPointCloud(viewer,cloudSegment.first,"ObjCloud",Color(1,1,1));

    vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI.Clustering(cloudSegment.first, 0.2, 20, 350, false);
    int clusterId;

    // Define colors for each cluster
    vector<Color> colors = {Color(1, 1, 0), Color(0, 1, 1), Color(1, 0, 1)};
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        cout << "Cluster size";
        pointProcessorI.numPoints(cluster);
        renderPointCloud(viewer, cluster, "ObjCloud" + to_string(clusterId), colors[clusterId]);
        // Add a bounding box
        Box box = pointProcessorI.BoundingBox(cluster);
        //BoxQ box = pointProcessorI.RotatingBoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // ----- Open 3D viewer and display City Block --------
    // ----------------------------------------------------

    // Call the filter function to downsample the huge pcd file
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtData = pointProcessorI->FilterCloud(inputCloud, 0.2, 
                                                    Eigen::Vector4f(-22,-6,-3.5,1.0), Eigen::Vector4f(25,6.5,3,1.0));
    // render the pcd file
    renderPointCloud(viewer, filtData, "Filtered Cloud Data");

    // Segment object plane vs road plane
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudSegment = pointProcessorI->SegmentPlane(filtData,25,0.2,false);
    renderPointCloud(viewer,cloudSegment.first,"ObjCloud",Color(1,1,1));

    vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(cloudSegment.first, 0.2, 10, 250, false);
    int clusterId;

    // Define colors for each cluster
    vector<Color> colors = {Color(1, 1, 0), Color(0, 1, 1), Color(1, 0, 1)};
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        cout << "Cluster size";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "ObjCloud" + to_string(clusterId), colors[clusterId]);
        // Add a bounding box
        Box box = pointProcessorI->BoundingBox(cluster);
        //BoxQ box = pointProcessorI.RotatingBoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
}

/* void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = true;
    bool useDefaultMethods = true;

    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    Lidar *lidar = new Lidar(cars, 0.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr dataCloud = lidar->scan();
    // renderRays(viewer, lidar->position, dataCloud);
    renderPointCloud(viewer, dataCloud, "DataCloud", Color(1, 0, 0));

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(dataCloud, 100, 0.3);
    renderPointCloud(viewer, segmentCloud.first, "ObjCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "PlaneCloud", Color(0, 1, 0));

    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 2.0, 3, 30);
    int clusterId;

    // Define colors for each cluster
    vector<Color> colors = {Color(1, 1, 0), Color(0, 1, 1), Color(1, 0, 1)};
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        cout << "Cluster size";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "ObjCloud" + to_string(clusterId), colors[clusterId]);
        // Add a bounding box
        // Box box = pointProcessor.BoundingBox(cluster);
        BoxQ box = pointProcessor.RotatingBoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
}
 */
// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    
    //simpleHighway(viewer);
    //cityBlock(viewer);
    demoCode(viewer);

    while (!viewer->wasStopped())
    {
        // Clear viewer
        /* viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin(); */
        viewer->spinOnce();
    }
}