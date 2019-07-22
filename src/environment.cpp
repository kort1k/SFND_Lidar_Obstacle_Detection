/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <memory>

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,
    ProcessPointClouds<pcl::PointXYZI>* processorPointI,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud
    )
{
    //ProcessPointClouds<pcl::PointXYZI>* processorPointI =  new ProcessPointClouds<pcl::PointXYZI>();
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = processorPointI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = processorPointI->FilterCloud(
        inputCloud, 0.20,
        Eigen::Vector4f( -80, -5.8, -2, 1),  // 4.5 as we drive on right side. At right handed driver pay attention 
        Eigen::Vector4f( 220,  7.2,  1.8, 1)
        ); 

    //renderPointCloud(viewer, inputCloud, "cityCloud");

    // render road 
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = processorPointI->SegmentPlane(filteredCloud, 100, 0.2);
    
    renderPointCloud(viewer, segmentCloud.second, "roadCload", Color(0,1,0));

    // render cars/obst
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = processorPointI->Clustering(segmentCloud.first, .45, 20, 1000);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1), 
                                Color(0.5,1,0), Color(0,1,1), Color(1,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
            std::cout << "cluster size ";
            processorPointI->numPoints(cluster);
            renderPointCloud(viewer, cluster, "Car" + std::to_string(clusterId), 
                colors[clusterId % (colors.size()-1) ]);
            Box box = processorPointI->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
            ++clusterId;
    }

}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    std::shared_ptr<Lidar> lidar(new Lidar(cars, 0.0));

    // TODO:: Create point processor
    pcl::PointCloud<pcl::PointXYZ>::Ptr rays = lidar->scan();
    //renderRays(viewer, lidar->position, rays);
    renderPointCloud(viewer, rays, "aa", Color(100,100,100));
  
  // kort1k
  ProcessPointClouds<pcl::PointXYZ> ppc;
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = ppc.SegmentPlane(rays, 100, 0.2);
  // renderPointCloud(viewer, segmentCloud.first, "obstCload", Color(1,0,0));
  // renderPointCloud(viewer, segmentCloud.second, "roadCload", Color(0,1,0));
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = ppc.Clustering(segmentCloud.first, 1.0, 3, 30);

  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

  for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
  {
        std::cout << "cluster size ";
    	ppc.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);
        Box box = ppc.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
  }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    
    // project
    ProcessPointClouds<pcl::PointXYZI>* processorPointI =  new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = processorPointI->streamPcd("../src/sensors/data/pcd/data_1");
    auto si = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped() ) {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        inputCloudI = processorPointI->loadPcd((*si).string());

        cityBlock(viewer, processorPointI, inputCloudI);

        si++;
        if (si == stream.end())
            si = stream.begin();

        viewer->spinOnce(100);
    }
 
}
