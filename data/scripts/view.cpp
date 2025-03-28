
#include <cmath>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/harris_3d.h>
#include <iostream>

// Callback function to display the picked point's coordinates
void pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
    if (event.getPointIndex() == -1) return;  // No point selected
    
    float x, y, z;
    event.getPoint(x, y, z);
    std::cout << "Point coordinates: x = " << x << ", y = " << y << ", z = " << z << std::endl;
}

void visualizePointCloudWithCorners()
{
    /*x: 0.432307, y: -0.326420, z: 0.049738*/
    float x = 0.432307, y = -0.326420, z = 0.049738, width = 0.127893;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../object.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file object-transformed.pcd \n");
        return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr corners(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> harris;
    harris.setInputCloud(cloud);
    harris.setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::HARRIS);
    harris.setRadius(0.01);
    harris.setThreshold(1e-4);
    harris.compute(*corners);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cornerCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr centroidCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lowestPointCloud(new pcl::PointCloud<pcl::PointXYZ>);

    std::pair<float, float> cornerToProjectOn(-100.0f, -100.0f);
    for (const auto& point : corners->points)
    {
        if(point.x > x && point.y < y + 0.05){
          if(point.x > cornerToProjectOn.first){
            cornerToProjectOn.first = point.x;
            cornerToProjectOn.second = point.y;

            cornerCloud->clear();
            cornerCloud->push_back(pcl::PointXYZ(point.x, point.y, point.z));
          }
        }
    }

    std::pair<float, float> lowPointYAxis(x, y- width/2);
    
    std::cout << "Lowest Point: " << lowPointYAxis.first << " ," << lowPointYAxis.second << std::endl;
    std::cout << "Corner Point: " << cornerToProjectOn.first << " ," << cornerToProjectOn.second << std::endl;
    float angleRadians = atan2((cornerToProjectOn.second - lowPointYAxis.second), (cornerToProjectOn.first - lowPointYAxis.first));
    std::cout << "Angle in radians: " << angleRadians << std::endl;
    std::cout << "Angle in degrees: " << (angleRadians * 180/M_PI) << std::endl;

    centroidCloud->push_back(pcl::PointXYZ(x, y, z));
    lowestPointCloud->push_back(pcl::PointXYZ(x, y - width/2, z));
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Harris Corner Detection"));
    viewer->setBackgroundColor(0, 0, 0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloudColor(cloud, 255, 255, 255);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, cloudColor, "original cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cornerColor(cornerCloud, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cornerCloud, cornerColor, "corner cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "corner cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> centroidColor(centroidCloud, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(centroidCloud, centroidColor, "centroid cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "centroid cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> lowestPointColor(lowestPointCloud, 0, 0, 255);
    viewer->addPointCloud<pcl::PointXYZ>(lowestPointCloud, lowestPointColor, "lowestPoint cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "lowestPoint cloud");

    // Register the point picking callback function
    viewer->registerPointPickingCallback(pointPickingCallback, nullptr);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
}

int main()
{
    visualizePointCloudWithCorners();
    return 0;
}
