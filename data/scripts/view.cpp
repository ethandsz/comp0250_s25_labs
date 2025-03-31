
#include <cmath>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/harris_3d.h>
#include <iostream>
#include <cstdlib>  

// Callback function to display the picked point's coordinates
void pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
    if (event.getPointIndex() == -1) return;  // No point selected
    
    float x, y, z;
    event.getPoint(x, y, z);
    std::cout << "Point coordinates: x = " << x << ", y = " << y << ", z = " << z << std::endl;
}

void visualizePointCloudWithCorners(std::string objId) 
{
    std::string pointCloudFileName = "../object" + objId + ".pcd";
    std::string pointCloudInfoFileName = "../objectInfo" + objId + ".txt";

    std::ifstream inFile(pointCloudInfoFileName);  
    float x, y, z, width;
    int shapeType;
    inFile >> x >> y >> z >> width >> shapeType;
    inFile.close();  

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pointCloudFileName, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file object-transformed.pcd \n");
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr augmentedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    *augmentedCloud = *cloud;
    
    float max_z = -std::numeric_limits<float>::max();
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        if (cloud->points[i].z > max_z)
            max_z = cloud->points[i].z;
    }
    
    // Define how close a point should be to the max z to be considered "top"
    float tolerance = 0.005f;
    
    // Parameters for extrusion: how many layers to add and the vertical spacing
    int numLayers = 5;
    float layerSpacing = 0.0025f;  // the amount by which z is decreased each layer
    
    // Loop through the cloud, and for points near the top, add extra points below them
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        const pcl::PointXYZ& pt = cloud->points[i];
        // Check if the point's z is within the tolerance of the highest point
        if ( (max_z - pt.z) < tolerance) 
        {
            // For each top point, add several layers below it
            for (int layer = 1; layer <= numLayers; ++layer)
            {
                pcl::PointXYZ newPt = pt;
                newPt.z = pt.z - layer * layerSpacing;
                augmentedCloud->points.push_back(newPt);
            }
        }
    }
    
    // Update cloud dimensions (organized clouds require width * height = total points; if not organized, leave as is)
    augmentedCloud->width = augmentedCloud->points.size();
    augmentedCloud->height = 1;
    

    pcl::PointCloud<pcl::PointXYZI>::Ptr corners(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> harris;

    harris.setInputCloud(augmentedCloud);
    harris.setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::TOMASI);
    harris.setRadius(0.01);
    harris.setNonMaxSupression(true);
    harris.setThreshold(0.1);
    harris.compute(*corners);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cornerCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr centroidCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lowestPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cornerToProjectOnCloud(new pcl::PointCloud<pcl::PointXYZ>);

    std::pair<float, float> cornerToProjectOn(-100.0f, -100.0f);
    std::pair<float, float> lowPointYAxis(100.0f, 100.0f);

    float yLineToleranceMin = y - 0.025 * fabs(y);

    float xLineToleranceMin = x - 0.0025 * fabs(x);
    float xLineToleranceMax = x + 0.0025 * fabs(x);


    std::cout << "XLineToleranceMin: " << xLineToleranceMin << std::endl;
    std::cout << "XLineToleranceMax: " << xLineToleranceMax << std::endl;

    std::cout << "x: " << x << std::endl;
    if(shapeType == 0){
      lowPointYAxis.first = x;
      lowPointYAxis.second = 100.0f;
      for (const auto& point : corners->points)
      {

              cornerCloud->push_back(pcl::PointXYZ(point.x, point.y, point.z));
          if(point.x > x && point.y < y + 0.01){
            if(point.x > cornerToProjectOn.first){
              cornerToProjectOn.first = point.x;
              cornerToProjectOn.second = point.y;

              /*cornerCloud->clear();*/
            }
            
          }

      }
      
    for(const auto& point: cloud->points){
            if(point.y < lowPointYAxis.second && point.x > xLineToleranceMin && point.x < xLineToleranceMax){
            lowPointYAxis.second = point.y;
        }
    }
  }
  else{
    float tolX = 0.2;
    float tolY = 0.2;

    float max_x = x + tolX * fabs(width);
    float min_x = x - tolX * fabs(width);

    float max_y = y + tolY * fabs(width);
    float min_y = y - tolY * fabs(width);

    std::cout << "yLineToleranceMin: " << yLineToleranceMin << std::endl;

    std::cout << "y: " << y << std::endl;
    std::cout << "Max y: " << max_y << std::endl;
    for (const auto& point : corners->points)
    {


        if((point.x < max_x && point.x > min_x) && (point.y < max_y && point.y > min_y)){
            if((point.x < lowPointYAxis.first) && (point.y < yLineToleranceMin) && (point.x > xLineToleranceMin)){
              lowPointYAxis.first = point.x; 
              lowPointYAxis.second = point.y; 
            }  
            

            cornerCloud->push_back(pcl::PointXYZ(point.x, point.y, point.z));
            /*cornerCloud->clear();*/

            if(point.x > cornerToProjectOn.first){
            cornerToProjectOn.first = point.x;
            cornerToProjectOn.second = point.y;

          }
        }
    }
  }

    
    std::cout << "Lowest Point: " << lowPointYAxis.first << " ," << lowPointYAxis.second << std::endl;
    std::cout << "Corner Point: " << cornerToProjectOn.first << " ," << cornerToProjectOn.second << std::endl;
    float angleRadians = atan2((cornerToProjectOn.second - lowPointYAxis.second), (cornerToProjectOn.first - lowPointYAxis.first));
    std::cout << "Angle in radians: " << angleRadians << std::endl;
    std::cout << "Angle in degrees: " << (angleRadians * 180 / M_PI) << std::endl;

    centroidCloud->push_back(pcl::PointXYZ(x,y, z));
    cornerToProjectOnCloud->push_back(pcl::PointXYZ(cornerToProjectOn.first, cornerToProjectOn.second, z));
    lowestPointCloud->push_back(pcl::PointXYZ(lowPointYAxis.first, lowPointYAxis.second, z));
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


    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cornerToProjectOnColor(cornerToProjectOnCloud, 255, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cornerToProjectOnCloud, cornerToProjectOnColor, "cornerToProjectOn cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "cornerToProjectOn cloud");

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <objId>" << std::endl;
        return -1;
    }

    std::string objId = (argv[1]);
    std::cout << "Using objId: " << objId << std::endl;

    visualizePointCloudWithCorners(objId);

    return 0;
}
