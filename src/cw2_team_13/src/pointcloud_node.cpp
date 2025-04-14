#include <pcl/keypoints/harris_3d.h>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Geometry/Transform.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TransformStamped.h"
#include "ros/console.h"
#include "ros/publisher.h"
#include "ros/service_client.h"
#include "ros/subscriber.h"
#include <cstdio>
#include <cw2_class.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/search/kdtree.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <ros/package.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include "cw2_team_13/set_arm_cart.h"
#include "cw2_team_13/set_arm.h"
#include "cw2_team_13/map_env.h"
#include "std_msgs/ColorRGBA.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <geometry_msgs/PoseArray.h>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_ros/transforms.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl/segmentation/extract_clusters.h>
#include <utility>
#include <vector>
#include <helper_methods.h>
#include <pcl/common/pca.h>
#include <pcl/registration/icp.h>
#include <pcl/common/common.h>
#include "cw2_team_13/ObjectInfo.h"
#include <fstream>
#include <cstdlib>

// Enum to define the types of objects that can be detected in the environment
enum ObjectType
{
  Nought,   // 0 - Square/ring shape
  Cross,    // 1 - Cross shape
  Obstacle, // 2 - Black obstacles to avoid
  Box       // 3 - Basket/box for placing objects
};

// Utility function to convert ObjectType enum to string representation for logging
const char *objectTypeToString(ObjectType type)
{
  switch (type)
  {
  case Nought:
    return "Nought";
  case Cross:
    return "Cross";
  case Obstacle:
    return "Obstacle";
  case Box:
    return "Box";
  default:
    return "Unknown";
  }
}

// Struct to store all relevant data for a detected object
struct ObjectData
{
  Eigen::Vector3f objPointInCartesianSpace;       // 3D position of the object (centroid)
  Eigen::Vector4f objectOrientation;              // Orientation as quaternion
  float width;                                    // Width of the object
  float height;                                   // Height of the object
  std::pair<float, float> detectedCornerPosition; // Position of a detected corner (for orientation calculation)
  Eigen::Vector3i rgbValue;                       // RGB color of the object
  ObjectType objType;                             // Type of the object (Nought, Cross, etc.)

  // Constructor to initialize all members
  ObjectData(Eigen::Vector3f &objPointInCartesianSpace,
             Eigen::Vector4f &objectOrientation,
             float &width,
             float &height,
             std::pair<float, float> &detectedCornerPosition,
             Eigen::Vector3i &rgbValue,
             ObjectType objType)
      : objPointInCartesianSpace(objPointInCartesianSpace),
        objectOrientation(objectOrientation),
        width(width),
        height(height),
        detectedCornerPosition(detectedCornerPosition),
        rgbValue(rgbValue),
        objType(objType) {}
};

// ROS publishers and service clients declaration
ros::Publisher pointCloudPublisher;   // Publisher for processed point cloud
ros::Publisher objectMarkerPublisher; // Publisher for visualization markers
ros::Publisher objectPosePublisher;   // Publisher for object poses

ros::ServiceClient set_arm_cart_client_;                                                     // Client for Cartesian path arm movement service
ros::ServiceClient set_arm_client_;                                                          // Client for general arm movement service
pcl::PointCloud<pcl::PointXYZRGB>::Ptr completeCloud(new pcl::PointCloud<pcl::PointXYZRGB>); // Combined point cloud from all scans
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);         // Current point cloud from sensor
pcl::PointCloud<pcl::PointXYZRGB>::Ptr knownModel(new pcl::PointCloud<pcl::PointXYZRGB>);    // For model matching (unused in this code)

// Function to remove the planar surface (ground plane) from point cloud
void removePlaneSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PointIndices::Ptr inliers_plane)
{
  // Find Plane using RANSAC
  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segmentor;
  segmentor.setOptimizeCoefficients(true);
  segmentor.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  segmentor.setMethodType(pcl::SAC_RANSAC);

  // Setting Z-axis as normal direction for the plane
  Eigen::Vector3f axis = Eigen::Vector3f(0.0, 0.0, 1.0);
  segmentor.setAxis(axis);
  segmentor.setMaxIterations(100);
  segmentor.setDistanceThreshold(0.02);
  segmentor.setEpsAngle(0.1);
  segmentor.setNormalDistanceWeight(0.1);
  segmentor.setInputCloud(cloud);
  segmentor.setInputNormals(cloud_normals);

  // Output plane coefficients
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
  segmentor.segment(*inliers_plane, *coefficients_plane);

  /* Extract the planar inliers from the input cloud */
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
  extract_indices.setInputCloud(cloud);
  extract_indices.setIndices(inliers_plane);

  /* Remove the planar inliers, extract the rest */
  extract_indices.setNegative(true);
  extract_indices.filter(*cloud);
}

// Function to filter the point cloud by z-height (remove points too high or too low)
void filterCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 0.67); // Keep points between 0 and 67cm height
  pass.filter(*cloud);
}

// Extract normals that don't belong to the plane
void extractNormals(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PointIndices::Ptr inliers_plane)
{
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  extract_normals.setNegative(true);
  extract_normals.setInputCloud(cloud_normals);
  extract_normals.setIndices(inliers_plane);
  extract_normals.filter(*cloud_normals);
}

// Calculate surface normals for the point cloud
void calcNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud);
  // Set the number of k nearest neighbors to use for the feature estimation.
  ne.setKSearch(100);
  ne.compute(*cloud_normals);
}

// Filter out specific colors (green for grass tiles and gray for irrelevant objects)
void filterColors(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorFilteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (size_t i = 0; i < cloud->points.size(); i++)
  {
    Eigen::Vector3i colorVec = cloud->points[i].getRGBVector3i();
    uint8_t red = colorVec[0];
    uint8_t green = colorVec[1];
    uint8_t blue = colorVec[2];

    // More precise green detection (grass tiles)
    bool isGreen = (green > red + 20) && (green > blue + 20) && (green > 100);

    // Better gray detection (includes a wider range of gray values)
    bool isGray = (std::abs(red - green) < 20) &&
                  (std::abs(green - blue) < 20) &&
                  (std::abs(red - blue) < 20) &&
                  (red + green + blue > 300); // Light grays

    // Black obstacle detection as specified (RGB=[0.1, 0.1, 0.1])
    bool isBlackObstacle = (red < 30) && (green < 30) && (blue < 30);

    // If it's not green or gray
    if (!isGreen && !isGray)
    {
      // If it's a black obstacle, you could set a flag or property
      if (isBlackObstacle)
      {
        // Example: Set a specific channel or property to mark it as an obstacle
        pcl::PointXYZRGB point = cloud->points[i];
        point.r = 1; // Mark with specific color for later identification
        point.g = 1;
        point.b = 1;
        colorFilteredCloud->points.push_back(point);
      }
      else
      {
        // Regular object
        colorFilteredCloud->points.push_back(cloud->points[i]);
      }
    }
  }
  cloud->swap(*colorFilteredCloud);
}

/////////////////////////////////////////////////////////////////////////////

// Callback function for the RealSense camera point cloud topic
void realSenseCallback(const sensor_msgs::PointCloud2ConstPtr &input)
{
  // Convert the ROS message to a PCL point cloud
  pcl::fromROSMsg(*input, *cloud);
}

// Extract and classify objects from the point cloud
std::vector<ObjectData> extractObjectsInScene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  std::vector<ObjectData> objects;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  kdTree->setInputCloud(cloud);
  std::vector<pcl::PointIndices> cluster_indices;

  // Euclidean cluster extraction to separate objects
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(0.005); // 5mm
  ec.setMinClusterSize(75);      // Minimum number of points per cluster
  ec.setMaxClusterSize(25000);   // Maximum number of points per cluster
  ec.setSearchMethod(kdTree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  int objId = 0;
  // Process each cluster (potential object)
  for (size_t i = 0; i < cluster_indices.size(); i++)
  {
    Eigen::Vector3f centroid(0, 0, 0);
    Eigen::Vector3i rgbValue(0, 0, 0);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectCluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    // Extract points for this cluster
    for (int idx : cluster_indices[i].indices)
    {
      objectCluster->points.push_back(completeCloud->points[idx]);
      auto point = completeCloud->points[idx];
    }

    // Calculate min and max boundaries of the object
    Eigen::Vector4f minPoint, maxPoint;
    pcl::getMinMax3D(*objectCluster, minPoint, maxPoint);
    float objLength = maxPoint[0] - minPoint[1];
    float objWidth = maxPoint[1] - minPoint[1];
    float objHeight = maxPoint[2] - minPoint[2];

    // Calculate centroid using points near the top of the object
    int pointsAddedToCentroid = 0;
    for (size_t i = 0; i < objectCluster->points.size(); i++)
    {
      if (objectCluster->points[i].z >= maxPoint[2] * 0.9)
      {
        pointsAddedToCentroid += 1;
        centroid += Eigen::Vector3f(objectCluster->points[i].x, objectCluster->points[i].y, objectCluster->points[i].z);
        rgbValue += Eigen::Vector3i(objectCluster->points[i].r, objectCluster->points[i].g, objectCluster->points[i].b);
      }
    }

    // setting up object cluster point cloud
    objectCluster->width = objectCluster->points.size();
    objectCluster->height = 1;
    objectCluster->is_dense = true;

    // Normalize the centroid and RGB values
    centroid /= static_cast<float>(pointsAddedToCentroid);
    rgbValue /= cluster_indices[i].indices.size();

    float x = centroid[0], y = centroid[1], z = centroid[2];

    //////////////////////////////////////////////////////////////////
    // if point is dark red (r < 80) and the width is > 50mm
    // then object type = Box

    // Check if the calculated centroid actually exists in the point cloud
    double tolerance = 0.01;
    bool foundPoint = false;
    for (size_t i = 0; i < objectCluster->points.size(); i++)
    {
      if (std::abs(objectCluster->points[i].x - x) <= tolerance &&
          std::abs(objectCluster->points[i].y - y) <= tolerance &&
          std::abs(objectCluster->points[i].z - z) <= tolerance)
      {
        foundPoint = true;
        break;
      }
    }

    // Find the maximum height of the object
    float max_z = -std::numeric_limits<float>::max();
    for (size_t i = 0; i < objectCluster->points.size(); ++i)
    {
      if (objectCluster->points[i].z > max_z)
      {
        max_z = objectCluster->points[i].z;
      }
    }

    // Type of object determination based on various properties
    ObjectType objectType;

    // Check if it's a box (wide enough and in negative x)
    bool isWideEnough = objWidth > 0.3;
    bool isNegative = x < 0;

    if (isWideEnough && isNegative)
    {
      objectType = Box;
    }
    // Otherwise, determine if it's a Cross or Nought based on the existing logic
    else if (foundPoint)
    {
      objectType = Cross;
      if (rgbValue[0] < 5 && rgbValue[1] < 5 && rgbValue[2] < 5 && max_z > 0.07)
      {
        objectType = Obstacle;
      }
    }
    else
    {
      objectType = Nought;
    }

    // Create an augmented point cloud by adding layers below the top surface
    // This helps the Harris corner detector find corners more effectively
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr augmentedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    *augmentedCloud = *objectCluster;

    float toleranceZHeight = 0.005f;
    int numLayers = 5;
    float layerSpacing = 0.0025f; // the amount by which z is decreased each layer

    // Add artificial layers below the top surface
    for (size_t i = 0; i < objectCluster->points.size(); ++i)
    {
      const pcl::PointXYZRGB &pt = objectCluster->points[i];
      if ((max_z - pt.z) < toleranceZHeight)
      {
        for (int layer = 1; layer <= numLayers; ++layer)
        {
          pcl::PointXYZRGB newPt = pt;
          newPt.z = pt.z - layer * layerSpacing;
          augmentedCloud->points.push_back(newPt);
        }
      }
    }

    augmentedCloud->width = augmentedCloud->points.size();
    augmentedCloud->height = 1;

    // Adjust Harris threshold based on object type
    float harrisThreshold = objectType == Cross ? 0.05 : 0.075;

    // Harris corner detection for finding corners/features
    pcl::PointCloud<pcl::PointXYZI>::Ptr corners(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI> harris;
    harris.setInputCloud(augmentedCloud);
    harris.setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::TOMASI);
    harris.setRadius(0.01);
    harris.setNonMaxSupression(true);
    harris.setThreshold(harrisThreshold);
    harris.compute(*corners);

    // Initialize variables for corner detection
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cornerCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::pair<float, float> lowPointYAxis;
    std::pair<float, float> cornerToProjectOn;

    // Define tolerances for finding points on lines
    float yLineToleranceMin = y - 0.005 * fabs(y);
    float xLineToleranceMin = x - 0.005 * fabs(x);
    float xLineToleranceMax = x + 0.005 * fabs(x);

    // Find corners for Nought objects
    if (objectType == Nought)
    {
      lowPointYAxis.first = x;
      lowPointYAxis.second = 100.0f;
      for (const auto &point : corners->points)
      {
        // Find a corner in the positive X direction from centroid
        if (point.x > x && point.y < y + 0.01)
        {
          if (point.x > cornerToProjectOn.first || cornerToProjectOn.first == 0.0)
          {
            cornerToProjectOn.first = point.x;
            cornerToProjectOn.second = point.y;
          }
        }
      }

      // Find the lowest point (in Y) that's near the X-line passing through centroid
      for (const auto &point : objectCluster->points)
      {
        if (point.y < lowPointYAxis.second && point.x > xLineToleranceMin && point.x < xLineToleranceMax && point.z > 0.06)
        {
          lowPointYAxis.second = point.y;
        }
      }
    }
    else
    {
      // Handle corner detection for Cross objects
      float tolX = 0.225;
      float tolY = 0.225;

      // Define search regions relative to the object's width
      float max_x = x + tolX * fabs(objWidth);
      float min_x = x - tolX * fabs(objWidth);

      float max_y = y + tolY * fabs(objWidth);
      float min_y = y - tolY * fabs(objWidth);

      // Calculate a refined centroid from detected corners
      Eigen::Vector2f newCentroid(0, 0);
      int pointsIterated = 0;
      for (const auto &point : corners->points)
      {
        if ((point.x < max_x && point.x > min_x) && (point.y < max_y && point.y > min_y))
        {
          newCentroid[0] += point.x;
          newCentroid[1] += point.y;
          pointsIterated += 1;
        }
      }
      newCentroid[0] /= pointsIterated;
      newCentroid[1] /= pointsIterated;
      x = newCentroid[0];
      y = newCentroid[1];

      // Find specific corners relative to the new centroid
      for (const auto &point : corners->points)
      {
        if ((point.x < max_x && point.x > min_x) && (point.y < max_y && point.y > min_y))
        {
          if (point.x > x && point.y < y + 0.01)
          {
            // Find a corner in the +X, -Y quadrant
            if ((cornerToProjectOn.first == 0.0) || point.x > cornerToProjectOn.first - 0.02 && point.y < cornerToProjectOn.second)
            {
              cornerToProjectOn.first = point.x;
              cornerToProjectOn.second = point.y;
            }
          }

          // Find a point in the -Y direction
          if (point.y < y - 0.01)
          {
            if (lowPointYAxis.first == 0.0 || point.x < lowPointYAxis.first)
            {
              lowPointYAxis.first = point.x;
              lowPointYAxis.second = point.y;
            }
          }
        }
      }
    }

    // Calculate orientation angle from the detected corners
    float angleRadians = atan2((cornerToProjectOn.second - lowPointYAxis.second), (cornerToProjectOn.first - lowPointYAxis.first));

    // Convert the angle to quaternion for pose representation
    double objRoll = 0.0;
    double obPitch = 0.0;
    double objYaw = angleRadians;
    std::vector<double> objQuaternion = HelperMethods::getQuaternionFromEuler(objRoll, obPitch, objYaw);
    Eigen::Vector4f objOrientation(objQuaternion[0], objQuaternion[1], objQuaternion[2], objQuaternion[3]);

    // Check if all the necessary points were found for orientation calculation
    bool allPointsFound = lowPointYAxis.first != 0.0 && lowPointYAxis.second != 0.0 && cornerToProjectOn.first != 0.0 && cornerToProjectOn.second != 0.0;

    // Log object detection information
    ROS_INFO("-------------OBJECT SUMMARY-------------");
    ROS_INFO("ESTIMATED WIDTH OF OBJECT: %f", objWidth);
    ROS_INFO("ESTIMATED TYPE OF OBJECT: %s", objectTypeToString(objectType));

    ROS_INFO("LOWEST POINT: %f, %f", lowPointYAxis.first, lowPointYAxis.second);
    ROS_INFO("CORNER POINT: %f, %f", cornerToProjectOn.first, cornerToProjectOn.second);
    ROS_INFO("CENTROID: %f, %f", centroid[0], centroid[1]);
    ROS_INFO("ANGLE IN RADIANS: %f", angleRadians);
    ROS_INFO("ANGLE IN DEGREES: %f", angleRadians * 180 / M_PI);
    ROS_INFO("MAX Z HEIGHT: %f", maxPoint[2]);

    // Only add valid objects to the list
    if ((objectType == Box || objectType == Obstacle) || !(std::isnan(x) || std::isnan(y) || std::isnan(z)) && (allPointsFound))
    {
      // Special case for Cross objects with identical corner points
      if (objectType == Cross && lowPointYAxis.first == cornerToProjectOn.first && lowPointYAxis.second == cornerToProjectOn.second)
      {
        objRoll = 0.0;
        obPitch = 0.0;
        objYaw = 22.5; // Default orientation in degrees
        objQuaternion = HelperMethods::getQuaternionFromEuler(objRoll, obPitch, objYaw);
        objOrientation[0] = objQuaternion[0];
        objOrientation[1] = objQuaternion[1];
        objOrientation[2] = objQuaternion[2];
        objOrientation[3] = objQuaternion[3];
      }
      // Create and add valid object to the results list
      ObjectData object(centroid, objOrientation, objWidth, maxPoint[2], cornerToProjectOn, rgbValue, objectType);
      objects.push_back(object);
    }

    objId += 1;
  }

  // Print summary of detected objects
  for (size_t i = 0; i < objects.size(); i++)
  {
    ObjectData object = objects[i];
    Eigen::Vector3f position = object.objPointInCartesianSpace;
    Eigen::Vector3i rgb = object.rgbValue;
    ROS_INFO("%s at [x: %f, y: %f, z: %f] with RGB of [%i, %i, %i]", objectTypeToString(object.objType), position[0], position[1], position[2], rgb[0], rgb[1], rgb[2]);
  }
  return objects;
}

// Publish markers and poses for all detected objects
void publishObjectPositions(std::vector<ObjectData> objects)
{
  visualization_msgs::MarkerArray markerArray;
  geometry_msgs::PoseArray poseArray;

  poseArray.header.frame_id = "panda_link0";
  poseArray.header.stamp = ros::Time::now();

  for (size_t i = 0; i < objects.size(); i++)
  {
    ObjectData object = objects[i];
    visualization_msgs::Marker marker;
    visualization_msgs::Marker cornerMarker;
    geometry_msgs::Pose pose;

    Eigen::Vector3f positions = object.objPointInCartesianSpace;

    float x = positions[0], y = positions[1], z = positions[2];

    Eigen::Vector4f objQuat = object.objectOrientation;

    std::pair<float, float> detectedCornerPosition = object.detectedCornerPosition;

    // Create pose message for each object
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z + 0.05; // Slight offset in Z for better grasp point
    pose.orientation.x = objQuat[0];
    pose.orientation.y = objQuat[1];
    pose.orientation.z = objQuat[2];
    pose.orientation.w = objQuat[3];
    poseArray.poses.push_back(pose);

    // Create visualization marker for each object
    marker.header.frame_id = "panda_link0";
    marker.header.stamp = ros::Time::now();

    marker.ns = "obj_half_width";
    marker.id = rand() % 1001; // Random ID to avoid collisions

    marker.type = visualization_msgs::Marker::ARROW;

    marker.action = visualization_msgs::Marker::ADD;

    // Calculate pickup point with consideration of orientation
    Eigen::Quaternionf quat(objQuat[3], objQuat[0], objQuat[1], objQuat[2]);
    std::vector<double> eulerAngles = HelperMethods::getEulerFromQuaternion(quat);
    double yaw = eulerAngles[2];
    ROS_INFO("YAW OF OBJECT = %f", yaw);
    float y_shifted = y + (object.width * 0.275);
    ROS_INFO("Pickup location assuming 0 degree: %f, %f", x, y_shifted);
    marker.pose.position.x = -((y_shifted - y) * std::sin(yaw)) + x;
    marker.pose.position.y = ((y_shifted - y) * std::cos(yaw)) + y;

    ROS_INFO("Estimated pickup location = %f, %f", marker.pose.position.x, marker.pose.position.y);

    marker.pose.position.z = z + 0.025;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;

    // Set marker size
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;

    // Set marker color (blue)
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    // Create marker for the detected corner
    cornerMarker.header.frame_id = "panda_link0";
    cornerMarker.header.stamp = ros::Time::now();

    // Create a RViz marker for the detected corner
    cornerMarker.ns = "obj_cor"; // Namespace for the corner marker
    cornerMarker.id = i * 2;     // Unique ID for this corner marker (different from the object marker)

    cornerMarker.type = visualization_msgs::Marker::SPHERE; // Use sphere shape for corners

    cornerMarker.action = visualization_msgs::Marker::ADD; // Add this marker to the scene

    // Position the marker at the detected corner position
    cornerMarker.pose.position.x = detectedCornerPosition.first;
    cornerMarker.pose.position.y = detectedCornerPosition.second;
    cornerMarker.pose.position.z = z + 0.025; // Slightly above the object surface for visibility

    // Default orientation (no rotation needed for sphere markers)
    cornerMarker.pose.orientation.x = 0.0;
    cornerMarker.pose.orientation.y = 0.0;
    cornerMarker.pose.orientation.z = 0.0;
    cornerMarker.pose.orientation.w = 0.0;

    // Set the size of the sphere marker (2cm diameter)
    cornerMarker.scale.x = 0.02;
    cornerMarker.scale.y = 0.02;
    cornerMarker.scale.z = 0.02;

    // Set the color to red for corner markers
    cornerMarker.color.r = 1.0f;
    cornerMarker.color.g = 0.0f;
    cornerMarker.color.b = 0.0f;
    cornerMarker.color.a = 1.0f; // Fully opaque

    // Add the marker to the array
    markerArray.markers.push_back(marker);
  }

  // Publish all object markers and poses to their respective topics
  objectMarkerPublisher.publish(markerArray);
  objectPosePublisher.publish(poseArray);
}

// Function to publish the point cloud as a ROS message for visualization
void publishCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  sensor_msgs::PointCloud2 rosCloud;

  // Convert PCL cloud to ROS message format
  pcl::toROSMsg(*cloud, rosCloud);
  ROS_INFO("Point Cloud Message: width=%d, height=%d", rosCloud.width, rosCloud.height);

  // Set the coordinate frame for the point cloud
  rosCloud.header.frame_id = "panda_link0";
  ROS_INFO("PointCloud frame ID: %s", rosCloud.header.frame_id.c_str());

  // Publish the point cloud message
  pointCloudPublisher.publish(rosCloud);
}

// Main processing pipeline for the point cloud data
std::vector<ObjectData> processPointCloud()
{
  sensor_msgs::PointCloud2 rosCloud;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

  // Step 1: Filter the cloud by height (remove points too high or too low)
  filterCloud(completeCloud);
  ROS_INFO("Filtered Cloud");

  // Step 2: Calculate surface normals for the filtered cloud
  calcNormals(completeCloud, cloud_normals);
  ROS_INFO("Calculated Normals of Cloud");

  // Create an empty indices container for plane detection
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);

  // Step 3: Remove the planar surface (ground plane)
  removePlaneSurface(completeCloud, cloud_normals, inliers_plane);

  // Step 4: Filter out points based on color (remove green grass and gray areas)
  filterColors(completeCloud);
  ROS_INFO("Removed Plane");

  // Step 5: Publish the processed cloud for visualization
  publishCloud(completeCloud);

  // Step 6: Extract and classify objects from the processed cloud
  std::vector<ObjectData> objects = extractObjectsInScene(completeCloud);

  // Step 7: Publish visualization markers for detected objects
  publishObjectPositions(objects);

  // Return the detected objects
  return objects;
}

// Service function to move the robot arm to a target pose
// Uses Cartesian planning by default, falls back to RRT if needed
bool callSetArmService(const geometry_msgs::Pose &target_pose, bool setArmCart = true)
{
  ROS_INFO("setArmCart is %d", setArmCart);

  if (setArmCart)
  {
    // Try Cartesian path planning first for smoother, more direct movements
    // Wait for the service to be available
    if (!set_arm_cart_client_.waitForExistence(ros::Duration(5.0)))
    {
      ROS_ERROR("Service /cw2/set_arm_cart is not available.");
      return false;
    }

    // Create a service request object for Cartesian planning
    cw2_team_13::set_arm_cart srv;
    srv.request.pose = target_pose; // Set the desired pose

    // Call the Cartesian planning service
    if (set_arm_cart_client_.call(srv))
    {
      return srv.response.success;
    }
    else
    {
      ROS_WARN("Failed to find valid cart path with set_arm_cart service. Fallback to RRT plan");
    }
  }

  // Fall back to RRT planning if Cartesian planning fails or is not requested
  if (!set_arm_client_.waitForExistence(ros::Duration(5.0)))
  {
    ROS_ERROR("Service /cw2/set_arm is not available.");
    return false;
  }

  // Create a service request object for RRT planning
  cw2_team_13::set_arm srv;
  srv.request.pose = target_pose; // Set the desired pose

  // Call the RRT planning service
  if (set_arm_client_.call(srv))
  {
    ROS_INFO("Service call to set arm successful?: %s", srv.response.success ? "true" : "false");
    return srv.response.success;
  }
  else
  {
    ROS_ERROR("Failed to call set_arm service.");
    return false;
  }
}

// Function to scan the environment from multiple viewpoints
bool getScans(int taskId)
{
  // Set up transform listener to get camera-to-base transformations
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener transformListner(tfBuffer);

  // Define base scan pose (forward-looking position)
  geometry_msgs::Pose basePose;
  basePose.position.x = 0.45;                     // Forward of the robot
  basePose.position.y = 0.0;                      // Center
  basePose.position.z = 0.75;                     // Camera height
  double roll = M_PI, pitch = 0, yaw = -M_PI / 4; // Camera orientation
  std::vector<double> quaternionPose = HelperMethods::getQuaternionFromEuler(roll, pitch, yaw);
  basePose.orientation.x = quaternionPose[0];
  basePose.orientation.y = quaternionPose[1];
  basePose.orientation.z = quaternionPose[2];
  basePose.orientation.w = quaternionPose[3];

  // Define left side scan position
  geometry_msgs::Pose leftScan = basePose;
  leftScan.position.y = -0.3; // 30cm to the left

  // Define right side scan position
  geometry_msgs::Pose rightScan = basePose;
  rightScan.position.y = 0.3; // 30cm to the right

  // Define left-middle-right quadrant scan position
  geometry_msgs::Pose leftMiddleRightScan = leftScan;
  std::vector<double> quaternionLeftPose = HelperMethods::getQuaternionFromEuler(roll, pitch, 5 * M_PI / 4);
  leftMiddleRightScan.position.x = -0.2; // Back-left position
  leftMiddleRightScan.position.y -= 0.1; // More to the left
  // Set the orientation for this scan position
  leftMiddleRightScan.orientation.x = quaternionLeftPose[0];
  leftMiddleRightScan.orientation.y = quaternionLeftPose[1];
  leftMiddleRightScan.orientation.z = quaternionLeftPose[2];
  leftMiddleRightScan.orientation.w = quaternionLeftPose[3];

  // Define left-middle-left quadrant scan position
  geometry_msgs::Pose leftMiddleLeftScan = leftMiddleRightScan;
  leftMiddleLeftScan.position.x = 0.2; // Forward-left position

  // Define right-middle-right quadrant scan position
  geometry_msgs::Pose rightMiddleRightScan = rightScan;
  std::vector<double> quaternionrightPose = HelperMethods::getQuaternionFromEuler(roll, pitch, M_PI / 4);
  rightMiddleRightScan.position.x = -0.2; // Back-right position
  rightMiddleRightScan.position.y += 0.1; // More to the right
  // Set the orientation for this scan position
  rightMiddleRightScan.orientation.x = quaternionrightPose[0];
  rightMiddleRightScan.orientation.y = quaternionrightPose[1];
  rightMiddleRightScan.orientation.z = quaternionrightPose[2];
  rightMiddleRightScan.orientation.w = quaternionrightPose[3];

  // Define right-middle-left quadrant scan position
  geometry_msgs::Pose rightMiddleLeftScan = rightMiddleRightScan;
  rightMiddleLeftScan.position.x = 0.2; // Forward-right position

  // Define right-back scan position
  geometry_msgs::Pose rightBackScan = rightMiddleLeftScan;
  std::vector<double> quaternionrightBackPose = HelperMethods::getQuaternionFromEuler(roll, pitch, 3 * M_PI / 4);
  rightBackScan.position.x = -0.3; // Further back position
  // Set the orientation for this scan position
  rightBackScan.orientation.x = quaternionrightBackPose[0];
  rightBackScan.orientation.y = quaternionrightBackPose[1];
  rightBackScan.orientation.z = quaternionrightBackPose[2];
  rightBackScan.orientation.w = quaternionrightBackPose[3];

  // Define back-center scan position
  geometry_msgs::Pose backLeftScan = rightBackScan;
  backLeftScan.position.y = 0.0; // Center position behind robot

  // Define back-left scan position
  geometry_msgs::Pose backScan = backLeftScan;
  backScan.position.y = -0.3; // Left position behind robot

  // Select scan positions based on task ID
  std::vector<geometry_msgs::Pose> scanPoses;

  if (taskId == 1)
  {
    // Fewer positions for Task 1 (basic pick and place)
    scanPoses = {leftMiddleLeftScan, leftScan, basePose, rightScan, rightMiddleLeftScan};
  }
  else if (taskId == 2)
  {
    // Medium coverage for Task 2 (shape detection)
    scanPoses = {leftScan, basePose, rightScan, rightBackScan, backLeftScan, backScan};
  }
  else
  {
    // Most comprehensive coverage for Task 3 (multiple objects and obstacles)
    scanPoses = {leftMiddleRightScan, leftMiddleLeftScan, leftScan, basePose, rightScan, rightMiddleLeftScan, rightMiddleRightScan, rightBackScan, backLeftScan, backScan};
  }

  // Set up voxel grid filter to downsample point clouds
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setLeafSize(0.0025f, 0.0025f, 0.0025f); // 2.5mm voxel size
  ROS_INFO("Preparing to scan");

  // Perform scans from each position
  for (size_t i = 0; i < scanPoses.size(); i++)
  {
    // Move arm to the next scan position
    if (callSetArmService(scanPoses[i]))
    {
      ros::Duration(2.0).sleep(); // Wait for arm to stabilize
      ROS_INFO("Moving to scan position");

      // Get transform from camera frame to robot base frame
      geometry_msgs::TransformStamped transformStamped;
      transformStamped = tfBuffer.lookupTransform("panda_link0", "color", ros::Time(0), ros::Duration(2.0));

      // Convert transform to Eigen format for PCL transformations
      Eigen::Affine3d transformEigen = tf2::transformToEigen(transformStamped);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

      // Copy points from current sensor cloud to working cloud
      for (const auto &point : cloud->points)
      {
        pcl::PointXYZRGB newPoint;
        newPoint = point;
        currentCloud->points.push_back(newPoint);
      }

      // Downsample the current cloud with voxel grid filter
      sor.setInputCloud(currentCloud);
      sor.filter(*currentCloud);

      // Transform the cloud to the robot base frame
      pcl::transformPointCloud(*currentCloud, *transformedCloud, transformEigen);

      // Add the transformed cloud to the complete cloud
      *completeCloud += *transformedCloud;

      // Publish the updated cloud for visualization
      publishCloud(completeCloud);
    }
  }

  // Convert complete cloud to ROS message format
  sensor_msgs::PointCloud2 pointCloudRos;
  pcl::toROSMsg(*completeCloud, pointCloudRos);
  pointCloudRos.header.frame_id = "panda_link0";

  return true;
}

// ROS service callback to map the environment and identify objects
bool mapEnvironment(cw2_team_13::map_env::Request &req, cw2_team_13::map_env::Response &res)
{
  ROS_INFO("Map Environment Called");

  // Clear any existing data
  completeCloud->clear();
  cloud->clear();
  ROS_INFO("Cleared Markers");

  // Step 1: Perform multiple scans based on task ID
  bool scansSuccessful = getScans(req.taskId);
  ROS_INFO("Scans completed: %s", scansSuccessful ? "true" : "false");

  // Step 2: Process the point cloud to detect objects
  std::vector<ObjectData> objects = processPointCloud();

  // Step 3: Convert detected objects to ROS message format for response
  for (size_t i = 0; i < objects.size(); i++)
  {
    cw2_team_13::ObjectInfo objInfo;

    ObjectData object = objects[i];

    Eigen::Vector3f location = object.objPointInCartesianSpace;
    Eigen::Vector4f orientation = object.objectOrientation;
    Eigen::Vector3i rgbValue = object.rgbValue;

    // Fill position information in the response
    objInfo.position.x = location[0];
    objInfo.position.y = location[1];
    objInfo.position.z = location[2];

    // Fill orientation information in the response
    objInfo.orientation.x = orientation[0];
    objInfo.orientation.y = orientation[1];
    objInfo.orientation.z = orientation[2];
    objInfo.orientation.w = orientation[3];

    // Set object dimensions
    objInfo.width = object.width;
    objInfo.height = object.height;

    // Convert RGB values from 0-255 to 0.0-1.0 float range
    objInfo.color.r = static_cast<float>(rgbValue[0]) / 255.0f;
    objInfo.color.g = static_cast<float>(rgbValue[1]) / 255.0f;
    objInfo.color.b = static_cast<float>(rgbValue[2]) / 255.0f;
    objInfo.color.a = 1.0f; // Fully opaque

    // Set object type
    objInfo.objectType = object.objType;

    // Add this object to the response array
    res.objects.push_back(objInfo);
  }

  // Mark the service call as successful
  res.success = true;
  return true;
}

// Main entry point for the pointcloud_node
int main(int argc, char **argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "pointcloud_node");
  ros::NodeHandle nh;

  ROS_INFO("Setting PointCloud node up");

  // Initialize service clients for arm movement
  set_arm_cart_client_ = nh.serviceClient<cw2_team_13::set_arm_cart>("/cw2/set_arm_cart");
  set_arm_client_ = nh.serviceClient<cw2_team_13::set_arm>("/cw2/set_arm");

  // Subscribe to the RealSense point cloud topic
  ros::Subscriber realSenseSub = nh.subscribe("r200/camera/depth_registered/points", 1, realSenseCallback);

  // Set up publishers for visualization
  pointCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("pclPoints", 1);
  objectMarkerPublisher = nh.advertise<visualization_msgs::MarkerArray>("objectPositions", 1);
  objectPosePublisher = nh.advertise<geometry_msgs::PoseArray>("objectPoses", 1);

  // Advertise the environment mapping service
  ros::ServiceServer mapService = nh.advertiseService("cw2/map_env", &mapEnvironment);

  // Get the package path (not used in this file)
  std::string pkg_path = ros::package::getPath("cw2_team_13");

  // Use AsyncSpinner for non-blocking operation
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("Spun");

  // Main loop at 10Hz
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();   // Process callbacks
    loop_rate.sleep(); // Maintain the loop rate
  }
  return 0;
}
