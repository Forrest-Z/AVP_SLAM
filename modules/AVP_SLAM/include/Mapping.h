#pragma once

#include "hdMap.h"
#include "pcl/point_cloud.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/conversions.h"
#include "pcl/registration/icp.h"
#include "pcl/io/pcd_io.h"

#include "Eigen/Dense"
#include "Eigen/Core"
#include <string>

class Mapping
{
public:
    Mapping();
    void update(pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud, PoseData carPose);
    void saveMap(const std::string filename);

    void icp(pcl::PointCloud<pcl::PointXYZ>::Ptr CloudTarget, pcl::PointCloud<pcl::PointXYZ>::Ptr CloudSrc, cv::Point3f odom);
    void icp_map(pcl::PointCloud<pcl::PointXYZ>::Ptr CloudTarget, pcl::PointCloud<pcl::PointXYZ>::Ptr CloudSrc, cv::Point3f odom);


private:
    void drawMatchCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr CloudTarget, pcl::PointCloud<pcl::PointXYZ>::Ptr CloudSrc);

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr mpMap;
    pcl::PointCloud<pcl::PointXYZ> mLastCloud;

    cv::Mat mImMap;
    MapParam mMapParam;

    float x_estimated = 0;
    float y_estimated = 0;
    float z_estimated = 0;
};
