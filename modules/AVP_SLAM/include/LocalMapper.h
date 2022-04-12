#pragma once

#include "opencv2/core.hpp"
#include "hdMap.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/conversions.h"
#include "pcl/registration/icp.h"

class LocalMapper
{
public:
    LocalMapper();
    void update(pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud, cv::Point3f carPose);

private:
    void showLocalMap();
    void drawMatchCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr CloudTarget, pcl::PointCloud<pcl::PointXYZ>::Ptr CloudSrc);

private:
    static unsigned int index;
    unsigned int nId;
    
    bool bFirst;
    bool bFinish;
    cv::Point3f pose;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mpLocalMap;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mpLastFrame;
    cv::Point3f lastCarPose;

    int max_points;

    cv::Mat mImMap;
    MapParam mMapParam;
};