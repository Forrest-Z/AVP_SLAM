#pragma once

#include "hdMap.h"
#include "pcl/point_cloud.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/conversions.h"
#include "pcl/registration/icp.h"
#include "pcl/io/pcd_io.h"
#include "opencv2/core.hpp"


#include <string>
#include <vector>
using namespace std;

class Localizer
{
public:
    Localizer();
    cv::Vec3f update(pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud, PoseData carPose);

private:
    void readMap();
    void icp(pcl::PointCloud<pcl::PointXYZ>::Ptr CloudTarget, pcl::PointCloud<pcl::PointXYZ>::Ptr CloudSrc, cv::Point3f odom);
    void showLocal(const cv::Mat &im, cv::Point3f &carpose, int width, int time);

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr mpMap;
    MapParam mMapParam;
    cv::Mat mImMap;

    float x_estimated = 0;
    float y_estimated = 0;
    float z_estimated = 0;
};