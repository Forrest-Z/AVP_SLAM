#pragma once

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <set>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/search/kdtree.h"
#include "pcl/io/io.h"

// 地图参数
struct MapParam
{
    float OriginX; //原点的像素位置
    float OriginY;
    float Res; // 分辨率
    //uchar CentreCost;
    unsigned char CentreCost;
    int SizeX;
    int SizeY;
};

typedef struct 
{
    float x_noised, y_noised, z_noised;
    float x_true, y_true, z_true;
}PoseData;


class hdMap
{
public:
    static hdMap* Instance();
    MapParam getMapParam(){return mMapParam;}
    pcl::PointCloud<pcl::PointXYZ>::Ptr observe(cv::Point3f carPose);

protected:
    hdMap();

private:
    // void readSemantic(const cv::Mat & ImSemantic);
    void readSemanticPCL(const cv::Mat & ImSemantic);


private:
    static hdMap* _instance;
    MapParam mMapParam;
    // std::vector<pointType> mvSemantics;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mpSemanticsPCL;
    pcl::search::KdTree<pcl::PointXYZ> mpKDtree;

    float nPixel;
public:
    cv::Mat mImMap;
};