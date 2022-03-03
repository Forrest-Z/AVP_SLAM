#pragma once

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "MapStruct.h"
#include "semanticStruct.h"
#include "semanticMsg.hpp"
#include <lcm/lcm-cpp.hpp>

#include <string>
#include <fstream>
#include <iostream>
#include <mutex>
#include <set>

namespace AVP_SIM
{
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

    // Map类，保存高精度地图信息
    class Map
    {
    public:
        Map();
        // 根据输入的位置返回bev图 
        cv::Mat bevSensing(const cv::Point3f &CarPose);
        cv::Mat semanticSeg(const cv::Point3f &CarPose);

        void update(const cv::Point3f &CarPose);

    private:
        void readSemantic(std::string &path2Semantic);
        // void extractEuclideanClusters(std::string &path2Semantic);

        void drawOnMap();
        void drawLocal(const cv::Point3f &CarPose, int width);
        void drawGlobal(const cv::Point3f &CarPose);
        cv::Mat canBeSeen(cv::Mat &Twb, cv::Point2f point);

    public:
        cv::Mat mImage;
        cv::Mat mImageWithSemantic;
        cv::Mat mImageWithAllSemantic;

        cv::Mat mImageGlobal;
        cv::Mat mImageLocal;
        cv::Mat mFreeSpace;
        MapParam mMapParam;

        std::mutex mMutexLocal, mMutexGlobal;

        //semantic information
        std::vector<SemanticInformation> mvSemantic;                // 库位点
        std::vector<DottedLine> mvDottedLine;                       // 直线
        std::vector<StraightArrow> mvStraightArrow;                 // 直线箭头
        std::vector<ArrowTurns> mvArrowTurns;                       // 转弯箭头
        std::vector<StraightTurningArrow> mvStraightTurningArrow;   // 直线转弯箭头
        std::vector<DoubleArrow> mvDoubleArrow;                     // 双转弯箭头

        //lcm
        lcm::LCM* _lcm;
        lcm::LogFile* _lcmlogger;
    };
}