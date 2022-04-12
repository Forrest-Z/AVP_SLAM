#include "hdMap.h"
#include "Motion.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

#include "pcl/io/pcd_io.h"

#include <iostream>
#include <string>
#include <fstream>
using namespace std;

MapParam param;
cv::Mat imPath;

void drawGlobal(const cv::Point3f &CarPose, cv::Scalar color);
void drawLocal(const cv::Point3f &CarPose, int width);
void drawObserved(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud, const cv::Point3f &CarPose, int width);


int main()
{
    std::cout << "AVP_SIM : use key up down left right to control car move" << std::endl;

    hdMap* pHdMap = hdMap::Instance();

    Motion motion(0,0,0);
    imPath = pHdMap->mImMap.clone();
    param = pHdMap->getMapParam();

    cv::Point3f last_pose(0,0,0);
    ofstream fpose("../data/pose.bin");
    
    while (true)
    {
        cv::namedWindow("path", cv::WINDOW_NORMAL);
        cv::imshow("path", imPath);
        int nkey = cv::waitKey(5);
        if (nkey == 115)
        {
            break;
        }
        
        motion.update(nkey);
        cv::Point3f carPoseNoised = motion.getCarPoseNoised();
        cv::Point3f carPose = motion.getCarPose();


        if (sqrt( pow((carPose.x-last_pose.x),2)+pow((carPose.y-last_pose.y),2)) == 0.0 && sqrt(pow((carPose.z-last_pose.z),2)) == 0.0)
        {
            continue;
        }
        last_pose = carPose;

        static int cloud_index = 0;
        drawGlobal(carPoseNoised, cv::Scalar(255,0,0));
        drawGlobal(carPose, cv::Scalar(0,255,0));

        drawLocal(carPose,400);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud = pHdMap->observe(carPose);
        drawObserved(pcloud, carPose, 400);

        //save pointcloud and pose (crash when no data!)
        std::string filename = "../data/" + std::to_string(cloud_index) + ".pcd";
        pcl::PCDWriter writer;
        writer.write(filename, *pcloud);
        cloud_index++;
        PoseData pose_temp;
        pose_temp.x_true = carPose.x;
        pose_temp.y_true = carPose.y;
        pose_temp.z_true = carPose.z;
        pose_temp.x_noised = carPoseNoised.x;
        pose_temp.y_noised = carPoseNoised.y;
        pose_temp.z_noised = carPoseNoised.z;
        fpose.write((char*)&pose_temp, sizeof(PoseData));

        // pcl::visualization::PCLVisualizer viewer("semantics");
        // viewer.addPointCloud(pcloud);
        // viewer.spin();
    }
    fpose.close();
}

// void drawObserved(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud, const cv::Point3f &CarPose, int width)
// {
//     cv::Mat imTemp = imPath.clone();
//     for (size_t i = 0; i < pCloud->size(); i++)
//     {
//         pcl::PointXYZ temp = pCloud->points[i];
//         int col = (temp.x - param.OriginX) / param.Res;
//         int row = (temp.y - param.OriginY) / param.Res;
//         cv::circle(imTemp, cv::Point(col, row), 2, cv::Scalar(255,0,0), -1);
//     }
    
//     int row_center = (CarPose.y - param.OriginY) / param.Res;
//     int col_center = (CarPose.x - param.OriginX) / param.Res;

//     int row1, row2, col1, col2;
//     (row_center - width) <= 0 ? row1 = 0 : row1 = (row_center - width);
//     (row_center + width) >= imPath.rows ? row2 = imPath.rows : row2 = (row_center + width);
//     (col_center - width) <= 0 ? col1 = 0 : col1 = (col_center - width);
//     (col_center + width) >= imPath.cols ? col2 = imPath.cols : col2 = (col_center + width);

//     cv::Mat temp = imTemp.clone();
//     cv::Mat LocalMap = cv::Mat(width * 2, width * 2, imPath.type());
//     temp(cv::Rect(cv::Point2f(col1, row1), cv::Point2f(col2, row2))).copyTo(LocalMap);
//     cv::imshow("LocalObs", LocalMap);
// }

void drawObserved(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud, const cv::Point3f &CarPose, int width)
{
    std::cout << "Cloud size: " << pCloud->size() << std::endl;
    cv::Mat im(400, 400, CV_32FC3, cv::Scalar(0,0,0));
    for (size_t i = 0; i < pCloud->size(); i++)
    {
        pcl::PointXYZ temp = pCloud->points[i];
        // std::cout << "x: " << temp.x << " y: " << temp.y << std::endl;
        int col = (temp.x+10) / param.Res;
        int row = (temp.y+10) / param.Res;
        cv::circle(im, cv::Point(col, row), 1, cv::Scalar(0,255,0), -1);
    }
    cv::namedWindow("LocalObs", cv::WINDOW_AUTOSIZE);
    cv::imshow("LocalObs", im);
}


void drawGlobal(const cv::Point3f &CarPose, cv::Scalar color)
{
    //draw on Map
    float _x = CarPose.x;
    float _y = CarPose.y;
    int col = (_x - param.OriginX) / param.Res;
    int row = (_y - param.OriginY) / param.Res;
    
    cv::circle(imPath, cv::Point2f(col, row), 1, color, -1);
}

void drawLocal(const cv::Point3f &CarPose, int width)
{
    
    int row_center = (CarPose.y - param.OriginY) / param.Res;
    int col_center = (CarPose.x - param.OriginX) / param.Res;

    int row1, row2, col1, col2;
    (row_center - width) <= 0 ? row1 = 0 : row1 = (row_center - width);
    (row_center + width) >= imPath.rows ? row2 = imPath.rows : row2 = (row_center + width);
    (col_center - width) <= 0 ? col1 = 0 : col1 = (col_center - width);
    (col_center + width) >= imPath.cols ? col2 = imPath.cols : col2 = (col_center + width);

    cv::Mat temp = imPath.clone();

    //draw car
    float x0 = CarPose.x;
    float y0 = CarPose.y;
    float x1 = 1.5*cos(CarPose.z) + x0;
    float y1 = 1.5*sin(CarPose.z) + y0;

    int col_head = (x1 - param.OriginX) / param.Res;
    int row_head = (y1 - param.OriginY) / param.Res;

    cv::circle(temp, cv::Point2f(col_center, row_center), 161, cv::Scalar(255,0,0));
    cv::arrowedLine(temp, cv::Point2f(col_center, row_center), cv::Point2f(col_head, row_head), cv::Scalar(255,0,0));

    float xfl = 1.5*cos(CarPose.z - 0.5) + x0;
    float yfl = 1.5*sin(CarPose.z - 0.5) + y0;
    int col_fl = (xfl - param.OriginX) / param.Res;
    int row_fl = (yfl - param.OriginY) / param.Res;
    cv::circle(temp, cv::Point2f(col_fl, row_fl), 2, cv::Scalar(255,0,0), -1);

    float xfr = 1.5*cos(CarPose.z + 0.5) + x0;
    float yfr = 1.5*sin(CarPose.z + 0.5) + y0;
    int col_fr = (xfr - param.OriginX) / param.Res;
    int row_fr = (yfr - param.OriginY) / param.Res;
    cv::circle(temp, cv::Point2f(col_fr, row_fr), 2, cv::Scalar(255,0,0), -1);

    float xbr = -1.5*cos(CarPose.z + 0.5) + x0;
    float ybr = -1.5*sin(CarPose.z + 0.5) + y0;
    int col_br = (xbr - param.OriginX) / param.Res;
    int row_br = (ybr - param.OriginY) / param.Res;
    cv::circle(temp, cv::Point2f(col_br, row_br), 2, cv::Scalar(255,0,0), -1);

    float xbl = -1.5*cos(CarPose.z - 0.5) + x0;
    float ybl = -1.5*sin(CarPose.z - 0.5) + y0;
    int col_bl = (xbl - param.OriginX) / param.Res;
    int row_bl = (ybl - param.OriginY) / param.Res;
    cv::circle(temp, cv::Point2f(col_bl, row_bl), 2, cv::Scalar(255,0,0), -1);

    cv::Mat LocalMap = cv::Mat(width * 2, width * 2, imPath.type());
    temp(cv::Rect(cv::Point2f(col1, row1), cv::Point2f(col2, row2))).copyTo(LocalMap);
    cv::imshow("LocalMap", LocalMap);
}