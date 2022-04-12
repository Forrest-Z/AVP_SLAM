#include "hdMap.h"
#include "Mapping.h"
#include "LocalMapper.h"
#include "pcl/io/pcd_io.h"

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
using namespace std;

MapParam param;

void read(vector<pair<string, PoseData>>& data)
{
    data.clear();
    ifstream fin("../data/pose.bin", ios::binary);
    while (true)
    {
        if (fin.peek() == EOF)
            break;
        
        PoseData temp;
        fin.read((char*)&temp, sizeof(temp));
        static int cloud_index = 0;
        string filename = "../data/" + to_string(cloud_index) + ".pcd";
        data.push_back(make_pair(filename, temp));
        cloud_index++;
    }
    fin.close();
    std::cout << "data size: " << data.size() << std::endl;
}

void drawObserved(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud)
{
    std::cout << "Cloud size: " << pCloud->size() << std::endl;
    cv::Mat im(400, 400, CV_32FC3, cv::Scalar(0,0,0));
    for (size_t i = 0; i < pCloud->size(); i++)
    {
        pcl::PointXYZ temp = pCloud->points[i];
        // std::cout << "x: " << temp.x << " y: " << temp.y << std::endl;
        int col = (temp.x+10) / param.Res;
        int row = (temp.y+10) / param.Res;
        cv::circle(im, cv::Point(col, row), 2, cv::Scalar(0,255,0), -1);
    }
    cv::namedWindow("LocalObs", cv::WINDOW_AUTOSIZE);
    cv::imshow("LocalObs", im);
    cv::waitKey(5);
}

int main()
{
    // read data and map param
    vector<pair<string, PoseData>> vData;
    read(vData);

    std::string path2MapParam = "../Examples/map_param.bin";
    std::ifstream fin0(path2MapParam.c_str(), std::ifstream::in | std::ifstream::binary);
    if (fin0)
    {
        fin0.read((char*)&param, sizeof(param));
        fin0.close();
    }

    Mapping mapper;
    LocalMapper localmapper;
    

    // mapping 
    for (size_t i = 0; i < vData.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(vData[i].first, *cloud) == -1) 
        {
            PCL_ERROR("Couldn't read file rabbit.pcd\n");
            return(-1);
        }
        
        // see observerd feature points
        // drawObserved(cloud);

        mapper.update(cloud, vData[i].second);
    //     cv::Point3f carpose;
    //     carpose.x = vData[i].second.x_noised;
    //     carpose.y = vData[i].second.y_noised;
    //     carpose.z = vData[i].second.z_noised;

    //     localmapper.update(cloud, carpose);
    }
    mapper.saveMap("../Examples/map.pcd");
}